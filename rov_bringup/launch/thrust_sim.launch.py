# Copyright 2026 sam
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Start a thrust-driven simulator of the V4 ROV.

Both backends consume thruster_command (rov_interfaces/ThrusterCommand),
publish odom and the odom -> base_link TF, and offer the reset_vehicle
service (std_srvs/Trigger), so the same tests and controllers work with
either one. Nothing here drives the ROV: send thruster commands yourself,
for example with
    ros2 run rov_simulation open_loop_thrust_test.py --backend-label dynamics

backend:=dynamics (default): dynamics_sim_node, the lightweight RK4 model
    (rov_simulation). The odom frame matches the Gazebo world frame below,
    so both backends report the same coordinates.
backend:=gazebo: Gazebo Harmonic runs the pool world (worlds/pool.sdf)
    with the thrust-driven V4 model (models/rov_v4_thrust.sdf.template).
    PlanarThrusterSystem applies the thrust, Gazebo's Hydrodynamics system
    the damping, and the physics engine the added mass. ros_gz_bridge brings
    back /clock, /odom and the TF (Gazebo is the only pose source) and
    gazebo_thruster_relay forwards thruster_command and reset_vehicle.
    gui:=false runs the Gazebo server without the window.

start_x:=, start_y:=, start_yaw:= (m, m, rad): start pose in the pool
world frame (origin at the pool's bottom-left corner). reset_vehicle
returns the ROV here. Default (5, 5, 0): 5 m from both walls, facing +x.

tracking:=true also starts the closed loop of implementation step 5 (the
temporary Path -> trajectory converter was retired in step 8B: send
FollowTrajectory goals directly, e.g. with the scenario runner):
    trajectory_controller_node (FollowTrajectory -> wrench_command),
    thrust_allocator_node (wrench_command -> thruster_command), and a static
    map -> odom transform. The map is centred on the pool like
    maps/pool_map.yaml, so map -> odom is (-25, -12.5). Send a
    FollowTrajectory goal in the map frame to /follow_trajectory.
"""

import atexit
import os
from string import Template
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

# Pool world vs. the pool-centred map (as in sim_nav.launch.py):
# world (x, y) = map (x, y) + POOL_MAP_TO_WORLD.
POOL_MAP_TO_WORLD = (25.0, 12.5)


def load_yaml(path):
    with open(path, 'r') as stream:
        return yaml.safe_load(stream)


def render_rov_model(bringup_share, sim_config, current=(0.0, 0.0)):
    """Fill the model template with values from the parameter YAML files."""
    description_config = os.path.join(get_package_share_directory('rov_description'), 'config')
    thruster_config = os.path.join(description_config, 'thrusters_v4.yaml')
    hydro = load_yaml(os.path.join(description_config, 'hydrodynamics_v4.yaml'))
    sim = load_yaml(sim_config)['dynamics_sim_node']['ros__parameters']
    values = {
        'mass': hydro['mass'],
        'inertia_z': hydro['inertia_z'],
        # <fluid_added_mass> uses positive added mass: -X_udot, ...
        'added_mass_x': -hydro['added_mass']['x_udot'],
        'added_mass_y': -hydro['added_mass']['y_vdot'],
        'added_mass_yaw': -hydro['added_mass']['n_rdot'],
        'x_u': hydro['linear_damping']['x_u'],
        'y_v': hydro['linear_damping']['y_v'],
        'n_r': hydro['linear_damping']['n_r'],
        'x_uu': hydro['quadratic_damping']['x_uu'],
        'y_vv': hydro['quadratic_damping']['y_vv'],
        'n_rr': hydro['quadratic_damping']['n_rr'],
        'thruster_config': thruster_config,
        'command_timeout': sim['cmd_timeout_sec'],
        'current_x': current[0],
        'current_y': current[1],
    }
    # Provisional hydrostatics (gravity and buoyancy) for the Gazebo backend.
    values.update(load_yaml(os.path.join(
        get_package_share_directory('rov_simulation'), 'config', 'gazebo_hydrostatics.yaml')))
    template_path = os.path.join(bringup_share, 'models', 'rov_v4_thrust.sdf.template')
    with open(template_path, 'r') as stream:
        # substitute() raises KeyError if a placeholder has no value.
        return Template(stream.read()).substitute(values)


def hydrostatics():
    """Return the provisional Gazebo hydrostatic settings from gazebo_hydrostatics.yaml."""
    return load_yaml(os.path.join(
        get_package_share_directory('rov_simulation'), 'config', 'gazebo_hydrostatics.yaml'))


def float_height():
    """Return the base_link height (m) at which the ROV floats (spawn there)."""
    hydro = load_yaml(os.path.join(
        get_package_share_directory('rov_description'), 'config', 'hydrodynamics_v4.yaml'))
    h = hydrostatics()
    draft = hydro['mass'] / (h['water_density'] * h['waterplane_area'])
    return h['bottom_below_origin'] - draft


def gravity_world(world_file):
    """Return a copy of the pool world with gravity on (the original stays gravity-free)."""
    with open(world_file, 'r') as stream:
        text = stream.read()
    if '<gravity>0 0 0</gravity>' not in text:
        raise RuntimeError(f'{world_file}: no zero-gravity line to replace')
    text = text.replace('<gravity>0 0 0</gravity>', '<gravity>0 0 -9.81</gravity>')
    handle, path = tempfile.mkstemp(prefix='pool_gravity_', suffix='.sdf')
    with os.fdopen(handle, 'w') as stream:
        stream.write(text)
    # Gazebo reads the world at startup; remove the copy when launch exits.
    atexit.register(lambda: os.path.exists(path) and os.remove(path))
    return path


def parse_current(text):
    """Return the water current (x, y) in m/s from 'x,y' (world frame)."""
    values = [float(v) for v in text.split(',')]
    if len(values) != 2:
        raise RuntimeError(f"water_current must be 'x,y' in m/s, not '{text}'")
    return tuple(values)


def dynamics_actions(sim_config, start, current=(0.0, 0.0)):
    return [
        Node(
            package='rov_simulation',
            executable='dynamics_sim_node',
            output='screen',
            parameters=[sim_config, {'initial_pose': list(start),
                                     'water_current': list(current)}],
        ),
    ]


def gazebo_actions(bringup_share, sim_config, start, gui, current=(0.0, 0.0)):
    world_file = gravity_world(os.path.join(bringup_share, 'worlds', 'pool.sdf'))
    bridge_config = os.path.join(bringup_share, 'config', 'gazebo_thrust_bridge.yaml')
    # .../install/rov_description/share: resolves model://rov_description/...
    description_share_parent = os.path.dirname(get_package_share_directory('rov_description'))
    # The PlanarThrusterSystem plugin library is installed in rov_simulation/lib.
    plugin_dir = os.path.join(get_package_prefix('rov_simulation'), 'lib')
    gz_args = f'-r -v 3 {world_file}' + ('' if gui else ' -s')

    return [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', description_share_parent),
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_dir),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_rov',
            output='screen',
            arguments=[
                '-world', 'pool', '-name', 'rov_v4',
                '-string', render_rov_model(bringup_share, sim_config, current),
                '-x', str(start[0]), '-y', str(start[1]), '-z', str(float_height()),
                '-Y', str(start[2]),
            ],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gazebo_thrust_bridge',
            output='screen',
            parameters=[{'config_file': bridge_config}],
        ),
        Node(
            package='rov_simulation',
            executable='gazebo_thruster_relay',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ]


def tracking_actions(use_sim_time):
    control_config = os.path.join(get_package_share_directory('rov_control'), 'config')
    sim_time = {'use_sim_time': use_sim_time}
    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['--x', str(-POOL_MAP_TO_WORLD[0]), '--y', str(-POOL_MAP_TO_WORLD[1]),
                       '--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[sim_time],
        ),
        Node(
            package='rov_control',
            executable='thrust_allocator_node',
            output='screen',
            parameters=[os.path.join(control_config, 'thrust_allocator.yaml'), sim_time],
        ),
        Node(
            package='rov_control',
            executable='trajectory_controller_node',
            output='screen',
            parameters=[os.path.join(control_config, 'trajectory_controller.yaml'), sim_time],
        ),
    ]


def launch_setup(context):
    backend = LaunchConfiguration('backend').perform(context)
    start = tuple(float(LaunchConfiguration(name).perform(context))
                  for name in ('start_x', 'start_y', 'start_yaw'))
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    bringup_share = get_package_share_directory('rov_bringup')
    sim_config = os.path.join(
        get_package_share_directory('rov_simulation'), 'config', 'dynamics_sim.yaml')

    current = parse_current(LaunchConfiguration('water_current').perform(context))
    if backend == 'dynamics':
        actions = dynamics_actions(sim_config, start, current)
    elif backend == 'gazebo':
        actions = gazebo_actions(bringup_share, sim_config, start, gui, current)
    else:
        raise RuntimeError(f"backend must be 'dynamics' or 'gazebo', not '{backend}'")
    if LaunchConfiguration('tracking').perform(context).lower() == 'true':
        actions += tracking_actions(use_sim_time=(backend == 'gazebo'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='dynamics',
                              description='dynamics (lightweight RK4 model) or gazebo'),
        DeclareLaunchArgument('start_x', default_value='5.0',
                              description='start x in the pool world frame (m)'),
        DeclareLaunchArgument('start_y', default_value='5.0',
                              description='start y in the pool world frame (m)'),
        DeclareLaunchArgument('start_yaw', default_value='0.0',
                              description='start heading (rad)'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='gazebo only: false runs the server only'),
        DeclareLaunchArgument('water_current', default_value='0.0,0.0',
                              description="constant water current 'x,y' in the world frame (m/s)"),
        DeclareLaunchArgument('tracking', default_value='false',
                              description='true: also start the trajectory tracking loop'),
        OpaqueFunction(function=launch_setup),
    ])
