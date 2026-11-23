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
Start the simulation demo in one of three modes.

All modes also start robot_state_publisher, which shares the ROV model
(rov_description) with RViz2.

Default (no arguments): click-to-drive in a straight line.
    sim_node, controller_node, mission_node, RViz2 (Fixed Frame: odom).

planning_preview:=true: Nav2 global planning only, the robot stays still.
    sim_node, mission_node, RViz2 (Fixed Frame: map), a static map -> odom
    transform, and Nav2's map_server, planner_server, and lifecycle_manager.

planned_drive:=true: plan with Nav2, then drive along the planned route.
    Everything from planning_preview, plus controller_node.

map_yaml:=<file> picks the map for the two map-based modes. A plain file
name (for example pool_map.yaml) is looked up in the installed
rov_bringup/maps folder; a path containing '/' is used as given. When it is
not set, the wall test map preview_map.yaml is used. If rviz/<map name>.rviz
exists (for example pool_map.rviz), RViz2 opens with that view.

start_x:=, start_y:=, start_yaw:= (metres, radians) set where the robot
starts on the map in the two map-based modes. Each map has a default start
(see DEFAULT_START below); any value given here overrides it.

backend:=sim (default) or backend:=gazebo picks what moves the robot:
  sim: sim_node integrates /cmd_vel. The start pose is the static
      map -> odom transform, so odometry starts at (0, 0).
  gazebo: Gazebo Harmonic runs the pool world (worlds/pool.sdf) with the V4
      model, and sim_node is not started. ros_gz_bridge sends /cmd_vel to
      Gazebo and brings back /clock, /odom and odom -> base_link, so Gazebo
      is the only odometry source and every node uses Gazebo's clock.
      The odom frame is the Gazebo world frame (origin at the pool's
      bottom-left corner), while the map is centred on the pool, so
      map -> odom is the fixed offset -POOL_MAP_TO_WORLD. Needs
      planned_drive:=true and the pool map (used by default).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Default start pose (x, y, yaw) on each map, by map file name without
# ".yaml". Maps not listed here start at (0, 0, 0).
# pool_map: bottom-left corner of the pool, 1 m from both walls (the walls
# are at x = -25 m and y = -12.5 m), facing along the pool (+x).
DEFAULT_START = {
    'pool_map': (-24.0, -11.5, 0.0),
}

# Gazebo pool world vs. pool_map.yaml: same axes, different origin.
# The map is centred on the pool; the world origin is the pool's
# bottom-left corner. world (x, y) = map (x, y) + POOL_MAP_TO_WORLD.
POOL_MAP_TO_WORLD = (25.0, 12.5)


def is_true(context, name):
    """Return True if the launch argument is 'true' (any letter case)."""
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def gazebo_actions(share_dir, start):
    """Start Gazebo with the pool, spawn the V4 at the start pose, and bridge it."""
    world_file = os.path.join(share_dir, 'worlds', 'pool.sdf')
    model_file = os.path.join(share_dir, 'models', 'rov_v4_planar.sdf')
    bridge_config = os.path.join(share_dir, 'config', 'gazebo_bridge.yaml')
    # .../install/rov_description/share: lets Gazebo resolve
    # model://rov_description/meshes/ROV_V4.STL.
    description_share_parent = os.path.dirname(
        get_package_share_directory('rov_description'))

    # Start pose in Gazebo world coordinates.
    spawn_x = start[0] + POOL_MAP_TO_WORLD[0]
    spawn_y = start[1] + POOL_MAP_TO_WORLD[1]

    return [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', description_share_parent),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            # -r: start running. Closing the Gazebo window ends the launch.
            launch_arguments={'gz_args': f'-r -v 3 {world_file}',
                              'on_exit_shutdown': 'true'}.items(),
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_rov',
            output='screen',
            arguments=[
                '-world', 'pool', '-name', 'rov_v4', '-file', model_file,
                '-x', str(spawn_x), '-y', str(spawn_y), '-z', '0.0',
                '-Y', str(start[2]),
            ],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gazebo_bridge',
            output='screen',
            parameters=[{'config_file': bridge_config}],
        ),
    ]


def launch_setup(context):
    planning_preview = is_true(context, 'planning_preview')
    planned_drive = is_true(context, 'planned_drive')
    if planning_preview and planned_drive:
        raise RuntimeError('Use only one of planning_preview:=true and planned_drive:=true')

    if planned_drive:
        mode = 'planned_drive'
    elif planning_preview:
        mode = 'planning_preview'
    else:
        mode = 'drive'
    use_planner = mode != 'drive'          # map + Nav2 planner + map RViz view
    use_controller = mode != 'planning_preview'  # the robot is allowed to move

    backend = LaunchConfiguration('backend').perform(context)
    if backend not in ('sim', 'gazebo'):
        raise RuntimeError(f"backend must be 'sim' or 'gazebo', got '{backend}'")
    use_gazebo = backend == 'gazebo'
    if use_gazebo and mode != 'planned_drive':
        raise RuntimeError('backend:=gazebo needs planned_drive:=true')
    # With Gazebo, every node follows Gazebo's clock (published on /clock).
    use_sim_time = {'use_sim_time': use_gazebo}

    # Installed files: install/rov_bringup/share/rov_bringup/...
    share_dir = get_package_share_directory('rov_bringup')
    nav2_params = os.path.join(share_dir, 'config', 'planning_preview.yaml')
    map_yaml = LaunchConfiguration('map_yaml').perform(context)
    if map_yaml and not use_planner:
        raise RuntimeError(
            'map_yaml needs planning_preview:=true or planned_drive:=true '
            '(the default drive mode uses no map)')
    if not map_yaml:
        map_yaml = 'pool_map.yaml' if use_gazebo else 'preview_map.yaml'
    if '/' not in map_yaml:
        map_yaml = os.path.join(share_dir, 'maps', map_yaml)
    if use_planner and not os.path.isfile(map_yaml):
        raise RuntimeError(f'Map file not found: {map_yaml}')
    map_name = os.path.splitext(os.path.basename(map_yaml))[0]
    if use_gazebo and map_name != 'pool_map':
        raise RuntimeError('backend:=gazebo only supports pool_map.yaml (the Gazebo pool)')

    # RViz view: sim_nav.rviz without a map. With a map, use the view named
    # after the map if there is one (pool_map.yaml -> rviz/pool_map.rviz),
    # otherwise planning_preview.rviz.
    rviz_config = os.path.join(share_dir, 'rviz', 'sim_nav.rviz')
    if use_planner:
        rviz_config = os.path.join(share_dir, 'rviz', map_name + '.rviz')
        if not os.path.isfile(rviz_config):
            rviz_config = os.path.join(share_dir, 'rviz', 'planning_preview.rviz')

    # Start pose on the map: the map's default, then any start_* overrides.
    start = list(DEFAULT_START.get(map_name, (0.0, 0.0, 0.0)))
    for i, name in enumerate(['start_x', 'start_y', 'start_yaw']):
        value = LaunchConfiguration(name).perform(context)
        if value and not use_planner:
            raise RuntimeError(
                f'{name} needs planning_preview:=true or planned_drive:=true '
                '(the default drive mode uses no map)')
        if value:
            start[i] = float(value)

    # Fixed map -> odom transform.
    #   sim: odom starts where the robot starts, so map -> odom is the start pose.
    #   gazebo: odom is the Gazebo world frame (pool corner), so map -> odom
    #   is the corner-vs-centre offset; the start pose is used for spawning.
    if use_gazebo:
        map_to_odom = (-POOL_MAP_TO_WORLD[0], -POOL_MAP_TO_WORLD[1], 0.0)
    else:
        map_to_odom = start

    # Visual-only ROV model from the installed rov_description package.
    urdf_file = os.path.join(
        get_package_share_directory('rov_description'), 'urdf', 'rov.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    actions = [
        # Publishes the model on "robot_description" for RViz's RobotModel
        # display. The model has only base_link, so this node adds no TF;
        # odom -> base_link comes from sim_node or Gazebo.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}, use_sim_time],
        ),
        # output='screen' prints the mission logs in this launch terminal.
        Node(
            package='rov_mission',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=[{'mode': mode}, use_sim_time],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[use_sim_time],
        ),
    ]

    if use_gazebo:
        actions += gazebo_actions(share_dir, start)
    else:
        actions.append(Node(
            package='rov_simulation',
            executable='sim_node',
            name='sim_node',
            output='screen',
        ))

    if use_controller:
        actions.append(Node(
            package='rov_control',
            executable='controller_node',
            name='controller_node',
            output='screen',
            parameters=[use_sim_time],
        ))

    if use_planner:
        actions += [
            # Ideal odometry never drifts, so map -> odom is fixed (see above).
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom',
                arguments=[
                    '--x', str(map_to_odom[0]), '--y', str(map_to_odom[1]),
                    '--yaw', str(map_to_odom[2]),
                    '--frame-id', 'map', '--child-frame-id', 'odom'],
                parameters=[use_sim_time],
            ),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[nav2_params, {'yaml_filename': map_yaml}, use_sim_time],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav2_params, use_sim_time],
            ),
            # Configures and activates map_server and planner_server (lifecycle nodes).
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_planning',
                output='screen',
                parameters=[nav2_params, use_sim_time],
            ),
        ]

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'planning_preview',
            default_value='false',
            description='true: show Nav2 global plans for clicked goals without moving the robot',
        ),
        DeclareLaunchArgument(
            'planned_drive',
            default_value='false',
            description='true: plan a route with Nav2 and drive along it',
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value='',
            description='Map for the map-based modes: a file name in rov_bringup/maps '
                        '(e.g. pool_map.yaml) or a full path. Empty: preview_map.yaml '
                        '(pool_map.yaml with backend:=gazebo)',
        ),
        DeclareLaunchArgument(
            'start_x', default_value='',
            description='Start x on the map in metres. Empty: the map default',
        ),
        DeclareLaunchArgument(
            'start_y', default_value='',
            description='Start y on the map in metres. Empty: the map default',
        ),
        DeclareLaunchArgument(
            'start_yaw', default_value='',
            description='Start heading on the map in radians. Empty: the map default',
        ),
        DeclareLaunchArgument(
            'backend', default_value='sim',
            description="'sim': sim_node moves the robot. 'gazebo': Gazebo Harmonic pool "
                        '(needs planned_drive:=true)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
