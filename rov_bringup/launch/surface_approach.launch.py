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
Surface-approach simulation (implementation steps 8B-10).

Starts, on top of a thrust-driven simulator (thrust_sim.launch.py, no
tracking extras):
  - Nav2 map_server (pool_map.yaml) + planner_server with A* and its
    lifecycle manager (config/surface_approach_nav2.yaml);
  - a static map -> odom transform (-25, -12.5): the map is centred on the
    pool, odom is the pool-corner world frame;
  - thrust_allocator_node and trajectory_controller_node (rov_control);
  - trajectory_generator_node (rov_planning): the ONLY trajectory source.
    (The step 5 Path -> trajectory converter is retired.)
  - The planner's costmap includes the swimmer layer (step 9). With
    swimmers:=true, fake_shore_node (rov_simulation) publishes the swimmer
    tracks of config/fake_shore.yaml (or swimmer_config:=<file>; a bare name
    is a file in rov_bringup/config/scenarios/).
  - With mission:=true (step 10): mission_manager_node and
    safety_monitor_node (rov_mission/config/mission.yaml),
    fake_vehicle_sensors_node (battery, leak) and fake_shore_node for the
    shore heartbeat (no swimmers unless swimmers:=true). Start a mission with
    a rov_interfaces/VictimTarget on /shore_alarm.
  - With rviz:=true: RViz (config/surface_approach.rviz) with the map, the
    A* path, the corridor, the dispatched trajectory and the terminal pose.

Send a victim target in the map frame, for example
  ros2 topic pub --once /victim_target geometry_msgs/msg/PointStamped
    "{header: {frame_id: map}, point: {x: 0.0, y: 0.0}}"
and read the outcome on /planner_report.

Arguments: backend (dynamics | gazebo), gui, start_x / start_y / start_yaw
(pool world frame), victim_state (submerged | surface; surface is always
dry-run for now), dry_run (true | false), swimmers (true | false),
swimmer_config (fake shore parameter file), mission (true | false), rviz
(true | false), water_current ('x,y' m/s), use_feedforward (true | false).
sim_nav.launch.py is unchanged.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Pool world vs. the pool-centred map: world (x, y) = map (x, y) + offset.
POOL_MAP_TO_WORLD = (25.0, 12.5)


def launch_setup(context):
    backend = LaunchConfiguration('backend').perform(context)
    use_sim_time = {'use_sim_time': backend == 'gazebo'}
    bringup = get_package_share_directory('rov_bringup')
    control_config = os.path.join(get_package_share_directory('rov_control'), 'config')
    planning_config = os.path.join(get_package_share_directory('rov_planning'), 'config')
    nav2_params = os.path.join(bringup, 'config', 'surface_approach_nav2.yaml')
    map_yaml = os.path.join(bringup, 'maps', 'pool_map.yaml')

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'thrust_sim.launch.py')),
        launch_arguments={
            'backend': backend,
            'gui': LaunchConfiguration('gui').perform(context),
            'start_x': LaunchConfiguration('start_x').perform(context),
            'start_y': LaunchConfiguration('start_y').perform(context),
            'start_yaw': LaunchConfiguration('start_yaw').perform(context),
            'water_current': LaunchConfiguration('water_current').perform(context),
            'tracking': 'false',
        }.items(),
    )
    actions = []
    use_feedforward = LaunchConfiguration('use_feedforward').perform(context).lower() == 'true'
    swimmers = LaunchConfiguration('swimmers').perform(context).lower() == 'true'
    mission = LaunchConfiguration('mission').perform(context).lower() == 'true'
    if mission and not swimmers:
        # Shore heartbeat only: the default fake shore, inactive (no swimmers).
        actions.append(Node(
            package='rov_simulation',
            executable='fake_shore_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('rov_simulation'), 'config',
                                     'fake_shore.yaml'), {'active': False}, use_sim_time],
        ))
    if mission:
        mission_config = os.path.join(get_package_share_directory('rov_mission'), 'config',
                                      'mission.yaml')
        actions += [
            Node(package='rov_mission', executable='mission_manager_node', output='screen',
                 parameters=[mission_config, use_sim_time]),
            Node(package='rov_mission', executable='safety_monitor_node', output='screen',
                 parameters=[mission_config, use_sim_time]),
            Node(package='rov_simulation', executable='fake_vehicle_sensors_node',
                 output='screen', parameters=[use_sim_time]),
        ]
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(
            package='rviz2', executable='rviz2', output='log',
            arguments=['-d', os.path.join(bringup, 'rviz', 'surface_approach.rviz')],
            parameters=[use_sim_time]))
    if swimmers:
        swimmer_config = LaunchConfiguration('swimmer_config').perform(context) or os.path.join(
            get_package_share_directory('rov_simulation'), 'config', 'fake_shore.yaml')
        if not os.path.isabs(swimmer_config):
            # A bare name: a file in rov_bringup/config/scenarios/.
            swimmer_config = os.path.join(
                get_package_share_directory('rov_bringup'), 'config', 'scenarios', swimmer_config)
        actions.append(Node(
            package='rov_simulation',
            executable='fake_shore_node',
            output='screen',
            parameters=[swimmer_config, use_sim_time],
        ))
    return actions + [
        simulator,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['--x', str(-POOL_MAP_TO_WORLD[0]), '--y', str(-POOL_MAP_TO_WORLD[1]),
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
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_surface_planning',
            output='screen',
            parameters=[nav2_params, use_sim_time],
        ),
        Node(
            package='rov_control',
            executable='thrust_allocator_node',
            output='screen',
            parameters=[os.path.join(control_config, 'thrust_allocator.yaml'), use_sim_time],
        ),
        Node(
            package='rov_control',
            executable='trajectory_controller_node',
            output='screen',
            parameters=[os.path.join(control_config, 'trajectory_controller.yaml'),
                        {'use_feedforward': use_feedforward},
                        use_sim_time],
        ),
        Node(
            package='rov_planning',
            executable='trajectory_generator_node',
            output='screen',
            parameters=[
                os.path.join(planning_config, 'trajectory_generator.yaml'),
                {'victim_state': LaunchConfiguration('victim_state').perform(context),
                 'dry_run': LaunchConfiguration('dry_run').perform(context).lower() == 'true'},
                use_sim_time,
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='dynamics',
                              description='dynamics (lightweight model) or gazebo'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='gazebo only: false runs the server only'),
        DeclareLaunchArgument('start_x', default_value='5.0',
                              description='start x in the pool world frame (m)'),
        DeclareLaunchArgument('start_y', default_value='5.0',
                              description='start y in the pool world frame (m)'),
        DeclareLaunchArgument('start_yaw', default_value='0.0',
                              description='start heading (rad)'),
        DeclareLaunchArgument('victim_state', default_value='submerged',
                              description='submerged or surface (surface: dry-run only)'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='true: plan and report, do not send'),
        DeclareLaunchArgument('swimmers', default_value='false',
                              description='true: start fake_shore_node (simulated swimmers)'),
        DeclareLaunchArgument('swimmer_config', default_value='',
                              description='fake_shore_node parameter file (default: installed)'),
        DeclareLaunchArgument('water_current', default_value='0.0,0.0',
                              description="constant water current 'x,y', world frame (m/s)"),
        DeclareLaunchArgument('use_feedforward', default_value='true',
                              description='controller model feedforward (step 5 comparison)'),
        DeclareLaunchArgument('mission', default_value='false',
                              description='true: mission manager, safety monitor, sensors'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='true: RViz with the planning displays'),
        OpaqueFunction(function=launch_setup),
    ])
