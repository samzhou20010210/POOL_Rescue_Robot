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
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def is_true(context, name):
    """Return True if the launch argument is 'true' (any letter case)."""
    return LaunchConfiguration(name).perform(context).lower() == 'true'


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

    # Installed files: install/rov_bringup/share/rov_bringup/...
    share_dir = get_package_share_directory('rov_bringup')
    nav2_params = os.path.join(share_dir, 'config', 'planning_preview.yaml')
    preview_map = os.path.join(share_dir, 'maps', 'preview_map.yaml')
    rviz_config = os.path.join(
        share_dir, 'rviz', 'planning_preview.rviz' if use_planner else 'sim_nav.rviz')

    # Visual-only ROV model from the installed rov_description package.
    urdf_file = os.path.join(
        get_package_share_directory('rov_description'), 'urdf', 'rov.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    actions = [
        Node(
            package='rov_simulation',
            executable='sim_node',
            name='sim_node',
            output='screen',
        ),
        # Publishes the model on "robot_description" for RViz's RobotModel
        # display. The model has only base_link, so this node adds no TF;
        # sim_node still publishes odom -> base_link.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        # output='screen' prints the mission logs in this launch terminal.
        Node(
            package='rov_mission',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=[{'mode': mode}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
        ),
    ]

    if use_controller:
        actions.append(Node(
            package='rov_control',
            executable='controller_node',
            name='controller_node',
            output='screen',
        ))

    if use_planner:
        actions += [
            # Ideal simulation: odometry never drifts, so map and odom are the same frame.
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom',
                arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            ),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[nav2_params, {'yaml_filename': preview_map}],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav2_params],
            ),
            # Configures and activates map_server and planner_server (lifecycle nodes).
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_planning',
                output='screen',
                parameters=[nav2_params],
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
        OpaqueFunction(function=launch_setup),
    ])
