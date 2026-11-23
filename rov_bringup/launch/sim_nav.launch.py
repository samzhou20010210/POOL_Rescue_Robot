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
Start the simulation demo.

Default (planning_preview:=false): click-to-drive.
    sim_node, controller_node, mission_node, RViz2 (Fixed Frame: odom).

Preview (planning_preview:=true): Nav2 global planning only, robot stays still.
    sim_node, mission_node, RViz2 (Fixed Frame: map), a static map -> odom
    transform, and Nav2's map_server, planner_server, and lifecycle_manager.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Installed files: install/rov_bringup/share/rov_bringup/...
    share_dir = get_package_share_directory('rov_bringup')
    drive_rviz_config = os.path.join(share_dir, 'rviz', 'sim_nav.rviz')
    preview_rviz_config = os.path.join(share_dir, 'rviz', 'planning_preview.rviz')
    preview_params = os.path.join(share_dir, 'config', 'planning_preview.yaml')
    preview_map = os.path.join(share_dir, 'maps', 'preview_map.yaml')

    planning_preview = LaunchConfiguration('planning_preview')
    declare_planning_preview = DeclareLaunchArgument(
        'planning_preview',
        default_value='false',
        description='true: show Nav2 global plans for clicked goals without moving the robot',
    )
    in_preview = IfCondition(planning_preview)
    in_drive = UnlessCondition(planning_preview)

    # ---- Started in both modes ----

    sim_node = Node(
        package='rov_simulation',
        executable='sim_node',
        name='sim_node',
        output='screen',
    )

    # output='screen' prints the mission logs in this launch terminal.
    mission_node = Node(
        package='rov_mission',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[{'planning_preview': ParameterValue(planning_preview, value_type=bool)}],
    )

    # ---- Default mode: click-to-drive ----

    controller_node = Node(
        package='rov_control',
        executable='controller_node',
        name='controller_node',
        output='screen',
        condition=in_drive,
    )

    drive_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', drive_rviz_config],
        condition=in_drive,
    )

    # ---- Preview mode: Nav2 global planning ----

    # Ideal simulation: odometry never drifts, so map and odom are the same frame.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        condition=in_preview,
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[preview_params, {'yaml_filename': preview_map}],
        condition=in_preview,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[preview_params],
        condition=in_preview,
    )

    # Configures and activates map_server and planner_server (lifecycle nodes).
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planning',
        output='screen',
        parameters=[preview_params],
        condition=in_preview,
    )

    preview_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', preview_rviz_config],
        condition=in_preview,
    )

    return LaunchDescription([
        declare_planning_preview,
        sim_node,
        mission_node,
        controller_node,
        drive_rviz,
        map_to_odom,
        map_server,
        planner_server,
        lifecycle_manager,
        preview_rviz,
    ])
