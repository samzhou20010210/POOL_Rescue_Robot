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

"""Start the simulation demo: sim_node, mission_node, and RViz2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Find the installed RViz config: install/rov_bringup/share/rov_bringup/rviz/
    rviz_config = os.path.join(
        get_package_share_directory('rov_bringup'), 'rviz', 'sim_nav.rviz')

    sim_node = Node(
        package='rov_simulation',
        executable='sim_node',
        name='sim_node',
        output='screen',
    )

    # output='screen' prints "Received goal: ..." in this launch terminal.
    mission_node = Node(
        package='rov_mission',
        executable='mission_node',
        name='mission_node',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([sim_node, mission_node, rviz])
