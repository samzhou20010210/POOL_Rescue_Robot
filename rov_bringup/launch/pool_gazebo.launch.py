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
Show the V4 ROV in a Gazebo Harmonic pool (scene only, nothing moves).

Starts Gazebo with worlds/pool.sdf and spawns the V4 mesh from
rov_description as a fixed (static) model. No ROS topics are bridged and
the 2D simulation (sim_nav.launch.py) is not involved.

World frame: (0, 0) is the bottom-left corner of the 50 m x 25 m pool,
water surface at z = 0. The 2D map pool_map.yaml is centred instead, so
    world (x, y) = map (x + 25, y + 12.5).
The default spawn pose (1, 1) is the 2D pool start (-24, -11.5).

rov_x:=, rov_y:=, rov_z:=, rov_yaw:= (metres, radians) move the ROV.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The ROV as a static SDF model. "model://rov_description/..." is found
# through GZ_SIM_RESOURCE_PATH, which this launch file extends below.
ROV_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="rov_v4">
    <static>true</static>
    <link name="base_link">
      <visual name="hull">
        <geometry>
          <mesh>
            <uri>model://rov_description/meshes/ROV_V4.STL</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.6 0.6 0.6 1</ambient>
          <diffuse>0.6 0.6 0.6 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def generate_launch_description():
    world_file = os.path.join(
        get_package_share_directory('rov_bringup'), 'worlds', 'pool.sdf')
    # .../install/rov_description/share: the folder that contains rov_description/.
    description_share_parent = os.path.dirname(
        get_package_share_directory('rov_description'))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        # -r: start running (not paused). Closing Gazebo ends the launch.
        launch_arguments={'gz_args': f'-r -v 3 {world_file}',
                          'on_exit_shutdown': 'true'}.items(),
    )

    spawn_rov = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_rov',
        output='screen',
        arguments=[
            '-world', 'pool',
            '-name', 'rov_v4',
            '-string', ROV_SDF,
            '-x', LaunchConfiguration('rov_x'),
            '-y', LaunchConfiguration('rov_y'),
            '-z', LaunchConfiguration('rov_z'),
            '-Y', LaunchConfiguration('rov_yaw'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rov_x', default_value='1.0',
            description='ROV x in the pool world (m); 1.0 = 1 m from the west wall'),
        DeclareLaunchArgument(
            'rov_y', default_value='1.0',
            description='ROV y in the pool world (m); 1.0 = 1 m from the south wall'),
        DeclareLaunchArgument(
            'rov_z', default_value='0.0',
            description='ROV z (m); 0 = hull centre at the water surface, -5 = floor'),
        DeclareLaunchArgument(
            'rov_yaw', default_value='0.0',
            description='ROV heading (rad); 0 = bow along +x'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', description_share_parent),
        gazebo,
        spawn_rov,
    ])
