# Copyright 2023 Nick Morales.
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

# TODO - document

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from unitree_camera_launch_module import OrCondition, TernaryTextSubstitution, \
    ReplaceTextSubstitution

def generate_launch_description():
    """Launch the image publisher node for a certain specified camera."""
    return LaunchDescription([
        Node(
            package='unitree_camera',
            executable='info_publisher',
            name='info_head_front',
            output='screen',
            parameters=[PathJoinSubstitution([
                            FindPackageShare('unitree_camera'),
                            'config',
                            'head_front.yaml',
                        ]),
                        {'stereo_name': 'head/front',}]
        ),
        Node(
            package='unitree_camera',
            executable='info_publisher',
            name='info_body_left',
            output='screen',
            parameters=[PathJoinSubstitution([
                            FindPackageShare('unitree_camera'),
                            'config',
                            'body_left.yaml',
                        ]),
                        {'stereo_name': 'body/left',}]
        ),
        Node(
            package='unitree_camera',
            executable='info_publisher',
            name='info_body_right',
            output='screen',
            parameters=[PathJoinSubstitution([
                            FindPackageShare('unitree_camera'),
                            'config',
                            'body_right.yaml',
                        ]),
                        {'stereo_name': 'body/right',}]
        ),
    ])