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
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, Shutdown
from launch.substitutions import Command, TextSubstitution, \
    PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    """Launch the image publisher node for a certain specified camera."""
    return LaunchDescription([
        DeclareLaunchArgument(
            name='camera',
            default_value='head_front',
            choices=['head_front', 'head_bottom', 'body_left', 'body_right'],
            description='Which camera to publish images from.',
        ),
        SetLaunchConfiguration(
            name='node_name',
            value=[
                LaunchConfiguration('camera'),
                '_cam'
            ],
        ),
        Node(
            package='unitree_camera',
            executable='img_publisher',
            name=LaunchConfiguration('node_name'),
            output='screen',
        ),
    ])