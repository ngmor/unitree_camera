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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Launch all camera subscriber nodes."""
    return LaunchDescription([
        ########## HEAD ##########
        DeclareLaunchArgument(
            name='head.front.enable_cam',
            default_value='true',
            choices=['true','false'],
            description='Enable the front camera subscriber.',
        ),
        DeclareLaunchArgument(
            name='head.bottom.enable_cam',
            default_value='true',
            choices=['true','false'],
            description='Enable the bottom camera subscriber.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('unitree_camera'),
                    'launch',
                    'head_subscribers.launch.py',
                ]),
            ]),
            launch_arguments=[
                ('head.front.enable_cam', LaunchConfiguration('head.front.enable_cam')),
                ('head.bottom.enable_cam', LaunchConfiguration('head.bottom.enable_cam')),
            ],
        ),

        ########## BODY ##########
        DeclareLaunchArgument(
            name='body.left.enable_cam',
            default_value='true',
            choices=['true','false'],
            description='Enable the left camera subscriber.',
        ),
        DeclareLaunchArgument(
            name='body.right.enable_cam',
            default_value='true',
            choices=['true','false'],
            description='Enable the right camera subscriber.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('unitree_camera'),
                    'launch',
                    'body_subscribers.launch.py',
                ]),
            ]),
            launch_arguments=[
                ('body.left.enable_cam', LaunchConfiguration('body.left.enable_cam')),
                ('body.right.enable_cam', LaunchConfiguration('body.right.enable_cam')),
            ],
        ),
    ])