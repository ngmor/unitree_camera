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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    """Launch the image subscriber nodes for the bottom cameras."""
    return LaunchDescription([

        ########## BOTTOM CAMERA ##########
        DeclareLaunchArgument(
            name='body.bottom.enable_cam',
            default_value='true',
            choices=['true','false'],
            description='Enable the bottom camera subscriber.',
        ),
        Node(
            package='unitree_camera',
            executable='img_subscriber',
            namespace='body/bottom/cam',
            name='sub',
            output='screen',
            condition=IfCondition(LaunchConfiguration('body.bottom.enable_cam')),
        ),
    ])