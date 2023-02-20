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
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, TextSubstitution, \
    PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    """Launch selected head camera nodes."""
    return LaunchDescription([

        ########## FRONT CAMERA ##########
        DeclareLaunchArgument(
            name='front.enable_cam',
            default_value='true',
            choices=['true','false'],
            description='Enable the front camera.',
        ),
        DeclareLaunchArgument(
            name='front.enable_raw',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing of raw frames from the front camera.',
        ),
        DeclareLaunchArgument(
            name='front.enable_rect',
            default_value='true',
            choices=['true','false'],
            description='Enable publishing of rectified frames from the front camera.',
        ),
        DeclareLaunchArgument(
            name='front.enable_depth',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing of depth frames from the front camera.',
        ),
        DeclareLaunchArgument(
            name='front.enable_point_cloud',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing point cloud data from the front camera.',
        ),
        DeclareLaunchArgument(
            name='front.point_cloud_frame',
            default_value='camera_face',
            description='Frame ID for point cloud messages from the front camera.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('unitree_camera'),
                    'launch',
                    'img_publisher.launch.py',
                ]),
            ]),
            launch_arguments=[
                ('camera', 'head_front'),
                ('enable_raw', LaunchConfiguration('front.enable_raw')),
                ('enable_rect', LaunchConfiguration('front.enable_rect')),
                ('enable_depth', LaunchConfiguration('front.enable_depth')),
                ('enable_point_cloud', LaunchConfiguration('front.enable_point_cloud')),
                ('point_cloud_frame', LaunchConfiguration('front.point_cloud_frame')),
            ],
            condition=IfCondition(LaunchConfiguration('front.enable_cam')),
        ),

        ########## BOTTOM CAMERA ##########
        DeclareLaunchArgument(
            name='bottom.enable_cam',
            default_value='true',
            choices=['true','false'],
            description='Enable the bottom camera.',
        ),
        DeclareLaunchArgument(
            name='bottom.enable_raw',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing of raw frames from the bottom camera.',
        ),
        DeclareLaunchArgument(
            name='bottom.enable_rect',
            default_value='true',
            choices=['true','false'],
            description='Enable publishing of rectified frames from the bottom camera.',
        ),
        DeclareLaunchArgument(
            name='bottom.enable_depth',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing of depth frames from the bottom camera.',
        ),
        DeclareLaunchArgument(
            name='bottom.enable_point_cloud',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing point cloud data from the bottom camera.',
        ),
        DeclareLaunchArgument(
            name='bottom.point_cloud_frame',
            default_value='camera_chin',
            description='Frame ID for point cloud messages from the bottom camera.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('unitree_camera'),
                    'launch',
                    'img_publisher.launch.py',
                ]),
            ]),
            launch_arguments=[
                ('camera', 'head_bottom'),
                ('enable_raw', LaunchConfiguration('bottom.enable_raw')),
                ('enable_rect', LaunchConfiguration('bottom.enable_rect')),
                ('enable_depth', LaunchConfiguration('bottom.enable_depth')),
                ('enable_point_cloud', LaunchConfiguration('bottom.enable_point_cloud')),
                ('point_cloud_frame', LaunchConfiguration('bottom.point_cloud_frame')),
            ],
            condition=IfCondition(LaunchConfiguration('bottom.enable_cam')),
        ),
    ])