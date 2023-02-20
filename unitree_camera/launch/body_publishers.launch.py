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
    """Launch selected body camera nodes."""
    return LaunchDescription([

        ########## LEFT CAMERA ##########
        DeclareLaunchArgument(
            name='left.enable_cam',
            default_value='true',
            choices=['true','false'],
            description='Enable the left camera.',
        ),
        DeclareLaunchArgument(
            name='left.enable_raw',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing of raw frames from the left camera.',
        ),
        DeclareLaunchArgument(
            name='left.enable_rect',
            default_value='true',
            choices=['true','false'],
            description='Enable publishing of rectified frames from the left camera.',
        ),
        DeclareLaunchArgument(
            name='left.enable_depth',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing of depth frames from the left camera.',
        ),
        DeclareLaunchArgument(
            name='left.enable_point_cloud',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing point cloud data from the left camera.',
        ),
        DeclareLaunchArgument(
            name='left.point_cloud_frame',
            default_value='camera_left',
            description='Frame ID for point cloud messages from the left camera.',
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
                ('camera', 'body_left'),
                ('enable_raw', LaunchConfiguration('left.enable_raw')),
                ('enable_rect', LaunchConfiguration('left.enable_rect')),
                ('enable_depth', LaunchConfiguration('left.enable_depth')),
                ('enable_point_cloud', LaunchConfiguration('left.enable_point_cloud')),
                ('point_cloud_frame', LaunchConfiguration('left.point_cloud_frame')),
            ],
            condition=IfCondition(LaunchConfiguration('left.enable_cam')),
        ),

        ########## RIGHT CAMERA ##########
        DeclareLaunchArgument(
            name='right.enable_cam',
            default_value='true',
            choices=['true','false'],
            description='Enable the right camera.',
        ),
        DeclareLaunchArgument(
            name='right.enable_raw',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing of raw frames from the right camera.',
        ),
        DeclareLaunchArgument(
            name='right.enable_rect',
            default_value='true',
            choices=['true','false'],
            description='Enable publishing of rectified frames from the right camera.',
        ),
        DeclareLaunchArgument(
            name='right.enable_depth',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing of depth frames from the right camera.',
        ),
        DeclareLaunchArgument(
            name='right.enable_point_cloud',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing point cloud data from the right camera.',
        ),
        DeclareLaunchArgument(
            name='right.point_cloud_frame',
            default_value='camera_right',
            description='Frame ID for point cloud messages from the right camera.',
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
                ('camera', 'body_right'),
                ('enable_raw', LaunchConfiguration('right.enable_raw')),
                ('enable_rect', LaunchConfiguration('right.enable_rect')),
                ('enable_depth', LaunchConfiguration('right.enable_depth')),
                ('enable_point_cloud', LaunchConfiguration('right.enable_point_cloud')),
                ('point_cloud_frame', LaunchConfiguration('right.point_cloud_frame')),
            ],
            condition=IfCondition(LaunchConfiguration('right.enable_cam')),
        ),
    ])