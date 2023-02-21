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
    """Launch selected head camera nodes."""
    return LaunchDescription([

        ########## FRONT CAMERA ##########
        DeclareLaunchArgument(
            name='head.front.enable_cam',
            default_value='true',
            choices=['true','false'],
            description='Enable the head front camera.',
        ),
        DeclareLaunchArgument(
            name='head.front.fps',
            default_value='30',
            description='The frame rate of the head front camera (fps). Should not exceed the FPS set in the YAML file.',
        ),
        DeclareLaunchArgument(
            name='head.front.enable_raw',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing of raw frames from the head front camera.',
        ),
        DeclareLaunchArgument(
            name='head.front.enable_rect',
            default_value='true',
            choices=['true','false'],
            description='Enable publishing of rectified frames from the head front camera.',
        ),
        DeclareLaunchArgument(
            name='head.front.enable_depth',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing of depth frames from the head front camera.',
        ),
        DeclareLaunchArgument(
            name='head.front.enable_point_cloud',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing point cloud data from the head front camera.',
        ),
        DeclareLaunchArgument(
            name='head.front.point_cloud_frame',
            default_value='camera_face',
            description='Frame ID for point cloud messages from the head front camera.',
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
                ('fps', LaunchConfiguration('head.front.fps')),
                ('enable_raw', LaunchConfiguration('head.front.enable_raw')),
                ('enable_rect', LaunchConfiguration('head.front.enable_rect')),
                ('enable_depth', LaunchConfiguration('head.front.enable_depth')),
                ('enable_point_cloud', LaunchConfiguration('head.front.enable_point_cloud')),
                ('point_cloud_frame', LaunchConfiguration('head.front.point_cloud_frame')),
            ],
            condition=IfCondition(LaunchConfiguration('head.front.enable_cam')),
        ),

        ########## BOTTOM CAMERA ##########
        DeclareLaunchArgument(
            name='head.bottom.enable_cam',
            default_value='true',
            choices=['true','false'],
            description='Enable the bottom camera.',
        ),
        DeclareLaunchArgument(
            name='head.bottom.fps',
            default_value='30',
            description='The frame rate of the head bottom camera (fps). Should not exceed the FPS set in the YAML file.',
        ),
        DeclareLaunchArgument(
            name='head.bottom.enable_raw',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing of raw frames from the head bottom camera.',
        ),
        DeclareLaunchArgument(
            name='head.bottom.enable_rect',
            default_value='true',
            choices=['true','false'],
            description='Enable publishing of rectified frames from the head bottom camera.',
        ),
        DeclareLaunchArgument(
            name='head.bottom.enable_depth',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing of depth frames from the head bottom camera.',
        ),
        DeclareLaunchArgument(
            name='head.bottom.enable_point_cloud',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing point cloud data from the head bottom camera.',
        ),
        DeclareLaunchArgument(
            name='head.bottom.point_cloud_frame',
            default_value='camera_chin',
            description='Frame ID for point cloud messages from the head bottom camera.',
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
                ('fps', LaunchConfiguration('head.bottom.fps')),
                ('enable_raw', LaunchConfiguration('head.bottom.enable_raw')),
                ('enable_rect', LaunchConfiguration('head.bottom.enable_rect')),
                ('enable_depth', LaunchConfiguration('head.bottom.enable_depth')),
                ('enable_point_cloud', LaunchConfiguration('head.bottom.enable_point_cloud')),
                ('point_cloud_frame', LaunchConfiguration('head.bottom.point_cloud_frame')),
            ],
            condition=IfCondition(LaunchConfiguration('head.bottom.enable_cam')),
        ),
    ])