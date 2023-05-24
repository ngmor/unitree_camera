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
    """Launch selected bottom camera nodes."""
    return LaunchDescription([

        ########## BOTTOM CAMERA ##########
        DeclareLaunchArgument(
            name='body.bottom.enable_cam',
            default_value='true',
            choices=['true','false'],
            description='Enable the body bottom camera.',
        ),
        DeclareLaunchArgument(
            name='body.bottom.fps',
            default_value='30',
            description='The frame rate of the body bottom camera (fps). Should not exceed the FPS set in the YAML file.',
        ),
        DeclareLaunchArgument(
            name='body.bottom.enable_raw',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing of raw frames from the body bottom camera.',
        ),
        DeclareLaunchArgument(
            name='body.bottom.enable_rect',
            default_value='true',
            choices=['true','false'],
            description='Enable publishing of rectified frames from the body bottom camera.',
        ),
        DeclareLaunchArgument(
            name='body.bottom.enable_depth',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing of depth frames from the body bottom camera.',
        ),
        DeclareLaunchArgument(
            name='body.bottom.enable_point_cloud',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing point cloud data from the body bottom camera.',
        ),
        DeclareLaunchArgument(
            name='body.bottom.point_cloud_frame',
            default_value='camera_rearDown',
            description='Frame ID for point cloud messages from the body bottom camera.',
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
                ('camera', 'body_bottom'),
                ('fps', LaunchConfiguration('body.bottom.fps')),
                ('enable_raw', LaunchConfiguration('body.bottom.enable_raw')),
                ('enable_rect', LaunchConfiguration('body.bottom.enable_rect')),
                ('enable_depth', LaunchConfiguration('body.bottom.enable_depth')),
                ('enable_point_cloud', LaunchConfiguration('body.bottom.enable_point_cloud')),
                ('point_cloud_frame', LaunchConfiguration('body.bottom.point_cloud_frame')),
            ],
            condition=IfCondition(LaunchConfiguration('body.bottom.enable_cam')),
        ),
    ])