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
        DeclareLaunchArgument(
            name='camera',
            default_value='head_front',
            choices=['head_front', 'head_bottom', 'body_left', 'body_right', 'body_bottom'],
            description='Which camera to publish images from.',
        ),
        DeclareLaunchArgument(
            name='fps',
            default_value='30',
            description='The frame rate of the camera (fps). Should not exceed the FPS set in the YAML file.',
        ),
        DeclareLaunchArgument(
            name='enable_raw',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing of raw frames.',
        ),
        DeclareLaunchArgument(
            name='enable_rect',
            default_value='true',
            choices=['true','false'],
            description='Enable publishing of rectified frames.',
        ),
        DeclareLaunchArgument(
            name='enable_depth',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing of depth frames.',
        ),
        DeclareLaunchArgument(
            name='enable_point_cloud',
            default_value='false',
            choices=['true','false'],
            description='Enable publishing point cloud data.',
        ),
        DeclareLaunchArgument(
            name='point_cloud_frame',
            default_value='map',
            description='Frame ID for point cloud messages.',
        ),
        SetLaunchConfiguration(
            name='namespace',
            value=ReplaceTextSubstitution(LaunchConfiguration('camera'), '_', '/'), # replace underscores with slashes
        ),
        # select a launch file based on which camera we're using
        SetLaunchConfiguration(
            name='yaml_file',
            value=TernaryTextSubstitution(
                OrCondition([
                    LaunchConfigurationEquals('camera', 'head_bottom'),
                    LaunchConfigurationEquals('camera', 'body_right'),
                    LaunchConfigurationEquals('camera', 'body_bottom'),
                ]),
                "stereo_camera_config0.yaml", # head_bottom camera, body_right, and body_bototm camera
                "stereo_camera_config1.yaml", # head_front camera and body_left camera
            )
        ),
        Node(
            package='unitree_camera',
            executable='img_publisher',
            namespace=LaunchConfiguration('namespace'),
            name='cam',
            output='screen',
            parameters=[{
                'use_yaml': True,
                'yaml_path':
                    ParameterValue(
                        PathJoinSubstitution([
                            FindPackageShare('unitree_camera'),
                            'config',
                            LaunchConfiguration('yaml_file'),
                        ]),
                        value_type=str
                    ),
                'fps': LaunchConfiguration('fps'),
                'enable_raw': LaunchConfiguration('enable_raw'),
                'enable_rect': LaunchConfiguration('enable_rect'),
                'enable_depth': LaunchConfiguration('enable_depth'),
                'enable_point_cloud': LaunchConfiguration('enable_point_cloud'),
                'point_cloud_frame': LaunchConfiguration('point_cloud_frame'),
            }]
        ),
    ])