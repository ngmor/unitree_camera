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
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from unitree_camera_launch_module import OrCondition, TernaryTextSubstitution

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
            name='dummy1',
            value='false'
        ),
        SetLaunchConfiguration(
            name='dummy2',
            value='false'
        ),
        SetLaunchConfiguration(
            name='yaml_file',
            value=TernaryTextSubstitution(
                OrCondition([
                    LaunchConfigurationEquals('dummy1', 'false'),
                    LaunchConfigurationEquals('dummy1', 'false'),
                ]),
                "true val",
                "false_val"
            )
        ),

    # param.description = "Path to yaml configuration file.";
    # declare_parameter("yaml_path", "", param);
    # auto yaml_path = get_parameter("yaml_path").get_parameter_value().get<std::string>();

    # param.description = "Enable publishing of raw frames.";
    # declare_parameter("enable_raw", false, param);
    # enable_raw_ = get_parameter("enable_raw").get_parameter_value().get<bool>();

    # param.description = "Enable publishing of rectified frames.";
    # declare_parameter("enable_rect", true, param);
    # enable_rect_ = get_parameter("enable_rect").get_parameter_value().get<bool>();

    # param.description = "Enable publishing of depth frames.";
    # declare_parameter("enable_depth", false, param);
    # enable_depth_ = get_parameter("enable_depth").get_parameter_value().get<bool>();

    # param.description = "Enable publishing point cloud data.";
    # declare_parameter("enable_point_cloud", false, param);
    # enable_point_cloud_ = get_parameter("enable_point_cloud").get_parameter_value().get<bool>();

    # param.description = "Frame ID for point cloud messages.";
    # declare_parameter("point_cloud_frame", "map", param);
    # point_cloud_frame_ = get_parameter("point_cloud_frame").get_parameter_value().get<std::string>();


        Node(
            package='unitree_camera',
            executable='img_publisher',
            name=LaunchConfiguration('camera'),
            output='screen',
            parameters=[{
                'use_yaml':True,
                'yaml_path':
                    ParameterValue(
                        PathJoinSubstitution([
                            FindPackageShare('unitree_camera'),
                            'config',
                            LaunchConfiguration('yaml_file'),
                        ]),
                        value_type=str
                    ),
            }]
        ),
    ])