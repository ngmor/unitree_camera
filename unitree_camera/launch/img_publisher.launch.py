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
# from .ternary_text_substitution import TernaryTextSubstitution

from typing import Text

from launch.condition import Condition
from launch.substitution import Substitution
from launch.launch_context import LaunchContext

class TernaryTextSubstitution(Substitution):
    """Substitution that returns the first text argument if the condition is true, otherwise it returns the second text argument."""
    def __init__(self, condition: Condition, true_text: Text, false_text: Text) -> None:
        """Create a TernaryTextSubstitution."""
        super().__init__()

        if not isinstance(true_text, Text):
            raise TypeError(
                "TernaryTextSubstitution expected Text object got '{}' instead.".format(type(true_text))
            )
        if not isinstance(false_text, Text):
            raise TypeError(
                "TernaryTextSubstitution expected Text object got '{}' instead.".format(type(false_text))
            )
        
        if not isinstance(condition, Condition):
            raise TypeError(
                "TernaryTextSubstitution expected Condition object got '{}' instead.".format(type(condition))
            )

        self.__true_text = true_text
        self.__false_text = false_text
        self.__condition = condition

    @property
    def true_text(self) -> Text:
        """Getter for true text."""
        return self.__true_text
    
    @property
    def false_text(self) -> Text:
        """Getter for false text."""
        return self.__false_text
    
    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return f"'{self.true_text}' if true, '{self.false_text}' if false"

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by returning the string itself."""
        if self.__condition.evaluate(context):
            return self.true_text
        else:
            return self.false_text

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
            name='dummy',
            value='true'
        ),
        SetLaunchConfiguration(
            name='yaml_file',
            value=TernaryTextSubstitution(LaunchConfigurationEquals('dummy', 'true'),"true val", "false_val")
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