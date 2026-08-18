# Copyright (C) 2024 Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue

from dji_msgs.msg import Topics
from launch_ros.actions import Node

def generate_launch_description():

    robot_name = LaunchConfiguration("robot_name", default="M350")

    model_path = PathJoinSubstitution(
        [
            FindPackageShare(
                LaunchConfiguration(
                    "model_package",
                    default="alars_labeling_training",
                )
            ),
            LaunchConfiguration(
                "model_subdir",
                default="trained_models",
            ),
            LaunchConfiguration(
                "model_file",
                default="yolo_model_2cls_mixed.pt",
            ),
        ]
    )

    threshold = LaunchConfiguration("threshold", default="0.5")

    input_image_topic = LaunchConfiguration(
        "input_image_topic",
        default=[
            "/",
            robot_name,
            "/",
            Topics.GIMBAL_CAMERA_RAW_TOPIC,
        ],
    )
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    max_fps = LaunchConfiguration("max_fps")

    return LaunchDescription(
        [
            LogInfo(msg=["[yolocustom] model_path = ", model_path]),
            LogInfo(msg=["[yolocustom] threshold = ", threshold]),
            LogInfo(msg=["[yolocustom] input_image_topic = ", input_image_topic]),
            DeclareLaunchArgument(
                "width",
                default_value="640",
            ),

            DeclareLaunchArgument(
                "height",
                default_value="360",
            ),

            DeclareLaunchArgument(
                "max_fps",
                default_value="10.0",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("yolo_bringup"),
                        "launch",
                        "yolo.launch.py",
                    )
                ),
                launch_arguments={
                    "model": model_path,
                    "device": LaunchConfiguration("device", default="cuda:0"),
                    "tracker": LaunchConfiguration("tracker", default="bytetrack.yaml"),
                    "use_tracking": LaunchConfiguration("use_tracking", default="False"),
                    "enable": LaunchConfiguration("enable", default="True"),
                    "threshold": threshold,
                    "input_image_topic": input_image_topic,
                    "image_reliability": LaunchConfiguration(
                        "image_reliability", default="1"
                    ),
                    "namespace": LaunchConfiguration(
                        "namespace",
                        default=[
                            robot_name,
                            "/yolo",
                        ],
                    ),
                }.items(),
            ),
            Node(
                package="yolo_smarc_actions",
                executable="yolo_downsampler.py",
                name="yolo_downsampler",
                output="screen",
                parameters=[
                    {
                        "input_topic": [
                            "/",
                            robot_name,
                            "/yolo/dbg_image",
                        ],
                        "output_topic": [
                            "/",
                            robot_name,
                            "/yolo/dbg_image_down",
                        ],
                        "width": ParameterValue(width, value_type=int),
                        "height": ParameterValue(height, value_type=int),
                        "max_fps": ParameterValue(max_fps, value_type=float),
                        "publish_compressed": False,
                        "jpeg_quality": 70,
                    }
                ],
            ),
        ]
    )