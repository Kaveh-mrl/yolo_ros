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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from dji_msgs.msg import Topics

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

    return LaunchDescription(
        [
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
                    "tracker": LaunchConfiguration("tracker", default="bytetrack.yaml"),
                    "device": LaunchConfiguration("device", default="cuda:0"),
                    "enable": LaunchConfiguration("enable", default="True"),
                    "threshold": LaunchConfiguration("threshold", default="0.5"),
                    "input_image_topic": LaunchConfiguration(
                        "input_image_topic",
                        default=[
                            "/",
                            robot_name,
                            "/",
                            Topics.GIMBAL_CAMERA_RAW_TOPIC,
                        ],
                    ),
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
            )
        ]
    )