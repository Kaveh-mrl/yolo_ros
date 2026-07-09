import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

from dji_msgs.msg import Topics


def generate_launch_description():

    robot_name = LaunchConfiguration("robot_name", default="M350")

    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time", default="false"),
        value_type=bool,
    )

    yolo_namespace = [
        robot_name,
        "/yolo",
    ]

    input_image_topic = [
        "/",
        robot_name,
        "/",
        Topics.GIMBAL_CAMERA_RAW_TOPIC,
    ]

    input_camera_info_topic = [
        "/",
        robot_name,
        "/",
        Topics.GIMBAL_CAMERA_INFO_TOPIC,
    ]

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("yolo_bringup"),
                        "launch",
                        "yolocustom.launch.py",
                    )
                ),
                launch_arguments={
                    "model_package": LaunchConfiguration(
                        "model_package",
                        default="alars_labeling_training",
                    ),
                    "model_subdir": LaunchConfiguration(
                        "model_subdir",
                        default="trained_models",
                    ),
                    "model_file": LaunchConfiguration(
                        "model_file",
                        default="yolo_model_2cls_mixed.pt",
                    ),
                    "namespace": yolo_namespace,
                    "device": LaunchConfiguration("device", default="cuda:0"),
                    "threshold": LaunchConfiguration("threshold", default="0.5"),
                    "enable": LaunchConfiguration("enable", default="True"),
                    "use_tracking": LaunchConfiguration("use_tracking", default="False"),
                    "input_image_topic": input_image_topic,
                }.items(),
            ),

            Node(
                package="yolo_smarc_actions",
                executable="yolo_corners_adapter.py",
                name="yolo_corners_adapter",
                namespace=yolo_namespace,
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_detections_topic": "detections",
                        "input_camera_info_topic": input_camera_info_topic,
                        "output_detections_corners_topic": "detections_with_corners",
                    }
                ],
            ),
        ]
    )