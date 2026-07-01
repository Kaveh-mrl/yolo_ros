from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Namespace as command line argument.
    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="", description="Namespace for the nodes.")
    robot_name = LaunchConfiguration("robot_name")

    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value='False', description="Use simulation time.")
    use_sim_time = LaunchConfiguration("use_sim_time")


    # And finally, launch the action server for gimbal control.
    gimbal_action_server_node = Node(
        package="yolo_smarc_actions",
        executable="yolo_action.py",
        name="yolo_action_server",
        namespace=robot_name,
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "startup_classes": ["person", "boat", "buoy"],
        }]
    )

    return LaunchDescription([
        robot_name_arg,
        sim_time_arg,
        gimbal_action_server_node 
    ])
