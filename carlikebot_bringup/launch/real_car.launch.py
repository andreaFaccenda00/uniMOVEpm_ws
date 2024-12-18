from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import os
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():


    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
                                      description="Use simulated time"
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(get_package_share_directory("carlikebot_bringup"), "config", "joy_teleop.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("carlikebot_bringup"), "config", "joy_config.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    carlikebotTf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros2_control_demo_carlikebot"),
                "launch",
                "carlikebot.launch.py"
            ])
        )
    )

    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("carlikebot_localization"),
                "launch",
                "localization.launch.py"
            ])
        )
    )




    return LaunchDescription([
        carlikebotTf_launch,
        odometry_launch,
        use_sim_time_arg,
        joy_teleop,
        joy_node
    ])


