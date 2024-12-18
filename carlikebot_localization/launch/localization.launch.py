from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition


def generate_launch_description():


    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False",
        description="Set to true to use the python implementation; otherwise, the cpp implementation will be used."
    )


    use_python = LaunchConfiguration("use_python")


    simple_odometry_node_py = Node(
        package="carlikebot_localization",
        executable="simple_odometry.py",
        condition=IfCondition(use_python)
    )

 
    simple_odometry_node_cpp = Node(
        package="carlikebot_localization",
        executable="simple_odometry",
        condition=UnlessCondition(use_python)
    )


    return LaunchDescription([
        use_python_arg,
        simple_odometry_node_py,
        simple_odometry_node_cpp
    ])
