from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    return LaunchDescription([
        DeclareLaunchArgument("debug", default_value="false"),
        DeclareLaunchArgument("rviz_config", default_value=""),

        # Launch RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                LaunchConfiguration("rviz_config")
            ],
            condition=UnlessCondition(LaunchConfiguration("rviz_config"))
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d", LaunchConfiguration("rviz_config")
            ],
            condition=IfCondition(LaunchConfiguration("rviz_config"))
        )
    ])

