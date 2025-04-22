from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument("reset", default_value="false"),
        DeclareLaunchArgument(
            "moveit_warehouse_database_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("arctos_config"),
                "default_warehouse_mongo_db"
            ])
        ),

        # Include warehouse.launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("arctos_config"),
                    "launch",
                    "warehouse.launch.py"
                ])
            ),
            launch_arguments={
                "moveit_warehouse_database_path": LaunchConfiguration("moveit_warehouse_database_path")
            }.items()
        ),

        # Optionally launch reset node
        Node(
            package="moveit_ros_warehouse",
            executable="moveit_init_demo_warehouse",
            name="moveit_default_db_reset",
            output="screen",
            condition=IfCondition(LaunchConfiguration("reset"))
        ),
    ])

