from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Declare the database path argument
        DeclareLaunchArgument("moveit_warehouse_database_path"),

        # Include warehouse settings (assuming it's still in XML)
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("arctos_config"),
                    "launch",
                    "warehouse_settings.launch.xml"
                ])
            )
        ),

        # Launch the mongo_wrapper_ros.py node
        Node(
            package="warehouse_ros_mongo",
            executable="mongo_wrapper_ros.py",
            name="mongo_wrapper_ros",
            output="screen",
            parameters=[{
                "overwrite": False,
                "database_path": LaunchConfiguration("moveit_warehouse_database_path")
            }],
            cwd="ROS_HOME"
        )
    ])

