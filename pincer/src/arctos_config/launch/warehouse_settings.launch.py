from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument("moveit_warehouse_port", default_value="33829"),
        DeclareLaunchArgument("moveit_warehouse_host", default_value="localhost"),

        # Set warehouse parameters
        SetParameter(name="warehouse_port", value=LaunchConfiguration("moveit_warehouse_port")),
        SetParameter(name="warehouse_host", value=LaunchConfiguration("moveit_warehouse_host")),
        SetParameter(name="warehouse_exec", value="mongod"),
        SetParameter(name="warehouse_plugin", value="warehouse_ros_mongo::MongoDatabaseConnection"),
    ])

