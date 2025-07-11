from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument("moveit_sensor_manager", default_value="arctos_urdf"),

        # Set parameters for sensors and octomap
        SetParameter(name="octomap_resolution", value=0.025),
        SetParameter(name="max_range", value=5.0),

        # Load 3D sensors configuration
        SetParameter(
            name="sensor_config",
            value=PathJoinSubstitution([FindPackageShare("arctos_config"), "config", "sensors_3d.yaml"])
        ),

        # Dynamically include the sensor manager launch file
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("arctos_config"),
                    "launch",
                    LaunchConfiguration("moveit_sensor_manager"),
                    TextSubstitution(text="_moveit_sensor_manager.launch.xml")
                ])
            )
        ),
    ])

