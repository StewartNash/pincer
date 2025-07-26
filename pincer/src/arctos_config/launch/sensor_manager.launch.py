from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "moveit_sensor_manager",
            default_value="arctos_urdf",
            description="Name of sensor manager launch file (without extension)",
        )
    ]

    return LaunchDescription(
        declared_arguments
        + [
            SetParameter(name="octomap_resolution", value=0.025),
            SetParameter(name="max_range", value=5.0),
            SetParameter(
                name="sensor_config",
                value=PathJoinSubstitution(
                    [FindPackageShare("arctos_config"), "config", "sensors_3d.yaml"]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("arctos_config"),
                            "launch",
                            LaunchConfiguration("moveit_sensor_manager"),
                            TextSubstitution(text="_moveit_sensor_manager.launch.py"),
                        ]
                    )
                )
            ),
        ]
    )

