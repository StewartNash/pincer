from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "launch_filename",
            default_value="moveit_planning_pipeline",
            description="Launch file name to include",
        ),
        DeclareLaunchArgument(
            "fake_execution",
            default_value="true",
            description="Use fake execution (bool)",
        ),
    ]

    launch_file = PathJoinSubstitution(
        [
            FindPackageShare("arctos_config"),
            "launch",
            LaunchConfiguration("launch_filename"),
            TextSubstitution(text=".launch.py"),
        ]
    )

    return LaunchDescription(
        declared_arguments
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={"fake_execution": LaunchConfiguration("fake_execution")}.items(),
            ),
        ]
    )

