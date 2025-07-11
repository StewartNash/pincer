from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pipeline = LaunchConfiguration("pipeline")

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument("pipeline", default_value="ompl"),
        DeclareLaunchArgument("capabilities", default_value=""),
        DeclareLaunchArgument("disable_capabilities", default_value=""),

        # Dynamically include the chosen planning pipeline
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("arctos_config"),
                    "launch",
                    TextSubstitution(text=""),  # Needed to concatenate dynamic filename
                    LaunchConfiguration("pipeline"),
                    TextSubstitution(text="_planning_pipeline.launch.xml")
                ])
            ),
            launch_arguments={
                "capabilities": LaunchConfiguration("capabilities"),
                "disable_capabilities": LaunchConfiguration("disable_capabilities"),
            }.items()
        )
    ])

