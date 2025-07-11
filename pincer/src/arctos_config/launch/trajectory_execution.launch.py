from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument("fake_execution_type", default_value="interpolate"),
        DeclareLaunchArgument("moveit_manage_controllers", default_value="true"),
        DeclareLaunchArgument("moveit_controller_manager", default_value="arctos_urdf"),

        # Parameters for trajectory execution
        SetParameter(name="moveit_manage_controllers", value=LaunchConfiguration("moveit_manage_controllers")),
        SetParameter(name="trajectory_execution/allowed_execution_duration_scaling", value=1.2),
        SetParameter(name="trajectory_execution/allowed_goal_duration_margin", value=0.5),
        SetParameter(name="trajectory_execution/allowed_start_tolerance", value=0.01),

        # Dynamically include the controller manager launch file (can be fake or real)
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("arctos_config"),
                    "launch",
                    LaunchConfiguration("moveit_controller_manager"),
                    TextSubstitution(text="_moveit_controller_manager.launch.xml")
                ])
            ),
            launch_arguments={
                "fake_execution_type": LaunchConfiguration("fake_execution_type"),
                "moveit_manage_controllers": LaunchConfiguration("moveit_manage_controllers"),
                "moveit_controller_manager": LaunchConfiguration("moveit_controller_manager")
            }.items()
        )
    ])

