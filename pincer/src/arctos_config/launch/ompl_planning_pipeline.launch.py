from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument("planning_plugin", default_value="ompl_interface/OMPLPlanner"),
        DeclareLaunchArgument("capabilities", default_value=""),
        DeclareLaunchArgument("disable_capabilities", default_value=""),
        DeclareLaunchArgument("planning_adapters", default_value="default_planner_request_adapters/AddTimeParameterization "
                                                              "default_planner_request_adapters/FixWorkspaceBounds "
                                                              "default_planner_request_adapters/FixStartStateBounds "
                                                              "default_planner_request_adapters/FixStartStateCollision "
                                                              "default_planner_request_adapters/FixStartStatePathConstraints"),
        DeclareLaunchArgument("start_state_max_bounds_error", default_value="0.1"),

        # Set parameters for OMPL planner and adapters
        SetParameter(name="planning_plugin", value=LaunchConfiguration("planning_plugin")),
        SetParameter(name="request_adapters", value=LaunchConfiguration("planning_adapters")),
        SetParameter(name="start_state_max_bounds_error", value=LaunchConfiguration("start_state_max_bounds_error")),
        SetParameter(name="capabilities", value=LaunchConfiguration("capabilities")),
        SetParameter(name="disable_capabilities", value=LaunchConfiguration("disable_capabilities")),

        # Load configuration parameters from file
        SetParameter(name="ompl_planning", value=PathJoinSubstitution([
            FindPackageShare("arctos_config"),
            "config",
            "ompl_planning.yaml"
        ])),
    ])

