from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import LoadComposableNodes, Node
from launch.actions import OpaqueFunction
from launch_ros.actions import SetParameter
from launch_ros.actions import PushRosNamespace
import os


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument("load_robot_description", default_value="false"),
        DeclareLaunchArgument("robot_description", default_value="robot_description"),
    ]

    load_robot_description = LaunchConfiguration("load_robot_description")
    robot_description_param = LaunchConfiguration("robot_description")

    # Resolve xacro path and command
    urdf_xacro_path = PathJoinSubstitution([
        FindPackageShare("arctos_urdf_description"),
        "urdf",
        "arctos_urdf.xacro"
    ])
    urdf_command = Command(["xacro ", urdf_xacro_path])

    # SRDF path
    srdf_path = PathJoinSubstitution([
        FindPackageShare("arctos_config"),
        "config",
        "arctos_urdf.srdf"
    ])

    # Joint and Cartesian limits
    joint_limits_yaml = PathJoinSubstitution([
        FindPackageShare("arctos_config"),
        "config",
        "joint_limits.yaml"
    ])
    cartesian_limits_yaml = PathJoinSubstitution([
        FindPackageShare("arctos_config"),
        "config",
        "cartesian_limits.yaml"
    ])

    # Kinematics config
    kinematics_yaml = PathJoinSubstitution([
        FindPackageShare("arctos_config"),
        "config",
        "kinematics.yaml"
    ])

    # Parameters to return
    return LaunchDescription(declared_arguments + [

        # Load robot description if enabled
        SetParameter(
            name=robot_description_param,
            value=urdf_command,
            condition=OpaqueFunction(lambda context: LaunchConfiguration("load_robot_description").perform(context) == "true")
        ),

        # Always load SRDF
        SetParameter(
            name=[robot_description_param, "_semantic"],
            value=srdf_path
        ),

        # Load planning group params
        GroupAction([
            PushRosNamespace([robot_description_param, "_planning"]),
            Node(
                package="rcl_yaml_param_parser",
                executable="yaml_param_parser",
                name="joint_limits_loader",
                parameters=[joint_limits_yaml, cartesian_limits_yaml],
                output="screen"
            )
        ]),

        # Load kinematics settings
        GroupAction([
            PushRosNamespace([robot_description_param, "_kinematics"]),
            Node(
                package="rcl_yaml_param_parser",
                executable="yaml_param_parser",
                name="kinematics_loader",
                parameters=[kinematics_yaml],
                output="screen"
            )
        ]),
    ])

