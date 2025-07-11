from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource, XMLLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument("debug", default_value="false"),
        DeclareLaunchArgument("info", default_value=LaunchConfiguration("debug")),
        DeclareLaunchArgument("pipeline", default_value="ompl"),
        DeclareLaunchArgument("allow_trajectory_execution", default_value="true"),
        DeclareLaunchArgument("fake_execution", default_value="false"),
        DeclareLaunchArgument("fake_execution_type", default_value="interpolate"),
        DeclareLaunchArgument("max_safe_path_cost", default_value="1"),
        DeclareLaunchArgument("jiggle_fraction", default_value="0.05"),
        DeclareLaunchArgument("publish_monitored_planning_scene", default_value="true"),
        DeclareLaunchArgument("capabilities", default_value=""),
        DeclareLaunchArgument("disable_capabilities", default_value=""),
        DeclareLaunchArgument("load_robot_description", default_value="true"),
    ]

    # Launch prefix and command args depending on debug/info
    launch_prefix = LaunchConfiguration("launch_prefix", default="")
    command_args = LaunchConfiguration("command_args", default="")

    gdb_prefix = "gdb -x " + str(PathJoinSubstitution([
        FindPackageShare("arctos_config"), "launch", "gdb_settings.gdb"
    ])) + " --ex run --args"

    # Include planning_context.launch.py
    planning_context = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("arctos_config"), "launch", "planning_context.launch.py"
        ])),
        launch_arguments={
            "load_robot_description": LaunchConfiguration("load_robot_description")
        }.items()
    )

    # Include planning_pipeline.launch.xml
    planning_pipeline = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("arctos_config"), "launch", "planning_pipeline.launch.xml"
        ])),
        launch_arguments={
            "pipeline": LaunchConfiguration("pipeline"),
            "capabilities": LaunchConfiguration("capabilities"),
            "disable_capabilities": LaunchConfiguration("disable_capabilities")
        }.items(),
        namespace="move_group"
    )

    # Include trajectory_execution.launch.xml
    trajectory_execution = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("arctos_config"), "launch", "trajectory_execution.launch.xml"
        ])),
        condition=IfCondition(LaunchConfiguration("allow_trajectory_execution")),
        launch_arguments={
            "moveit_manage_controllers": "true",
            "moveit_controller_manager": LaunchConfiguration("fake_execution").perform({}) == "true" and "fake" or "arctos_urdf",
            "fake_execution_type": LaunchConfiguration("fake_execution_type")
        }.items(),
        namespace="move_group"
    )

    # Include sensor_manager.launch.xml
    sensor_manager = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("arctos_config"), "launch", "sensor_manager.launch.xml"
        ])),
        condition=IfCondition(LaunchConfiguration("allow_trajectory_execution")),
        launch_arguments={
            "moveit_sensor_manager": "arctos_urdf"
        }.items(),
        namespace="move_group"
    )

    # move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        prefix=IfCondition(LaunchConfiguration("debug"), gdb_prefix),
        arguments=IfCondition(LaunchConfiguration("info"), ["--debug"]),
        parameters=[{
            "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
            "max_safe_path_cost": LaunchConfiguration("max_safe_path_cost"),
            "jiggle_fraction": LaunchConfiguration("jiggle_fraction"),
            "planning_scene_monitor": {
                "publish_planning_scene": LaunchConfiguration("publish_monitored_planning_scene"),
                "publish_geometry_updates": LaunchConfiguration("publish_monitored_planning_scene"),
                "publish_state_updates": LaunchConfiguration("publish_monitored_planning_scene"),
                "publish_transforms_updates": LaunchConfiguration("publish_monitored_planning_scene"),
            }
        }],
        env={"DISPLAY": LaunchConfiguration("DISPLAY", default=":0")}
    )

    return LaunchDescription(declared_arguments + [
        planning_context,
        planning_pipeline,
        trajectory_execution,
        sensor_manager,
        move_group_node,
    ])

