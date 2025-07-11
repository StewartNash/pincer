from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument("pipeline", default_value="ompl"),
        DeclareLaunchArgument("db", default_value="false"),
        DeclareLaunchArgument("db_path", default_value=PathJoinSubstitution([
            FindPackageShare("arctos_config"), "default_warehouse_mongo_db"
        ])),
        DeclareLaunchArgument("debug", default_value="false"),
        DeclareLaunchArgument("load_robot_description", default_value="true"),
        DeclareLaunchArgument("fake_execution_type", default_value="interpolate"),
        DeclareLaunchArgument("use_gui", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
    ]

    # Joint State Publisher (non-GUI)
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=UnlessCondition(LaunchConfiguration("use_gui")),
        parameters=[{
            "source_list": ["move_group/fake_controller_joint_states"]
        }]
    )

    # Joint State Publisher (GUI)
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("use_gui")),
        parameters=[{
            "source_list": ["move_group/fake_controller_joint_states"]
        }]
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        respawn=True
    )

    # Move Group Launch Include
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("arctos_config"), "launch", "move_group.launch.py"
            ])
        ),
        launch_arguments={
            "allow_trajectory_execution": "true",
            "fake_execution": "true",
            "fake_execution_type": LaunchConfiguration("fake_execution_type"),
            "info": "true",
            "debug": LaunchConfiguration("debug"),
            "pipeline": LaunchConfiguration("pipeline"),
            "load_robot_description": LaunchConfiguration("load_robot_description")
        }.items()
    )

    # RViz Launch Include
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("arctos_config"), "launch", "moveit_rviz.launch.py"
            ])
        ),
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        launch_arguments={
            "rviz_config": PathJoinSubstitution([
                FindPackageShare("arctos_config"), "launch", "moveit.rviz"
            ]),
            "debug": LaunchConfiguration("debug")
        }.items()
    )

    # Warehouse DB Include
    db_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("arctos_config"), "launch", "default_warehouse_db.launch.py"
            ])
        ),
        condition=IfCondition(LaunchConfiguration("db")),
        launch_arguments={
            "moveit_warehouse_database_path": LaunchConfiguration("db_path")
        }.items()
    )

    return LaunchDescription(
        declared_arguments + [
            joint_state_publisher,
            joint_state_publisher_gui,
            robot_state_publisher,
            move_group_launch,
            rviz_launch,
            db_launch
        ]
    )

