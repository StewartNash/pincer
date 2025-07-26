#from launch import LaunchDescription
#from launch_ros.actions import Node
#
#
#def generate_launch_description():
#    return LaunchDescription([
#        Node(
#            package="moveit_ros_perception",
#            executable="depth_image_octomap_updater",
#            name="depth_image_octomap_updater",
#            output="screen",
#            parameters=[{
#                "sensor_config": "/path/to/sensors_3d.yaml",  # Overwritten at runtime by sensor_manager.launch.py
#                "octomap_frame": "world",
#                "max_range": 5.0,
#                "octomap_resolution": 0.025,
#            }]
#        )
#    ])

