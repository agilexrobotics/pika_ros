import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('sensor_tools')

    declared_arguments = [
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('fisheye_port', default_value='22'),
    ]

    camera_fps = LaunchConfiguration('camera_fps')
    camera_height = LaunchConfiguration('camera_height')
    camera_width = LaunchConfiguration('camera_width')
    fisheye_port = LaunchConfiguration('fisheye_port')

    return LaunchDescription(declared_arguments+[
        Node(
            package='sensor_tools',
            executable='usb_camera.py',
            name='camera_fisheye',
            parameters=[{'camera_port': fisheye_port,
                         'camera_fps': camera_fps,
                         'camera_height': camera_height,
                         'camera_width': camera_width}],
            remappings=[
                ('/camera_rgb/color/image_raw', '/camera_fisheye/color/image_raw'),
            ],
            output='screen'
        )
    ])
