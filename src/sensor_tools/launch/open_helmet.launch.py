import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchContext


def generate_launch_description():

    share_dir = get_package_share_directory('sensor_tools')

    declared_arguments = [
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_profile', default_value='640x480x30'),
        DeclareLaunchArgument('fisheye_port', default_value='22'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB22'),
        DeclareLaunchArgument('joint_name', default_value='center_joint'),
        DeclareLaunchArgument('depth_camera_no', default_value='230322270688'),
    ]

    camera_fps = LaunchConfiguration('camera_fps')
    camera_height = LaunchConfiguration('camera_height')
    camera_width = LaunchConfiguration('camera_width')
    camera_profile = LaunchConfiguration('camera_profile')
    fisheye_port = LaunchConfiguration('fisheye_port')
    serial_port = LaunchConfiguration('serial_port')
    joint_name = LaunchConfiguration('joint_name')
    depth_camera_no = LaunchConfiguration('depth_camera_no')

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
        launch_arguments={'serial_no': depth_camera_no,
                          'camera_namespace': "",
                          'camera_name': "camera_h",
                          'rgb_camera.color_profile': camera_profile, 
                          'depth_module.color_profile': camera_profile, 
                          'depth_module.depth_profile': camera_profile,
                          'depth_module.infra_profile': camera_profile}.items()
    )

    return LaunchDescription(declared_arguments+[
        depth_camera_launch,
        Node(
            package='sensor_tools',
            executable='usb_camera.py',
            name='camera_fisheye',
            parameters=[{'camera_port': fisheye_port,
                         'camera_fps': camera_fps,
                         'camera_height': camera_height,
                         'camera_width': camera_width,
                         'camera_frame_id': "camera_fisheye_h_link"}],
            remappings=[
                ('/camera_rgb/color/image_raw', '/camera_fisheye_h/color/image_raw'),
                ('/camera_rgb/color/camera_info', '/camera_fisheye_h/color/camera_info')
            ],
            respawn=True,
            output='screen'
        ),
        Node(
            package='sensor_tools',
            executable='serial_gripper_imu',
            name='serial_gripper_imu',
            parameters=[{'serial_port': serial_port,
                         'joint_name': joint_name}],
            remappings=[
                ('/imu/data', '/imu_h/data'),
            ],
            respawn=True,
            output='screen'
        )
    ])
