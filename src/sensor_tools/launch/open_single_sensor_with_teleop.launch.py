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
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB50'),
        DeclareLaunchArgument('joint_name', default_value='center_joint')
    ]

    serial_port = LaunchConfiguration('serial_port')
    joint_name = LaunchConfiguration('joint_name')

    locator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('pika_locator'), 'launch', 'pika_single_locator.launch.py')])
    )

    return LaunchDescription(declared_arguments+[
        locator_launch,
        Node(
            package='sensor_tools',
            executable='serial_gripper_imu',
            name='serial_gripper_imu',
            parameters=[{'serial_port': serial_port,
                         'joint_name': joint_name}],
            remappings=[
                ('/imu/data', '/imu/data'),
                ('/gripper/data', '/gripper/data'),
                ('/gripper/ctrl', '/gripper/ctrl'),
                ('/gripper/joint_state', '/gripper/joint_state'),
                ('/gripper/joint_state_ctrl', '/joint_states'),
                ('/joint_state_info', '/joint_states'),
                ('/joint_state_gripper', '/joint_states_gripper'),
                ('/data_capture_status', '/data_tools_dataCapture/status'),
                ('/teleop_status', '/teleop_status'),
                ('/localization_status', '/pika_localization_status'),
                ('/arm_control_status', '/arm_control_status'),
                ('/teleop_trigger', '/teleop_trigger'),
                ('/data_tools_dataCapture/capture_service', '/data_tools_dataCapture/capture_service'),
                
                
                
                
            ],
            respawn=True,
            output='screen'
        )
    ])
