import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    share_dir = get_package_share_directory('sensor_tools')

    declared_arguments = [
        DeclareLaunchArgument('name', default_value=''),
        DeclareLaunchArgument('name_index', default_value=''),
        DeclareLaunchArgument('l_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('r_serial_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('l_joint_name', default_value='gripper_l_center_joint'),
        DeclareLaunchArgument('r_joint_name', default_value='gripper_r_center_joint')
    ]
    name = LaunchConfiguration('name')
    name_index = LaunchConfiguration('name_index')
    l_serial_port = LaunchConfiguration('l_serial_port')
    r_serial_port = LaunchConfiguration('r_serial_port')
    l_joint_name = LaunchConfiguration('l_joint_name')
    r_joint_name = LaunchConfiguration('r_joint_name')

    locator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('pika_locator'), 'launch', 'pika_double_locator.launch.py')])
    )


    return LaunchDescription(declared_arguments+[
        locator_launch,
        Node(
            package='sensor_tools',
            executable='serial_gripper_imu',
            name=[name_index,TextSubstitution(text='serial_gripper_imu_l')],
            parameters=[{'serial_port': l_serial_port,
                         'joint_name': l_joint_name}],
            remappings=[
                ('/imu/data', [name,TextSubstitution(text='/imu_l/data')]),
                ('/gripper/data', [name,TextSubstitution(text='/gripper_l/data')]),
                ('/gripper/ctrl', [name,TextSubstitution(text='/gripper_l/ctrl')]),
                ('/gripper/joint_state', [name,TextSubstitution(text='/gripper_l/joint_state')]),
                ('/gripper/joint_state_ctrl', [name,TextSubstitution(text='/joint_states_l')]),
                ('/joint_state_info', '/joint_states_l'),
                ('/joint_state_gripper', '/joint_states_gripper_l'),
                ('/teleop_trigger', 'teleop_trigger_l'),
                ('/data_capture_status', '/data_tools_dataCapture/status'),
                ('/teleop_status', '/teleop_status_l'),
                ('/localization_status', '/pika_localization_status_l'),
                ('/arm_control_status', '/arm_control_status_l'),
                ('/data_tools_dataCapture/capture_service', '/data_tools_dataCapture/capture_service_l'),
            ],
            respawn=True,
            output='screen'
        ),
        Node(
            package='sensor_tools',
            executable='serial_gripper_imu',
            name=[name_index,TextSubstitution(text='serial_gripper_imu_r')],
            parameters=[{'serial_port': r_serial_port,
                         'joint_name': r_joint_name}],
            remappings=[
                ('/imu/data', [name,TextSubstitution(text='/imu_r/data')]),
                ('/gripper/data', [name,TextSubstitution(text='/gripper_r/data')]),
                ('/gripper/ctrl', [name,TextSubstitution(text='/gripper_r/ctrl')]),
                ('/gripper/joint_state', [name,TextSubstitution(text='/gripper_r/joint_state')]),
                ('/gripper/joint_state_ctrl', [name,TextSubstitution(text='/joint_states_r')]),
                ('/joint_state_info', '/joint_states_r'),
                ('/joint_state_gripper', '/joint_states_gripper_r'),
                ('/teleop_trigger', 'teleop_trigger_r'),
                ('/data_capture_status', '/data_tools_dataCapture/status'),
                ('/teleop_status', '/teleop_status_r'),
                ('/localization_status', '/pika_localization_status_r'),
                ('/arm_control_status', '/arm_control_status_r'),
                ('/data_tools_dataCapture/capture_service', '/data_tools_dataCapture/capture_service_r'),
            ],
            respawn=True,
            output='screen'
        )
    ])
