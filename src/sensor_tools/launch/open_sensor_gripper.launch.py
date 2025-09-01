import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    share_dir = get_package_share_directory('sensor_tools')

    declared_arguments = [
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_profile', default_value='640x480x30'),
        DeclareLaunchArgument('sensor_fisheye_port', default_value='22'),
        DeclareLaunchArgument('gripper_fisheye_port', default_value='23'),
        DeclareLaunchArgument('sensor_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('gripper_serial_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('sensor_depth_camera_no', default_value='230322270688'),
        DeclareLaunchArgument('gripper_depth_camera_no', default_value='230322272619'),
        DeclareLaunchArgument('sensor_joint_name', default_value='sensor_gripper_center_joint'),
        DeclareLaunchArgument('gripper_joint_name', default_value='gripper_gripper_center_joint'),
        DeclareLaunchArgument('motor_current_limit', default_value='1000.0'),
        DeclareLaunchArgument('motor_current_redundancy', default_value='500.0'),
        DeclareLaunchArgument('mit_mode', default_value='true'),
        DeclareLaunchArgument('ctrl_rate', default_value='50.0')
    ]
    camera_fps = LaunchConfiguration('camera_fps')
    camera_height = LaunchConfiguration('camera_height')
    camera_width = LaunchConfiguration('camera_width')
    camera_profile = LaunchConfiguration('camera_profile')
    sensor_fisheye_port = LaunchConfiguration('sensor_fisheye_port')
    gripper_fisheye_port = LaunchConfiguration('gripper_fisheye_port')
    sensor_serial_port = LaunchConfiguration('sensor_serial_port')
    gripper_serial_port = LaunchConfiguration('gripper_serial_port')
    sensor_depth_camera_no = LaunchConfiguration('sensor_depth_camera_no')
    gripper_depth_camera_no = LaunchConfiguration('gripper_depth_camera_no')
    sensor_joint_name = LaunchConfiguration('sensor_joint_name')
    gripper_joint_name = LaunchConfiguration('gripper_joint_name')
    motor_current_limit = LaunchConfiguration('motor_current_limit')
    motor_current_redundancy = LaunchConfiguration('motor_current_redundancy')
    mit_mode = LaunchConfiguration('mit_mode')
    ctrl_rate = LaunchConfiguration('ctrl_rate')

    locator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('pika_locator'), 'launch', 'pika_single_locator.launch.py')])
    )

    l_depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
        launch_arguments={'serial_no': sensor_depth_camera_no,
                          'camera_namespace': "sensor",
                          'camera_name': "camera",
                          'rgb_camera.color_profile': camera_profile, 
                          'depth_module.color_profile': camera_profile, 
                          'depth_module.depth_profile': camera_profile,
                          'depth_module.infra_profile': camera_profile}.items()
    )
    r_depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
        launch_arguments={'serial_no': gripper_depth_camera_no,
                          'camera_namespace': "gripper",
                          'camera_name': "camera",
                          'rgb_camera.color_profile': camera_profile, 
                          'depth_module.color_profile': camera_profile, 
                          'depth_module.depth_profile': camera_profile,
                          'depth_module.infra_profile': camera_profile}.items()
    )

    return LaunchDescription(declared_arguments+[
        locator_launch,
        l_depth_camera_launch,
        r_depth_camera_launch,
        Node(
            package='sensor_tools',
            executable='usb_camera.py',
            name='sensor_camera_fisheye',
            parameters=[{'camera_port': sensor_fisheye_port,
                         'camera_fps': camera_fps,
                         'camera_height': camera_height,
                         'camera_width': camera_width,
                         'camera_frame_id': "sensor/camera_fisheye_link"}],
            remappings=[
                ('/camera_rgb/color/image_raw', '/sensor/camera_fisheye/color/image_raw'),
                ('/camera_rgb/color/camera_info', '/sensor/camera_fisheye/color/camera_info')
            ],
            respawn=True,
            output='screen'
        ),
        Node(
            package='sensor_tools',
            executable='usb_camera.py',
            name='gripper_camera_fisheye',
            parameters=[{'camera_port': gripper_fisheye_port,
                         'camera_fps': camera_fps,
                         'camera_height': camera_height,
                         'camera_width': camera_width,
                         'camera_frame_id': "gripper/camera_fisheye_link"}],
            remappings=[
                ('/camera_rgb/color/image_raw', '/gripper/camera_fisheye/color/image_raw'),
                ('/camera_rgb/color/camera_info', '/gripper/camera_fisheye/color/camera_info')
            ],
            respawn=True,
            output='screen'
        ),
        Node(
            package='sensor_tools',
            executable='serial_gripper_imu',
            name='sensor_serial_gripper_imu',
            parameters=[{'serial_port': sensor_serial_port,
                         'joint_name': sensor_joint_name}],
            remappings=[
                ('/imu/data', '/sensor/imu/data'),
                ('/gripper/data', '/sensor/gripper/data'),
                ('/gripper/ctrl', '/sensor/gripper/ctrl'),
                ('/gripper/joint_state', '/sensor/gripper/joint_state'),
                ('/gripper/joint_state_ctrl', '/sensor/gripper/joint_state_ctrl'),
                ('/joint_state_info', '/joint_states'),
                ('/joint_state_gripper', '/joint_states_gripper'),
                ('/data_capture_status', '/data_tools_dataCapture/status'),
                ('/teleop_status', '/teleop_status'),
                ('/localization_status', '/pika_localization_status'),
                ('/arm_control_status', '/arm_control_status'),
            ],
            respawn=True,
            output='screen'
        ),
        Node(
            package='sensor_tools',
            executable='serial_gripper_imu',
            name='gripper_serial_gripper_imu',
            parameters=[{'serial_port': gripper_serial_port,
                         'joint_name': gripper_joint_name,
                         'motor_current_limit': motor_current_limit,
                         'motor_current_redundancy': motor_current_redundancy,
                         'mit_mode': mit_mode,
                         'ctrl_rate': ctrl_rate}],
            remappings=[
                ('/imu/data', '/imu/data'),
                ('/gripper/data', '/gripper/gripper/data'),
                ('/gripper/ctrl', '/gripper/gripper/ctrl'),
                ('/gripper/joint_state', '/gripper/gripper/joint_state'),
                ('/gripper/joint_state_ctrl', '/sensor/gripper/joint_state'),
                ('/joint_state_info', '/joint_states_single'),
                ('/joint_state_gripper', '/joint_states_single_gripper'),
            ],
            respawn=True,
            output='screen'
        )
    ])
