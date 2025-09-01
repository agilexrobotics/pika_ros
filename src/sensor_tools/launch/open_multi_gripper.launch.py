import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():

    share_dir = get_package_share_directory('sensor_tools')

    declared_arguments = [
        DeclareLaunchArgument('name', default_value=''),
        DeclareLaunchArgument('name_index', default_value=''),
        DeclareLaunchArgument('sub_name', default_value=''),
        DeclareLaunchArgument('ctrl', default_value='ctrl'),
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_profile', default_value='640x480x30'),
        DeclareLaunchArgument('l_fisheye_port', default_value='22'),
        DeclareLaunchArgument('r_fisheye_port', default_value='23'),
        DeclareLaunchArgument('l_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('r_serial_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('l_depth_camera_no', default_value='_230322270688'),
        DeclareLaunchArgument('r_depth_camera_no', default_value='_230322272619'),
        DeclareLaunchArgument('l_joint_name', default_value='gripper_l_center_joint'),
        DeclareLaunchArgument('r_joint_name', default_value='gripper_r_center_joint'),
        DeclareLaunchArgument('motor_current_limit', default_value='1000.0'),
        DeclareLaunchArgument('motor_current_redundancy', default_value='500.0'),
        DeclareLaunchArgument('mit_mode', default_value='true'),
        DeclareLaunchArgument('ctrl_rate', default_value='50.0')
    ]

    name = LaunchConfiguration('name')
    name_index = LaunchConfiguration('name_index')
    sub_name = LaunchConfiguration('sub_name')
    ctrl = LaunchConfiguration('ctrl')
    camera_fps = LaunchConfiguration('camera_fps')
    camera_height = LaunchConfiguration('camera_height')
    camera_width = LaunchConfiguration('camera_width')
    camera_profile = LaunchConfiguration('camera_profile')
    l_fisheye_port = LaunchConfiguration('l_fisheye_port')
    r_fisheye_port = LaunchConfiguration('r_fisheye_port')
    l_serial_port = LaunchConfiguration('l_serial_port')
    r_serial_port = LaunchConfiguration('r_serial_port')
    l_depth_camera_no = LaunchConfiguration('l_depth_camera_no')
    r_depth_camera_no = LaunchConfiguration('r_depth_camera_no')
    l_joint_name = LaunchConfiguration('l_joint_name')
    r_joint_name = LaunchConfiguration('r_joint_name')
    motor_current_limit = LaunchConfiguration('motor_current_limit')
    motor_current_redundancy = LaunchConfiguration('motor_current_redundancy')
    mit_mode = LaunchConfiguration('mit_mode')
    ctrl_rate = LaunchConfiguration('ctrl_rate')

    # 2. 创建基本启动描述
    ld = LaunchDescription(declared_arguments)
    
    # 3. 添加条件设置 - 必须在节点之前
    # 修正表达式语法：使用单引号包裹整个表达式
    ld.add_action(SetLaunchConfiguration(
        'sub_name',
        value=PythonExpression([
            "'sensor' if '",  # 注意单引号
            LaunchConfiguration('name'),
            "' == 'gripper' else ''"
        ])
    ))
    
    ld.add_action(SetLaunchConfiguration(
        'ctrl',
        value=PythonExpression([
            "'data' if '",   # 注意单引号
            LaunchConfiguration('name'),
            "' == 'gripper' else 'ctrl'"
        ])
    ))
    
    # 4. 添加调试日志
    ld.add_action(LogInfo(msg=['DEBUG: name = ', LaunchConfiguration('name')]))
    ld.add_action(LogInfo(msg=['DEBUG: sub_name = ', LaunchConfiguration('sub_name')]))
    ld.add_action(LogInfo(msg=['DEBUG: ctrl = ', LaunchConfiguration('ctrl')]))

    l_depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
        launch_arguments={'serial_no': l_depth_camera_no,
                          'camera_namespace': name,
                          'camera_name': "camera_l",
                          'rgb_camera.color_profile': camera_profile, 
                          'depth_module.color_profile': camera_profile, 
                          'depth_module.depth_profile': camera_profile,
                          'depth_module.infra_profile': camera_profile}.items()
    )
    r_depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
        launch_arguments={'serial_no': r_depth_camera_no,
                          'camera_namespace': name,
                          'camera_name': "camera_r",
                          'rgb_camera.color_profile': camera_profile, 
                          'depth_module.color_profile': camera_profile, 
                          'depth_module.depth_profile': camera_profile,
                          'depth_module.infra_profile': camera_profile}.items()
    )
    ld.add_action(l_depth_camera_launch)
    ld.add_action(r_depth_camera_launch)
    ld.add_action(Node(
            package='sensor_tools',
            executable='usb_camera.py',
            name=[name_index,TextSubstitution(text='camera_fisheye_l')],
            parameters=[{'camera_port': l_fisheye_port,
                         'camera_fps': camera_fps,
                         'camera_height': camera_height,
                         'camera_width': camera_width,
                         'camera_frame_id': [name,TextSubstitution(text="/camera_fisheye_l_link")]}],
            remappings=[
                ('/camera_rgb/color/image_raw', [name,TextSubstitution(text='/camera_fisheye_l/color/image_raw')]),
                ('/camera_rgb/color/camera_info', [name,TextSubstitution(text='/camera_fisheye_l/color/camera_info')])
            ],
            respawn=True,
            output='screen'
        ))
    ld.add_action(Node(
            package='sensor_tools',
            executable='usb_camera.py',
            name=[name_index,TextSubstitution(text='camera_fisheye_r')],
            parameters=[{'camera_port': r_fisheye_port,
                         'camera_fps': camera_fps,
                         'camera_height': camera_height,
                         'camera_width': camera_width,
                         'camera_frame_id': [name,TextSubstitution(text="/camera_fisheye_r_link")]}],
            remappings=[
                ('/camera_rgb/color/image_raw', [name,TextSubstitution(text='/camera_fisheye_r/color/image_raw')]),
                ('/camera_rgb/color/camera_info', [name,TextSubstitution(text='/camera_fisheye_r/color/camera_info')])
            ],
            respawn=True,
            output='screen'
        ))
    ld.add_action(Node(
            package='sensor_tools',
            executable='serial_gripper_imu',
            name=[name_index,TextSubstitution(text='serial_gripper_imu_l')],
            parameters=[{'serial_port': l_serial_port,
                         'joint_name': l_joint_name,
                         'motor_current_limit': motor_current_limit,
                         'motor_current_redundancy': motor_current_redundancy,
                         'mit_mode': mit_mode,
                         'ctrl_rate': ctrl_rate}],
            remappings=[
                ('/imu/data', [name,TextSubstitution(text='/imu_l/data')]),
                ('/gripper/data', [name,TextSubstitution(text='/gripper_l/data')]),
                ('/gripper/ctrl', [sub_name,TextSubstitution(text='/gripper_l/'), ctrl]),
                ('/gripper/joint_state', [name,TextSubstitution(text='/gripper_l/joint_state')]),
                ('/gripper/joint_state_ctrl', [name,TextSubstitution(text='/joint_states_l')]),
                ('/joint_state_info', '/joint_states_single_l'),
                ('/joint_state_gripper', '/joint_states_single_gripper_l'),
            ],
            respawn=True,
            output='screen'
        ))
    ld.add_action(Node(
            package='sensor_tools',
            executable='serial_gripper_imu',
            name=[name_index,TextSubstitution(text='serial_gripper_imu_r')],
            parameters=[{'serial_port': r_serial_port,
                         'joint_name': r_joint_name,
                         'motor_current_limit': motor_current_limit,
                         'motor_current_redundancy': motor_current_redundancy,
                         'mit_mode': mit_mode,
                         'ctrl_rate': ctrl_rate}],
            remappings=[
                ('/imu/data', [name,TextSubstitution(text='/imu_r/data')]),
                ('/gripper/data', [name,TextSubstitution(text='/gripper_r/data')]),
                ('/gripper/ctrl', [sub_name,TextSubstitution(text='/gripper_r/'), ctrl]),
                ('/gripper/joint_state', [name,TextSubstitution(text='/gripper_r/joint_state')]),
                ('/gripper/joint_state_ctrl', [name,TextSubstitution(text='/joint_states_r')]),
                ('/joint_state_info', '/joint_states_single_r'),
                ('/joint_state_gripper', '/joint_states_single_gripper_r'),
            ],
            respawn=True,
            output='screen'
        ))
    return ld
