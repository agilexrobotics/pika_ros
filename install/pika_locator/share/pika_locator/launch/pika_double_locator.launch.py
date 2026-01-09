#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('pika_locator')
    
    # 从环境变量读取设备代码，如果环境变量不存在则使用空字符串作为默认值
    left_hand_code_env = os.environ.get('pika_L_code', '')
    right_hand_code_env = os.environ.get('pika_R_code', '')
    
    # 声明launch参数
    dist_limit_arg = DeclareLaunchArgument(
        'dist_limit',
        default_value='0.2',
        description='Distance limit for pose filtering'
    )
    
    angle_limit_arg = DeclareLaunchArgument(
        'angle_limit',
        default_value='0.2',
        description='Angle limit for pose filtering'
    )
    
    linear_limit_arg = DeclareLaunchArgument(
        'linear_limit',
        default_value='5.0',
        description='Linear velocity limit'
    )
    
    angular_limit_arg = DeclareLaunchArgument(
        'angular_limit',
        default_value='20.0',
        description='Angular velocity limit'
    )

    left_hand_code_arg = DeclareLaunchArgument(
        'left_hand_code',
        default_value=left_hand_code_env,
        description='Left hand device code (from environment variable pika_L_code)'
    )

    right_hand_code_arg = DeclareLaunchArgument(
        'right_hand_code',
        default_value=right_hand_code_env,
        description='Right hand device code (from environment variable pika_R_code)'
    )

    # 创建pika_double_locator节点
    pika_double_locator_node = Node(
        package='pika_locator',
        executable='pika_double_locator_node',
        name='pika_double_locator',
        output='screen',
        parameters=[{
            'dist_limit': LaunchConfiguration('dist_limit'),
            'angle_limit': LaunchConfiguration('angle_limit'),
            'linear_limit': LaunchConfiguration('linear_limit'),
            'angular_limit': LaunchConfiguration('angular_limit'),
            'left_hand_code': LaunchConfiguration('left_hand_code'),
            'right_hand_code': LaunchConfiguration('right_hand_code'),
        }]
    )

    # 创建rviz节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': False}]
    )

    # 声明rviz配置文件参数
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=share_dir+'/rviz/vive.rviz',
        description='Path to rviz config file'
    )

    return LaunchDescription([
        dist_limit_arg,
        angle_limit_arg,
        linear_limit_arg,
        angular_limit_arg,
        left_hand_code_arg,
        right_hand_code_arg,
        rviz_config_arg,
        pika_double_locator_node,
        rviz_node
    ]) 