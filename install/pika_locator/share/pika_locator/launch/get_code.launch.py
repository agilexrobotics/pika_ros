#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('pika_locator')
    # 创建get_code节点
    get_code_node = Node(
        package='pika_locator',
        executable='get_code',
        name='get_code',
        output='screen'
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
        get_code_node,
        rviz_config_arg,
        rviz_node
    ]) 