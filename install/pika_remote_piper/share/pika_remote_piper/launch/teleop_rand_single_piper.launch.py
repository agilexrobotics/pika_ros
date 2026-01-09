import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    share_dir = get_package_share_directory('pika_remote_piper')

    declared_arguments = [
        DeclareLaunchArgument('paramsFile', default_value=os.path.join(share_dir, 'config', 'piper_rand_params.yaml')),
    ]
    parameter_file = LaunchConfiguration('paramsFile')

    locator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('piper'), 'launch', 'start_single_piper.launch.py')])
    )

    return LaunchDescription(declared_arguments+[
        locator_launch,
        Node(
            package='pika_remote_piper',
            executable='piper_FK.py',
            name='piper_FK',
            parameters=[{'index_name': '',
                         'paramsFile': parameter_file}],
            respawn=True,
            output='screen'
        ),
        Node(
            package='pika_remote_piper',
            executable='piper_IK.py',
            name='piper_IK',
            parameters=[{'index_name': '',
                         'paramsFile': parameter_file}],
            respawn=True,
            output='screen'
        ),
        Node(
            package='pika_remote_piper',
            executable='teleop_piper_publish.py',
            name='teleop_piper',
            parameters=[{'index_name': ''}],
            respawn=True,
            output='screen'
        ),
    ])
