import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    share_dir = get_package_share_directory('data_tools')
    
    declared_arguments = [
        DeclareLaunchArgument('type', default_value='aloha'),
        DeclareLaunchArgument('useService', default_value='false'),
        DeclareLaunchArgument('datasetDir', default_value='/home/agilex/data'),
        DeclareLaunchArgument('episodeIndex', default_value='0'),
        DeclareLaunchArgument('instructions', default_value='\[null\]'),
        DeclareLaunchArgument('hz', default_value='20'),
        DeclareLaunchArgument('timeout', default_value='2'),
        DeclareLaunchArgument('cropTime', default_value='1.0')
    ]

    type = LaunchConfiguration('type')
    use_service = LaunchConfiguration('useService')
    dataset_dir = LaunchConfiguration('datasetDir')
    episode_index = LaunchConfiguration('episodeIndex')
    instructions = LaunchConfiguration('instructions')
    hz = LaunchConfiguration('hz')
    timeout = LaunchConfiguration('timeout')
    crop_time = LaunchConfiguration('cropTime')

    return LaunchDescription(declared_arguments + [
        Node(
            package='data_tools',
            executable='data_tools_dataCapture',
            # name='data_capture',
            parameters=[
                [TextSubstitution(text=os.path.join(share_dir, 'config/')), type, TextSubstitution(text='_data_params.yaml')],
                {
                    'useService': use_service,
                    'datasetDir': dataset_dir,
                    'episodeIndex': episode_index,
                    'instructions': instructions,
                    'hz': hz,
                    'timeout': timeout,
                    'cropTime': crop_time
                }
            ],
            output='screen',
            # prefix='gnome-terminal --'
            prefix='gnome-terminal -- bash -c "$0 $@; echo \\"The program has exited, press enter to close the terminal...\\"; read"'  # 保持终端窗口打开
        )
    ])
