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
        DeclareLaunchArgument('datasetDir', default_value='/home/agilex/data'),
        DeclareLaunchArgument('episodeIndex', default_value='-1'),
        DeclareLaunchArgument('timeDiffLimit', default_value='0.03')
    ]

    type = LaunchConfiguration('type')
    dataset_dir = LaunchConfiguration('datasetDir')
    episode_index = LaunchConfiguration('episodeIndex')
    time_diff_limit = LaunchConfiguration('timeDiffLimit')

    return LaunchDescription(declared_arguments + [
        Node(
            package='data_tools',
            executable='data_tools_dataSync',
            name='data_sync',
            parameters=[
                [TextSubstitution(text=os.path.join(share_dir, 'config/')), type, TextSubstitution(text='_data_params.yaml')],
                {
                    'episodeIndex': episode_index,
                    'datasetDir': dataset_dir,
                    'timeDiffLimit': time_diff_limit
                }
            ],
            output='screen'
        )
    ])
