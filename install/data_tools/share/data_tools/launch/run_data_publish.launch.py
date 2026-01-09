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
        DeclareLaunchArgument('publishIndex', default_value='-1'),
        DeclareLaunchArgument('datasetDir', default_value='/home/agilex/data'),
        DeclareLaunchArgument('episodeIndex', default_value='0'),
        DeclareLaunchArgument('publishRate', default_value='30'),
    ]

    type = LaunchConfiguration('type')
    publish_index = LaunchConfiguration('publishIndex')
    dataset_dir = LaunchConfiguration('datasetDir')
    episode_index = LaunchConfiguration('episodeIndex')
    publish_rate = LaunchConfiguration('publishRate')

    return LaunchDescription(declared_arguments + [
        Node(
            package='data_tools',
            executable='data_tools_dataPublish',
            name='data_publish',
            parameters=[
                [TextSubstitution(text=os.path.join(share_dir, 'config/')), type, TextSubstitution(text='_data_params.yaml')],
                {
                    'episodeIndex': episode_index,
                    'datasetDir': dataset_dir,
                    'publishIndex': publish_index,
                    'publishRate': publish_rate
                }
            ],
            output='screen'
        )
    ])
