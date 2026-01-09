import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    share_dir = get_package_share_directory('data_tools')
    
    declared_arguments = [
        DeclareLaunchArgument('useService', default_value='false'),
        DeclareLaunchArgument('datasetDir', default_value='/home/agilex/data'),
        DeclareLaunchArgument('episodeIndex', default_value='0'),
        DeclareLaunchArgument('paramsFile', default_value=os.path.join(share_dir, 'config', 'aloha_data_params.yaml')),
        DeclareLaunchArgument('hz', default_value='20'),
        DeclareLaunchArgument('timeout', default_value='2'),
    ]

    parameter_file = LaunchConfiguration('paramsFile')
    use_service = LaunchConfiguration('useService')
    dataset_dir = LaunchConfiguration('datasetDir')
    episode_index = LaunchConfiguration('episodeIndex')
    hz = LaunchConfiguration('hz')
    timeout = LaunchConfiguration('timeout')

    return LaunchDescription(declared_arguments+[
        Node(
            package='data_tools',
            executable='record_mcap.py',
            name='data_capture_to_mcap',
            output='screen',
            parameters=[
                {
                    'paramsFile': parameter_file,
                    'useService': use_service,
                    'datasetDir': dataset_dir,
                    'episodeIndex': episode_index,
                    'hz': hz,
                    'timeout': timeout,
                }
            ],
        ),
        Node(
            package='data_tools', 
            executable='compress_camera.py', 
            name="camera_l_compress_node",
            output='log',
            parameters=[{
                'topic_namespace': "camera_l",
            }],
        ),
        Node(
            package='data_tools', 
            executable='compress_camera.py', 
            name="camera_f_compress_node",
            output='log',
            parameters=[{
                'topic_namespace': "camera_f",
            }],
        ),
        Node(
            package='data_tools', 
            executable='compress_camera.py', 
            name="camera_r_compress_node",
            output='log',
            parameters=[{
                'topic_namespace': "camera_r",
            }],
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(share_dir,'launch', 'run_compress_camera.launch.py')),
        #     launch_arguments={
        #         'namespace_list': 'camera_l,camera_f'
        #     }.items(),
        # ),
    ])
