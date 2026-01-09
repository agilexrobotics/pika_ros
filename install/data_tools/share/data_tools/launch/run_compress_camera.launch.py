from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_service import LaunchService
import sys

def p(context):
    argv = context.argv
    print("****", argv)
    for arg in argv:
        if arg.find(":=") != -1:
            key, value = arg.split(':=')
            context.launch_configurations[key] = value
launch_service = LaunchService(argv=sys.argv[1:])
if "namespace_list" not in launch_service.context.launch_configurations.keys():
    launch_service.context.launch_configurations["namespace_list"] = "camera_l,camera_f,camera_r"
p(launch_service.context)

def generate_launch_description():
    return LaunchDescription([
        *[
            Node(
                package='data_tools',
                executable='compress_camera.py',
                name=f"{namespace}_compress_node",
                output='log',
                parameters=[{
                    'topic_namespace': namespace,
                }],
            ) for namespace in LaunchConfiguration('namespace_list').perform(launch_service.context).split(",")
        ]
        
    ])


# from launch import LaunchDescription
# # from launch_ros.actions import Node, IncludeLaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='data_tools', 
#             executable='compress_camera.py', 
#             name="camera_left_compress_node",
#             output='log',
#             parameters=[{
#                 'topic_namespace': "camera_l",
#             }],
#         ),
#         Node(
#             package='data_tools', 
#             executable='compress_camera.py', 
#             name="camera_front_compress_node",
#             output='log',
#             parameters=[{
#                 'topic_namespace': "camera_f",
#             }],
#         ),
#         Node(
#             package='data_tools', 
#             executable='compress_camera.py', 
#             name="camera_right_compress_node",
#             output='log',
#             parameters=[{
#                 'topic_namespace': "camera_r",
#             }],
#         ),
#     ])