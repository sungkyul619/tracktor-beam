# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import ExecuteProcess
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare

# def generate_launch_description():
#     return LaunchDescription([
#         # Run bridge nodes directly without screen
#         ExecuteProcess(
#             cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/camera@sensor_msgs/msg/Image@gz.msgs.Image'],
#             name='image_bridge_process',
#             output='screen',
#         ),
#         ExecuteProcess(
#             cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
#             name='camera_info_bridge_process',
#             output='screen',
#         ),
#         # Aruco tracker node
#         Node(
#             package='aruco_tracker',
#             executable='aruco_tracker',
#             name='aruco_tracker',
#             output='screen',
#             parameters=[
#                 PathJoinSubstitution([FindPackageShare('aruco_tracker'), 'cfg', 'params_orgo.yaml'])
#             ]
#         ),
#     ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # v4l2_camera node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480],
                'pixel_format': 'YUYV',
            }]
        ),
  
        # # Run bridge nodes
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        #          '/camera@sensor_msgs/msg/Image@gz.msgs.Image'],
        #     name='image_bridge_process',
        #     output='screen',
        # ),
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        #          '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        #     name='camera_info_bridge_process',
        #     output='screen',
        # ),

        # Aruco tracker node
        Node(
            package='aruco_tracker',
            executable='aruco_tracker',
            name='aruco_tracker',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('aruco_tracker'),
                    'cfg',
                    'params_orgo.yaml'
                ])
            ]
        ),
    ])
