from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="control",
            executable="aruco_tracker",
            name="aruco_tracker",
            output="screen",
            parameters=[
                {"image_sub_topic": "/camera/color/image_raw"},
                {"camera_info_sub_topic": "/camera/color/camera_info"},
                {"aruco_id": 0},
                {"dictionary": 0},   # cv::aruco::DICT_4X4_250
                {"marker_size": 0.05}
            ]
        )
    ])



# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import ExecuteProcess
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare

# def generate_launch_description():
#     return LaunchDescription([
#         # Run bridge nodes directly without screen
#         ExecuteProcess(
#             cmd=[
#                 'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
#                 '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image'
#             ],
#             name='image_bridge_process',
#             output='screen'
#         ),
#         ExecuteProcess(
#             cmd=[
#                 'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
#                 '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
#             ],
#             name='camera_info_bridge_process',
#             output='screen'
#         ),
               
#         # Aruco tracker node
#         Node(
#             package='aruco_tracker',
#             executable='aruco_tracker',
#             name='aruco_tracker',
#             output='screen',
#             parameters=[
#                 PathJoinSubstitution([FindPackageShare('aruco_tracker'), 'cfg', 'params.yaml'])
#             ]
#         ),
#     ])