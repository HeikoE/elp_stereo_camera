from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

ELP_STEREO_CAMERA_DIR = get_package_share_directory('elp_stereo_camera')
param_path = Path(ELP_STEREO_CAMERA_DIR, 'config', 'stereo_camera_params.yaml')

def generate_launch_description():
    return LaunchDescription([

        # Stereo camera node
        Node(
            package='elp_stereo_camera',
            executable='stereo_camera_node',
            name='stereo_camera_node',
            output='screen',
            parameters=[param_path]
        ),

        # Static transform for camera_link (world-oriented) to camera_optical_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_to_camera_optical_link',
            arguments=['0', '0', '0', '-1.5707963267948966', '0', '-1.5707963267948966', 'camera_link', 'camera_optical_link']
        ),

        # Static transform for camera_optical_link to left_camera
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_link_to_left_camera',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_optical_link', 'left_camera']
        ),

        # Static transform for camera_optical_link to right_camera (baseline of 60mm)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_link_to_right_camera',
            arguments=['0.06', '0', '0', '0', '0', '0', 'camera_optical_link', 'right_camera']
        )
    ])