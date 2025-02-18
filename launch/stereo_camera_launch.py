from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from pathlib import Path


ELP_STEREO_CAMERA_DIR = get_package_share_directory('elp_stereo_camera')
param_path=Path(ELP_STEREO_CAMERA_DIR, 'config', 'stereo_camera_params.yaml')

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='elp_stereo_camera',
            executable='stereo_camera_node',
            name='stereo_camera_node',
            output='screen',
            parameters=[param_path]
        )
    ])