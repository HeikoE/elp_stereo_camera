from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_proc',
            executable='rectify_node',
            name='left_image_proc',
            namespace='stereo/left',
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image_rect', 'image_rect')
            ]
        ),
        Node(
            package='image_proc',
            executable='rectify_node',
            name='right_image_proc',
            namespace='stereo/right',
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image_rect', 'image_rect')
            ]
        )
    ])