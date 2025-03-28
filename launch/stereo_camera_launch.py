from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument('video_port', default_value='6', description='Video port of the camera'),
        DeclareLaunchArgument('fps', default_value='60', description='Frames per second'),
        DeclareLaunchArgument('image_width', default_value='1600', description='Image width'),
        DeclareLaunchArgument('image_height', default_value='600', description='Image height'),
        DeclareLaunchArgument('vis_cv_image', default_value='true', description='Visualize CV image'),
        DeclareLaunchArgument('left_camera_info_url', default_value='package://elp_stereo_camera/cam_params/1280_480/left_camera.yaml', description='Left camera info URL'),
        DeclareLaunchArgument('right_camera_info_url', default_value='package://elp_stereo_camera/cam_params/1280_480/right_camera.yaml', description='Right camera info URL'),

        # Stereo camera node
        Node(
            package='elp_stereo_camera',
            executable='stereo_camera_node',
            name='stereo_camera_node',
            output='screen',
            parameters=[{
                'video_port': LaunchConfiguration('video_port'),
                'fps': LaunchConfiguration('fps'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'vis_cv_image': LaunchConfiguration('vis_cv_image'),
                'left_camera_info_url': LaunchConfiguration('left_camera_info_url'),
                'right_camera_info_url': LaunchConfiguration('right_camera_info_url'),
            }]
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