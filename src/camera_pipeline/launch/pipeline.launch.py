from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Image rectification
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_node',
            remappings=[
                ('image', '/image_raw'),
                ('camera_info', '/camera_info'),
            ],
            output='screen'
        ),

        # Gaussian blur node
        Node(
            package='camera_pipeline',
            executable='gaussian_blur',
            name='gaussian_blur',
            remappings=[
                ('image_rect', '/image_rect'),
                ('image_blurred', '/image_blurred')
            ],
            output='screen'
        ),

        # Canny edge node
        Node(
            package='camera_pipeline',
            executable='canny_edge',
            name='canny_edge',
            remappings=[
                ('image_blurred', '/image_blurred'),
                ('image_output', '/image_output')
            ],
            output='screen'
        )
    ])
