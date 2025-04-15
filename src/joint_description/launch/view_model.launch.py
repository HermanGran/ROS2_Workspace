from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    robot_description_content = '''<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot">
        <link name="world"/>
    </robot>'''

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[]
        )
    ])
