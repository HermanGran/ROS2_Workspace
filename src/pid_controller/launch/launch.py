import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pid_controller'),
        'config',
        'parameters.yaml'
        )

    return LaunchDescription([
        Node(
            package='pid_controller',
            executable='pid_controller_node',
            name='pid_controller_node',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='joint_simulator',
            executable='joint_simulator',
            name='joint_simulator',
            output='screen'
        ),
        Node(
            package='reference_input_node',
            executable='client',
            name='reference_input_node',
            output='screen'
        )
    ])
