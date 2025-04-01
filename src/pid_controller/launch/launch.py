from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('kp', default_value='1.0'),
        DeclareLaunchArgument('ki', default_value='0.0'),
        DeclareLaunchArgument('kd', default_value='0.0'),
    
    
        Node(
            package='pid_controller',
            executable='pid_controller_node',
            name='pid_controller_node',
            output='screen',
            parameters=[{
                'p': LaunchConfiguration('kp'),
                'i': LaunchConfiguration('ki'),
                'd': LaunchConfiguration('kd'),
            }]
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
