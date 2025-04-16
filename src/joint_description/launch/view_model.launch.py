import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Hent URDF/XACRO-fil
    xacro_file = os.path.join(
        get_package_share_directory('joint_description'),
        'urdf',
        'joint_model.urdf'  # eller .xacro
    )

    robot_description_config = xacro.process_file(xacro_file)
    robot_description_content = robot_description_config.toxml()

    # Hent RViz-konfigfil
    rviz_config_file = os.path.join(
        get_package_share_directory('joint_description'),
        'config',
        'config.rviz'  # ← denne må eksistere
    )

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
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
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
