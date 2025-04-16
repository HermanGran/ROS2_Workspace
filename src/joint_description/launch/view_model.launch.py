import os

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Sti til urdf/xacro-fila
    model_path = PathJoinSubstitution([
        FindPackageShare('joint_description'),
        'urdf',
        'joint_model.macro.urdf'  # ← bytt til .xacro hvis du lager det senere
    ])

    # Sti til RViz-konfigfil
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('joint_description'),
        'config',
        'config.rviz'
    ])

    # Node for joint_state_publisher_gui
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Node for robot_state_publisher med xacro-command
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', model_path]),
                value_type=str
            )
        }],
        output='screen'
    )

    # RViz2 node med config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
