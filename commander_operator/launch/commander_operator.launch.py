import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_data_dir = os.path.join(get_package_share_directory('commander_operator'), 'data')

    return LaunchDescription([
        DeclareLaunchArgument(
            'data_dir', default_value=default_data_dir,
            description='Folder the waypoints/routes are written to. Point it to the '
                        'package source to keep the recorded points in the repository.'),
        Node(
            package='commander_operator',
            executable='commander_operator_node',
            name='commander_operator',
            parameters=[
                {'ecef_frame': 'FP_ECEF'},
                {'data_dir': LaunchConfiguration('data_dir')},
            ],
            remappings=[
                ('switch_mode', 'crl_commander/switch_mode'),
                ('goal', '~/goal'),
                # ('operator_goal', 'operator_goal'),
                # ('operator_sequence', 'operator_sequence'),
            ],
        ),
    ])
