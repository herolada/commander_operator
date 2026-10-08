from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='commander_operator',
            executable='commander_operator_node',
            name='commander_operator',
            parameters=[
                {'ecef_frame': 'FP_ECEF'},
            ],
            remappings=[
                ('switch_mode', 'crl_commander/switch_mode'),
                # ('operator_goal', 'operator_goal'),
                # ('operator_sequence', 'operator_sequence'),
            ],
        ),
    ])
