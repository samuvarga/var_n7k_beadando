from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            #namespace='turtle1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='var_n7k_beadando',
            executable='archery_target_node',
            output='screen'
        ),
        Node(
            package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[
            {'port': 8765},
            {'address': '0.0.0.0'},
            {'tls': False},
            {'certfile': ''},
            {'keyfile': ''},
            #{'topic_whitelist': "'.*'"},
            {'max_qos_depth': 10},
            {'num_threads': 0},
            {'use_sim_time': False},
        ]
        ),
    ])
