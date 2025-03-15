from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtle1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='var_n7k_beadando',
            executable='sierpinski_triangle.py',
            output='screen'
        ),
    ])
