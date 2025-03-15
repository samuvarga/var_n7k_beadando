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
            package='sierpinski_triangle',
            executable='sierpinski_triangle.py',
            output='screen'
        ),
    ])
