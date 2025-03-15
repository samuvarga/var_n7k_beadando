import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class MultipleTurtlesNode(Node):
    def __init__(self):
        super().__init__('multiple_turtles')
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Spawn turtles at specific locations
        self.spawn_turtles()

    def spawn_turtles(self):
        # Turtle1 is spawned at the default position by the simulator
        self.spawn_turtle('turtle2', 5, 5)  # This is where we start drawing
        self.spawn_turtle('turtle3', 5, 6.5)  # 1.5 units away
        self.spawn_turtle('turtle4', 5, 7)    # 3 units away
        self.spawn_turtle('turtle5', 5, 8)    # 4 units away

    def spawn_turtle(self, name, x, y):
        # Spawn the turtle in the simulator at a specific position
        spawn_cmd = f"/spawn {x} {y} 0.0 {name}"
        self.get_logger().info(f"Spawning turtle at {x}, {y}")
        self.call_service(spawn_cmd)

        if name != 'turtle2':  # Turtle2 does not draw
            self.draw_circle(name, x, y)

    def draw_circle(self, name, x, y):
        # Set the turtle's position to the center (5, 5) and draw a circle
        radius = self.calculate_radius(x, y)
        self.get_logger().info(f"{name} drawing circle with radius {radius}")

        # Command turtle to move in a circle
        cmd_msg = Twist()
        cmd_msg.linear.x = radius
        cmd_msg.angular.z = 2 * math.pi / 360  # Small step for smooth turning
        self.cmd_pub.publish(cmd_msg)

    def calculate_radius(self, x, y):
        # Calculate distance from (5, 5) (center point) to (x, y)
        return math.sqrt((5 - x) ** 2 + (5 - y) ** 2)


def main(args=None):
    rclpy.init(args=args)
    multiple_turtles_node = MultipleTurtlesNode()
    rclpy.spin(multiple_turtles_node)
    multiple_turtles_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
