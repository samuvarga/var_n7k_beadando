from turtlesim.srv import Spawn
import rclpy
from rclpy.node import Node

class MultipleTurtlesNode(Node):
    def __init__(self):
        super().__init__('multiple_turtles_node')

        # 1. Teknős létrehozása
        self.spawn_turtle(2.0, 2.0, 'turtle1')

        # 2. Teknős létrehozása
        self.spawn_turtle(8.0, 8.0, 'turtle2')

    def spawn_turtle(self, x, y, name):
        """Technikai funkció a teknős létrehozására"""
        spawn_client = self.create_client(Spawn, '/spawn')

        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        request.name = name
        spawn_client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    multiple_turtles_node = MultipleTurtlesNode()
    rclpy.spin(multiple_turtles_node)
    multiple_turtles_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
