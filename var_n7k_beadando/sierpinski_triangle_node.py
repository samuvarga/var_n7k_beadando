import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

class MultipleTurtlesNode(Node):
    def __init__(self):
        super().__init__('multiple_turtles_node')

        # 4 teknős létrehozása, mindegyik 2 egységgel magasabb, mint az előző
        self.spawn_turtle(0.0, 2.0, 'turtle1')  # Első teknős
        self.spawn_turtle(0.0, 4.0, 'turtle2')  # Második teknős
        self.spawn_turtle(0.0, 6.0, 'turtle3')  # Harmadik teknős
        self.spawn_turtle(0.0, 8.0, 'turtle4')  # Negyedik teknős

    def spawn_turtle(self, x, y, name):
        """Technikai funkció a teknős létrehozására"""
        # Hozzáférés a spawn szolgáltatáshoz
        spawn_client = self.create_client(Spawn, '/spawn')

        # Várakozás a szolgáltatás elérhetőségére
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        # Szolgáltatás kérés előkészítése
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = 0.0  # A teknős szöge, 0.0, tehát nem lesz elforgatva
        request.name = name  # A teknős neve

        # Szolgáltatás meghívása
        spawn_client.call_async(request)

def main(args=None):
    rclpy.init(args=args)  # ROS2 inicializálás
    multiple_turtles_node = MultipleTurtlesNode()  # Csomag node-ja
    rclpy.spin(multiple_turtles_node)  # Node futtatása
    multiple_turtles_node.destroy_node()  # Node leállítása
    rclpy.shutdown()  # ROS2 leállítása

if __name__ == '__main__':
    main()
