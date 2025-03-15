import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill

class MultipleTurtlesNode(Node):
    def __init__(self):
        super().__init__('multiple_turtles_node')

        # Töröljük a már meglévő turtle1-et, amit a turtlesim spawnolt
        self.kill_turtle('turtle1')  

        # 4 teknős létrehozása
        self.spawn_turtle(5.0, 5.0, 'turtle2')  # Második teknős
        self.spawn_turtle(5.0, 6.5, 'turtle3')  # Harmadik teknős
        self.spawn_turtle(5.0, 8.0, 'turtle4')  # Negyedik teknős
        self.spawn_turtle(5.0, 9.5, 'turtle5')  # Ötödik teknős

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

    def kill_turtle(self, name):
        """Technikai funkció egy teknős törlésére"""
        kill_client = self.create_client(Kill, '/kill')

        # Várakozás a szolgáltatás elérhetőségére
        while not kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for kill service...')

        # Szolgáltatás kérés előkészítése
        request = Kill.Request()
        request.name = name

        # Szolgáltatás meghívása
        kill_client.call_async(request)

def main(args=None):
    rclpy.init(args=args)  # ROS2 inicializálás
    multiple_turtles_node = MultipleTurtlesNode()  # Csomag node-ja
    rclpy.spin(multiple_turtles_node)  # Node futtatása
    multiple_turtles_node.destroy_node()  # Node leállítása
    rclpy.shutdown()  # ROS2 leállítása

if __name__ == '__main__':
    main()
