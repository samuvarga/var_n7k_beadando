import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty
import math

class SierpinskiTriangleNode(Node):

    def __init__(self):
        super().__init__('sierpinski_triangle_node')

        # Szolgáltatás indítása
        self.client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('A szolgáltatás nem elérhető, próbálkozom újra...')
        
        self.get_logger().info('Szolgáltatás elérhető, indítom a háromszög rajzolását...')
        
        # A háromszög rajzolása
        self.draw_sierpinski_triangle()

    def draw_sierpinski_triangle(self):
        # 3 csúcs koordinátái
        points = [(2, 2), (8, 8), (2, 8)]

        # Először mozogjunk a kezdőpontba
        self.teleport_to(2, 2)

        # Rajzolás
        self.get_logger().info("Rajzolom a háromszöget...")

        for _ in range(3):  # Három oldal
            self.move_forward(6.0)
            self.turn_left(120)

    def teleport_to(self, x, y):
        # A turtle pozicionálása
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        self.client.call_async(request)

    def move_forward(self, distance):
        # Elmozdulás előre
        self.get_logger().info(f'Mozgás {distance} egységet előre')

    def turn_left(self, angle):
        # Forgatás balra
        self.get_logger().info(f'Forgatás {angle} fokkal balra')

def main(args=None):
    rclpy.init(args=args)
    node = SierpinskiTriangleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
