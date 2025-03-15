import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from geometry_msgs.msg import Twist
import math
import time

class SierpinskiTriangleNode(Node):

    def __init__(self):
        super().__init__('sierpinski_triangle_node')
        
        # Készítünk egy publisher-t a turtle mozgásának vezérlésére
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Készítünk egy szolgáltatást a turtle teleportálásához
        self.client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('A szolgáltatás nem elérhető, próbálkozom újra...')

        # A háromszög rajzolása
        self.draw_sierpinski_triangle()

    def draw_sierpinski_triangle(self):
        # Háromszög csúcsai
        points = [(2, 2), (8, 8), (2, 8)]

        # A turtle teleportálása az első csúcsba
        self.teleport_to(2, 2)

        # Rajzolás
        for i in range(3):
            self.move_forward(6.0)
            self.turn_left(120)

    def teleport_to(self, x, y):
        # A turtle teleportálása
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        self.client.call_async(request)

    def move_forward(self, distance):
        # A turtle előre mozgása
        cmd_msg = Twist()
        cmd_msg.linear.x = distance
        self.cmd_pub.publish(cmd_msg)
        self.get_logger().info(f'Mozgás {distance} egységet előre')
        time.sleep(2)  # Várjunk 2 másodpercet, hogy elérje a célt

    def turn_left(self, angle):
        # A turtle balra fordulásának vezérlése
        cmd_msg = Twist()
        cmd_msg.angular.z = math.radians(angle)  # Átalakítjuk a fokot radiánná
        self.cmd_pub.publish(cmd_msg)
        self.get_logger().info(f'Forgatás {angle} fokkal balra')
        time.sleep(1)  # Várjunk 1 másodpercet a forgatás után

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
