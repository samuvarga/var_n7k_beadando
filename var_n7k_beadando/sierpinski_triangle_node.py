import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SierpinskiTriangleNode(Node):

    def __init__(self):
        super().__init__('sierpinski_triangle_node')

        # Publisher a cmd_vel topichoz
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Timer 0.5 másodperces intervallummal
        self.timer = self.create_timer(0.5, self.move_turtle)

        # A háromszög oldalának hossza és az új háromszög kisebb oldalának hossza
        self.side_length = 2.0  
        self.angle = 120.0  # Háromszög belső szöge

        # Előrehaladás számláló
        self.step_count = 0

        # Rekurzív háromszög rajzolás beállítása
        self.depth = 4  # Mennyi szintű háromszöget rajzoljunk (rekurzív lépés)

        self.get_logger().info("Sierpinski Triangle Node elindult")

    def move_turtle(self):
        cmd_msg = Twist()

        # Az első három oldal és forgatás
        if self.step_count < 3:
            cmd_msg.linear.x = self.side_length  # Előrehaladás
            cmd_msg.angular.z = 0.0  # Nincs forgatás

        # Ha három oldal el van rajzolva, akkor forgatás 120 fokkal
        else:
            cmd_msg.linear.x = 0.0  # Ne haladjon előre
            cmd_msg.angular.z = math.radians(self.angle)  # 120 fokos forgatás

        # Közzétesszük a parancsot
        self.cmd_pub.publish(cmd_msg)

        self.step_count += 1

        # Ha elértük a három oldal rajzolását, újra kezdjük
        if self.step_count > 5:
            self.step_count = 0

        # Rekurzív háromszög rajzolása (ha több szintet szeretnénk)
        if self.step_count == 0 and self.depth > 0:
            self.depth -= 1
            self.draw_sierpinski(self.side_length)

    def draw_sierpinski(self, side_length):
        # Ha a háromszög elég kicsi, akkor ne rajzoljunk többet
        if side_length < 0.1:
            return
        
        # Rajzolj egy háromszöget
        for _ in range(3):
            self.move_forward(side_length)
            self.turn_120_degrees()

        # Rajzolj három kisebb háromszöget
        for _ in range(3):
            self.move_forward(side_length / 2)
            self.turn_120_degrees()
            self.draw_sierpinski(side_length / 2)
            self.turn_120_degrees()

    def move_forward(self, length):
        # Előrehaladás
        cmd_msg = Twist()
        cmd_msg.linear.x = length
        self.cmd_pub.publish(cmd_msg)
        self.get_logger().info(f'Moving forward: {length}')

    def turn_120_degrees(self):
        # 120 fokos elforgatás
        cmd_msg = Twist()
        cmd_msg.angular.z = math.radians(120)  # 120 fok radianban
        self.cmd_pub.publish(cmd_msg)
        self.get_logger().info('Turning 120 degrees')

def main(args=None):
    rclpy.init(args=args)
    node = SierpinskiTriangleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
