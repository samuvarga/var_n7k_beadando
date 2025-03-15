import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SierpinskiTriangleNode(Node):

    def __init__(self):
        super().__init__('sierpinski_triangle_node')

        # Publisher a cmd_vel topichoz, hogy mozgassuk a teknőst
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Elindítjuk a timer-t, ami 0.5 másodpercenként hívja a move_turtle metódust
        self.timer = self.create_timer(0.5, self.move_turtle)

        # Különböző paraméterek a háromszög rajzolásához
        self.side_length = 2.0  # A háromszög oldalának hossza
        self.angle = 120.0      # A háromszög belső szögének értéke

        # Inicializáljuk a számlálót, hogy nyomon követhessük, hány lépést tettünk
        self.step_count = 0

        self.get_logger().info("Sierpinski Triangle Node elindult")

    def move_turtle(self):
        # Üzenet a teknős mozgásához
        cmd_msg = Twist()

        # Ha még nem rajzoltunk háromszöget, akkor kezdjük el
        if self.step_count < 3:
            cmd_msg.linear.x = self.side_length  # Haladás előre
            cmd_msg.angular.z = 0.0  # Nincs elforgatás

        # Ha három oldal el van rajzolva, akkor forgatjuk a teknőst 120 fokkal
        else:
            cmd_msg.linear.x = 0.0  # Ne mozduljon előre
            cmd_msg.angular.z = 1.57  # 120 fokos forgatás (rad)

        self.cmd_pub.publish(cmd_msg)

        # Frissítjük a lépéseinket
        self.step_count += 1

        # Ha elértük a három oldal rajzolását, akkor újra kezdjük
        if self.step_count > 5:
            self.step_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = SierpinskiTriangleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
