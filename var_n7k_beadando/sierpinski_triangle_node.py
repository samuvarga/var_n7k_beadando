import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SpiderWebNode(Node):

    def __init__(self):
        super().__init__('spider_web_node')
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.2, self.draw_spider_web)
        self.angle = 0
        self.radius = 50
        self.loop_count = 0

    def draw_spider_web(self):
        # Létrehozzuk az üzenetet
        cmd_msg = Twist()

        # Rajzoljuk a sugárvonalakat
        if self.loop_count < 12:
            cmd_msg.linear.x = 1.0  # Mozgás előre
            cmd_msg.angular.z = math.radians(30)  # 30 fokos elforgatás
        else:
            # Rajzoljuk a koncentrikus köröket
            cmd_msg.linear.x = 0.0  # Ne mozduljon előre
            cmd_msg.angular.z = 0.0  # Nincs elforgatás
            self.draw_circle(self.radius)  # Kör rajzolása

        # Közzétesszük az üzenetet
        self.cmd_pub.publish(cmd_msg)

        self.loop_count += 1
        if self.loop_count > 24:  # 12 vonal és 2 kör
            self.loop_count = 0
            self.radius += 50  # Következő körnél nagyobb sugár

    def draw_circle(self, radius):
        # Kör rajzolása
        for _ in range(36):  # Kicsi lépések a kör rajzolásához
            cmd_msg = Twist()
            cmd_msg.linear.x = 1.0
            cmd_msg.angular.z = math.radians(10)  # Kis lépések a körhöz
            self.cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    spider_web_node = SpiderWebNode()
    rclpy.spin(spider_web_node)
    spider_web_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
