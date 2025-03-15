import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class SpiderWebNode(Node):

    def __init__(self):
        super().__init__('spider_web_node')
        self.timer = self.create_timer(0.2, self.loop)  # 0.2 másodpercenként
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.loop_count = 0
        self.radius = 20  # Kezdeti sugár a körökhöz (kisebb)
        self.get_logger().info("Spider Web Node has been started")

    def loop(self):
        cmd_msg = Twist()

        if self.loop_count < 8:  # Kevesebb sugarat rajzolunk
            cmd_msg.linear.x = 1.0  # Előre mozgás
            cmd_msg.angular.z = math.radians(45)  # 45 fokos elforgatás
        elif self.loop_count < 16:  # 8 után rajzolunk egy kört
            cmd_msg.linear.x = 0.0  # Ne mozduljon előre
            cmd_msg.angular.z = 0.0  # Ne forgasson
            self.draw_circle(self.radius)  # Kör rajzolása
            self.radius += 20  # Növeljük a sugár értékét a következő körhöz
        else:
            cmd_msg.linear.x = 0.0  # Ne mozduljon előre
            cmd_msg.angular.z = 0.0  # Ne forgasson
            self.loop_count = 0  # Ciklus visszaállítása

        self.cmd_pub.publish(cmd_msg)
        self.loop_count += 1

    def draw_circle(self, radius):
        # Kör rajzolása kisebb lépésekben
        for _ in range(18):  # 180 fokot rajzolunk 10 fokos lépésekben (kisebb kör)
            cmd_msg = Twist()
            cmd_msg.linear.x = 1.0  # Előre mozgás
            cmd_msg.angular.z = math.radians(10)  # 10 fokos elforgatás
            self.cmd_pub.publish(cmd_msg)
            time.sleep(0.1)  # Kis szünet, hogy megjelenjen a mozgás

def main(args=None):
    rclpy.init(args=args)
    spider_web_node = SpiderWebNode()
    rclpy.spin(spider_web_node)
    spider_web_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
