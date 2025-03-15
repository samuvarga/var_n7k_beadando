import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class SpiderWebNode(Node):

    def __init__(self):
        super().__init__('spider_web_node')
        self.timer = self.create_timer(0.2, self.loop)  # 0.2 másodpercenkként
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.cmd_pub2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.cmd_pub3 = self.create_publisher(Twist, '/turtle3/cmd_vel', 10)
        self.cmd_pub4 = self.create_publisher(Twist, '/turtle4/cmd_vel', 10)
        self.loop_count = 0
        self.sides = [40, 60, 80, 100]  # 4 különböző méretű 8szög
        self.get_logger().info("Spider Web Node with multiple turtles has been started")

    def loop(self):
        # Rajzolás logika minden teknőshöz
        if self.loop_count == 0:
            # 4 teknős indítása
            self.draw_spider_web(self.sides[0], self.cmd_pub)  # Első teknős
            self.draw_spider_web(self.sides[1], self.cmd_pub2)  # Második teknős
            self.draw_spider_web(self.sides[2], self.cmd_pub3)  # Harmadik teknős
            self.draw_spider_web(self.sides[3], self.cmd_pub4)  # Negyedik teknős
            self.loop_count += 1
        else:
            self.loop_count += 1

    def draw_spider_web(self, size, cmd_pub):
        """Rajzoljon egy nyolcszöget a megadott méretben."""
        for _ in range(8):  # 8 oldalas nyolcszög
            cmd_msg = Twist()
            cmd_msg.linear.x = 2.0  # Előre mozgás
            cmd_msg.angular.z = math.radians(45)  # 45 fokos elforgatás
            cmd_pub.publish(cmd_msg)
            time.sleep(0.1)

        # Középpontból húzunk vonalakat a sarkokba
        for i in range(8):  # 8 vonalat rajzolunk
            cmd_msg = Twist()
            cmd_msg.linear.x = size  # A méret határozza meg a vonal hosszát
            cmd_msg.angular.z = math.radians(45)  # Elforgatás a következő sarokba
            cmd_pub.publish(cmd_msg)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    spider_web_node = SpiderWebNode()
    rclpy.spin(spider_web_node)
    spider_web_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
