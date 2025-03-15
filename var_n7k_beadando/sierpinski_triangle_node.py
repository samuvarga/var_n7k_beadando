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
        self.loop_count = 0
        self.sides = [40, 60, 80, 100]  # 4 különböző méretű 8szög
        self.positions = [(0, 0), (0, 40), (0, 100), (0, 180)]  # teleport pontok
        self.get_logger().info("Spider Web Node with one turtle has been started")

    def loop(self):
        # Rajzolás logika
        if self.loop_count < len(self.sides):
            self.draw_spider_web(self.sides[self.loop_count], self.positions[self.loop_count])
            self.loop_count += 1
        else:
            self.loop_count = 0  # Újraindítja a folyamatot

    def draw_spider_web(self, size, position):
        """Rajzoljon egy nyolcszöget a megadott méretben és pozícióban."""
        # Teleportálás a kezdőpontra
        cmd_msg = Twist()
        cmd_msg.linear.x = 0
        cmd_msg.angular.z = 0
        self.cmd_pub.publish(cmd_msg)
        time.sleep(0.5)  # Várjunk, hogy a teleportálás befejeződjön

        # Beállítjuk a pozíciót
        self.teleport_to(position)

        # Rajzolás: 8szög
        for _ in range(8):  # 8 oldalas nyolcszög
            cmd_msg = Twist()
            cmd_msg.linear.x = 2.0  # Előre mozgás
            cmd_msg.angular.z = math.radians(45)  # 45 fokos elforgatás
            self.cmd_pub.publish(cmd_msg)
            time.sleep(0.1)

        # Középpontból húzunk vonalakat a sarkokba
        for i in range(8):  # 8 vonalat rajzolunk
            cmd_msg = Twist()
            cmd_msg.linear.x = size  # A méret határozza meg a vonal hosszát
            cmd_msg.angular.z = math.radians(45)  # Elforgatás a következő sarokba
            self.cmd_pub.publish(cmd_msg)
            time.sleep(0.1)

    def teleport_to(self, position):
        """A teknőst a megadott pozícióba teleportálja."""
        cmd_msg = Twist()
        # X és Y koordinátákhoz tartozó pozícióba teleportálás (mivel TurtleSim 2D-ben működik)
        cmd_msg.linear.x = position[0]
        cmd_msg.linear.y = position[1]
        cmd_msg.angular.z = 0  # Nincs elforgatás
        self.cmd_pub.publish(cmd_msg)
        time.sleep(0.5)  # Várjunk egy kicsit, hogy a teleportálás befejeződjön

def main(args=None):
    rclpy.init(args=args)
    spider_web_node = SpiderWebNode()
    rclpy.spin(spider_web_node)
    spider_web_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
