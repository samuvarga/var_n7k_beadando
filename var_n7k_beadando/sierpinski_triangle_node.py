import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from geometry_msgs.msg import Twist
import math  # A math.pi használatához

class MultipleTurtlesNode(Node):
    def __init__(self):
        super().__init__('multiple_turtles_node')

        # Töröljük a már meglévő turtle1-et
        self.kill_turtle('turtle1')

        # 4 teknős létrehozása
        self.spawn_turtle(5.0, 5.0, 'turtle2')  # Második teknős
        self.spawn_turtle(5.0, 6.5, 'turtle3')  # Harmadik teknős
        self.spawn_turtle(5.0, 8.0, 'turtle4')  # Negyedik teknős
        self.spawn_turtle(5.0, 9.5, 'turtle5')  # Ötödik teknős

        # Megvárjuk, amíg mindegyik teknős spawnolódik, és csak utána kezdünk el köröket rajzolni
        self.get_logger().info("Várakozás a teknősök spawnolására...")
        self.create_timer(1.0, self.draw_circles)  # 1 másodperc késleltetés, hogy biztosan elinduljanak

    def spawn_turtle(self, x, y, name):
        """Technikai funkció a teknős létrehozására"""
        spawn_client = self.create_client(Spawn, '/spawn')

        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = 0.0  # Nem forgatjuk el a teknőst
        request.name = name

        spawn_client.call_async(request)

    def kill_turtle(self, name):
        """Technikai funkció egy teknős törlésére"""
        kill_client = self.create_client(Kill, '/kill')

        while not kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for kill service...')

        request = Kill.Request()
        request.name = name

        kill_client.call_async(request)

    def draw_circles(self):
        """Rajzolunk köröket, ha minden teknős spawnolódott"""
        self.get_logger().info('Most kezdjük el a körök rajzolását!')

        # turtle2 nem mozog, de a többiek a turtle2 körül húznak köröket
        self.draw_circle('turtle3', 1.5)  # Kisebb kör
        self.draw_circle('turtle4', 3.0)  # Közepes kör
        self.draw_circle('turtle5', 4.5)  # Nagy kör

    def draw_circle(self, turtle_name, radius):
        """Rajzolunk egy kört a megfelelő teknőssel"""
        # Kör rajzolása közvetlenül, timer nélkül
        self.get_logger().info(f'{turtle_name} elindította a kört rajzolását!')
        self.move_in_circle(turtle_name, radius)

    def move_in_circle(self, turtle_name, radius):
        """A kör rajzolása a teknőssel"""
        turtle_cmd_pub = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)

        # A kör rajzolása úgy, hogy a középpont turtle2 legyen
        move_cmd = Twist()
        move_cmd.linear.x = 1.0  # A teknős előre mozog
        move_cmd.angular.z = 2 * math.pi / radius  # Kör mozgás kiszámítása math.pi használatával
        turtle_cmd_pub.publish(move_cmd)

        # A teknős megállítása egy teljes kör megtétele után
        self.create_timer(2 * math.pi / radius, self.stop_turtle, turtle_name)  # 1 teljes kör megtételéhez szükséges idő

    def stop_turtle(self, turtle_name):
        """Leállítja a teknőst miután egy teljes kört tett meg"""
        turtle_cmd_pub = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)

        # Leállítjuk a teknőst
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        turtle_cmd_pub.publish(stop_cmd)

        self.get_logger().info(f'{turtle_name} megállt, miután egy teljes kört tett meg.')

def main(args=None):
    rclpy.init(args=args)
    multiple_turtles_node = MultipleTurtlesNode()
    rclpy.spin(multiple_turtles_node)
    multiple_turtles_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
