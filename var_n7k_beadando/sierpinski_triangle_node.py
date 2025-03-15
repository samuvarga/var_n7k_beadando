import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
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

        # Hozzáadjuk a körök rajzolását
        self.draw_circle('turtle2', 1.0)  # Kisebb kör
        self.draw_circle('turtle3', 1.5)  # Közepes kör
        self.draw_circle('turtle4', 2.0)  # Nagy kör
        self.draw_circle('turtle5', 2.5)  # Még nagyobb kör

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

    def draw_circle(self, turtle_name, radius):
        """Rajzolunk egy kört a megfelelő teknőssel"""
        # A kör rajzolását timer segítségével indítjuk
        self.get_logger().info(f'{turtle_name} elindította a kört rajzolását!')
        self.create_timer(0.5, lambda: self.draw_circle_move(turtle_name, radius))

    def draw_circle_move(self, turtle_name, radius):
        """Rajzolunk egy kört egy adott turtle-lal"""
        turtle_cmd_pub = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)

        # Kör rajzolás
        move_cmd = Twist()
        move_cmd.linear.x = 0.0  # Nincs előre mozgás
        move_cmd.angular.z = 2 * math.pi / radius  # Kör mozgás kiszámítása math.pi használatával
        turtle_cmd_pub.publish(move_cmd)

        # Az egyes köröket több iterációval rajzoljuk
        self.get_logger().info(f'{turtle_name} körét rajzolja most')

    def move_forward(self, cmd_pub, distance):
        """A teknős előre mozgatása a megadott távolságra"""
        move_cmd = Twist()
        move_cmd.linear.x = distance  # Előre mozgatás
        cmd_pub.publish(move_cmd)

    def turn(self, cmd_pub, angle):
        """A teknős elforgatása a megadott szögben"""
        turn_cmd = Twist()
        turn_cmd.angular.z = float(angle)  # Elforgatás float típusra konvertálva
        cmd_pub.publish(turn_cmd)

def main(args=None):
    rclpy.init(args=args)
    multiple_turtles_node = MultipleTurtlesNode()
    rclpy.spin(multiple_turtles_node)
    multiple_turtles_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
