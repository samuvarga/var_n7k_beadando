import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill, SetPen
from geometry_msgs.msg import Twist
import math

class MultipleTurtlesNode(Node):
    def __init__(self):
        super().__init__('multiple_turtles_node')

        # Töröljük a már meglévő turtle1-et
        self.kill_turtle('turtle1')

        # 9 teknős létrehozása különböző pontokban
        self.spawn_turtle(5.0, 6.5, 'turtle3')  # Harmadik teknős
        self.spawn_turtle(5.0, 8.0, 'turtle4')  # Negyedik teknős
        self.spawn_turtle(5.0, 9.5, 'turtle5')  # Ötödik teknős
        self.spawn_turtle(5.0, 5.5, 'turtle6')  # Hatodik teknős
        self.spawn_turtle(5.0, 6.0, 'turtle7')  # Hetedik teknős
        self.spawn_turtle(5.0, 7.0, 'turtle8')  # Nyolcadik teknős
        self.spawn_turtle(5.0, 7.5, 'turtle9')  # Kilencedik teknős
        self.spawn_turtle(5.0, 8.5, 'turtle10')  # Tizedik teknős
        #self.spawn_turtle(5.0, 9.0, 'turtle11')  # Tizenegyedik teknős

        # Szín és egyéb tulajdonságok beállítása
        self.set_turtle_appearance('turtle3', 0, 0, 0)  # Piros
        self.set_turtle_appearance('turtle4', 255, 0, 0)  # Piros
        self.set_turtle_appearance('turtle5', 255, 0, 0)  # Piros
        self.set_turtle_appearance('turtle6', 255, 255, 0)  # Citromsárga
        self.set_turtle_appearance('turtle7', 255, 255, 0)  # Citromsárga
        self.set_turtle_appearance('turtle8', 255, 0, 0)  # Piros
        self.set_turtle_appearance('turtle9', 255, 0, 0)  # Piros
        self.set_turtle_appearance('turtle10', 255, 0, 0)  # Piros
        #self.set_turtle_appearance('turtle11', 255, 0, 0)  # Piros

        # Megvárjuk, amíg mindegyik teknős spawnolódik, és csak utána kezdünk el köröket rajzolni
        self.get_logger().info("Várakozás a teknősök spawnolására...")

        # A körök rajzolása külön timer-ben
        self.create_timer(2.0, self.draw_circles)  # 2 másodperc késleltetés, hogy ne blokkolja az egyéb mozgásokat

        # Időzítő létrehozása, hogy 32 másodperc után törölje az összes teknőst
        self.create_timer(34.0, self.delete_all_turtles)  # 34 másodperc késleltetés (2 másodperc várakozás + 32 másodperc rajzolás)

    def spawn_turtle(self, x, y, name):
        """Technikai funkció a teknős létrehozására"""
        spawn_client = self.create_client(Spawn, '/spawn')

        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = 0.0  # Nem forgatjuk el a teknőst, tehát jobbra néz
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

    def set_turtle_appearance(self, turtle_name, r, g, b):
        """Beállítja a teknős megjelenését (szín, vastagság, stb.)"""
        set_pen_client = self.create_client(SetPen, f'/{turtle_name}/set_pen')

        while not set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {turtle_name} set_pen service...')

        request = SetPen.Request()
        request.r = r  # Piros szín
        request.g = g  # Zöld szín
        request.b = b  # Kék szín
        request.width = 2  # Vonalvastagság
        request.off = False  # Rajzolás engedélyezése

        set_pen_client.call_async(request)

    def draw_circles(self):
        """Rajzolunk köröket a többi teknőssel"""
        self.get_logger().info('Most kezdjük el a körök rajzolását!')

        # turtle3, turtle4 és turtle5 köröket rajzolnak
        self.move_in_circle('turtle3', 1.5)
        self.move_in_circle('turtle4', 3.0)
        self.move_in_circle('turtle5', 4.5)
        self.move_in_circle('turtle6', 0.5)
        self.move_in_circle('turtle7', 1.0)
        self.move_in_circle('turtle8', 2.0)
        self.move_in_circle('turtle9', 2.5)
        self.move_in_circle('turtle10', 3.5)
        #self.move_in_circle('turtle11', 4.0)

    def move_in_circle(self, turtle_name, radius):
        """A kör rajzolása a teknőssel"""
        turtle_cmd_pub = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)

        # Az angular sebességet úgy kell beállítani, hogy a teknős jobbra forogjon
        move_cmd = Twist()
        move_cmd.linear.x = 1.0  # Állandó sebesség, hogy körben mozogjanak
        move_cmd.angular.z = -1.0 * (1.0 / radius)  # A negatív angular sebesség biztosítja a jobbra forgást

        def publish_cmd():
            turtle_cmd_pub.publish(move_cmd)

        # Időzítő létrehozása, hogy folyamatosan küldjön parancsokat
        self.create_timer(0.1, publish_cmd)

    def delete_all_turtles(self):
        """Törli az összes teknőst"""
        self.get_logger().info('Töröljük az összes teknőst!')
        for i in range(3, 12):
            self.kill_turtle(f'turtle{i}')

def main(args=None):
    rclpy.init(args=args)
    multiple_turtles_node = MultipleTurtlesNode()
    rclpy.spin(multiple_turtles_node)
    multiple_turtles_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()