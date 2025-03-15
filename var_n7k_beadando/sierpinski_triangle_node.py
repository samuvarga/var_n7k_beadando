import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

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

        # Hozzáadjuk az 8-szög rajzolást a turtle3, turtle4, turtle5 számára
        self.draw_octagon('turtle3')
        self.draw_octagon('turtle4')
        self.draw_octagon('turtle5')

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

    def draw_octagon(self, turtle_name):
        """Rajzolunk egy 8-szöget a megfelelő teknőssel"""
        turtle_cmd_pub = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)

        # Először beállítjuk a turtle pozícióját
        self.set_position(turtle_name, 5.0, 5.0)  # Kezdő pozíció

        # Most indítjuk el az 8-szöget
        for _ in range(8):  # 8 oldal
            self.move_forward(turtle_cmd_pub, 2.0)  # Mozgás előre 2.0 m
            self.turn(turtle_cmd_pub, 45)  # 45 fokos elforgatás

    def set_position(self, turtle_name, x, y):
        """Beállítjuk a teknős pozícióját a megadott koordinátákra"""
        # Használhatunk egy szolgáltatást vagy más megoldást a pozíció módosításához,
        # itt a mozgás függvényeket fogjuk alkalmazni a megadott helyre.
        pass

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
