import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
import math

class CircleDrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer')
        self.turtle1 = None
        self.turtle2 = None
        self.circle1_radius = 2.0  # Radius of the first circle
        self.circle2_radius = 4.0  # Radius of the second circle

        self.create_turtle('turtle1', 3.0, 3.0, 'circle_turtle1')
        self.create_turtle('turtle2', 7.0, 7.0, 'circle_turtle2')
        self.draw_circle(self.turtle1, self.circle1_radius)
        self.draw_circle(self.turtle2, self.circle2_radius)

    def create_turtle(self, name, x, y, turtle_name):
        spawn_service = self.create_client(Spawn, 'spawn')
        while not spawn_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')

        req = Spawn.Request()
        req.name = turtle_name
        req.x = x
        req.y = y
        req.theta = 0.0
        spawn_service.call_async(req)
        self.get_logger().info(f'Spawning {name} at ({x}, {y})')

    def draw_circle(self, turtle, radius):
        cmd_vel = Twist()
        cmd_vel.linear.x = 2.0  # Change the linear velocity as needed
        cmd_vel.angular.z = 2.0 * math.pi / radius  # Angular velocity to make a circle
        rate = self.create_rate(1)  # Control loop rate

        while rclpy.ok():
            self.turtle_pub = self.create_publisher(Twist, f'{turtle}/cmd_vel', 10)
            self.turtle_pub.publish(cmd_vel)
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
