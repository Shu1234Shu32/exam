#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from my_robot_interfaces.srv import NextGoal

class HBTask1BController(Node):

    def __init__(self):
        super().__init__('hb_task1b_controller')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)

        self.vel = Twist()
        self.rate = self.create_rate(100)
        self.flag = 0

        self.cli = self.create_client(NextGoal, 'next_goal')
        self.req = NextGoal.Request()
        self.index = 0

    def odometry_callback(self, msg):
        if self.flag == 1:
            self.flag = 0
            self.req.request_goal += 1
            self.send_request(self.req.request_goal)

    def send_request(self, index):
        self.req.request_goal = self.index
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    ebot_controller = HBTask1BController()
    ebot_controller.send_request(ebot_controller.index)

    while rclpy.ok():
        if ebot_controller.future.done():
            try:
                response = ebot_controller.future.result()
            except Exception as e:
                ebot_controller.get_logger().info('Service call failed %r' % (e,))
            else:
                x_goal = response.x_goal
                y_goal = response.y_goal
                theta_goal = response.theta_goal
                ebot_controller.flag = response.end_of_list

                error_x_global = x_goal - ebot_controller.hb_x
                error_y_global = y_goal - ebot_controller.hb_y
                error_theta_global = theta_goal - ebot_controller.hb_theta

                error_x_body = error_x_global * math.cos(ebot_controller.hb_theta) + error_y_global * math.sin(ebot_controller.hb_theta)
                error_y_body = -error_x_global * math.sin(ebot_controller.hb_theta) + error_y_global * math.cos(ebot_controller.hb_theta)
                error_theta_body = error_theta_global

                v_x = ebot_controller.Kp_linear * error_x_body
                v_y = ebot_controller.Kp_linear * error_y_body
                w = ebot_controller.Kp_angular * error_theta_body

                ebot_controller.vel.linear.x = v_x
                ebot_controller.vel.linear.y = v_y
                ebot_controller.vel.angular.z = w

                ebot_controller.cmd_vel_pub.publish(ebot_controller.vel)

                if abs(error_x_global) < 0.1 and abs(error_y_global) < 0.1 and abs(error_theta_global) < 0.1:
                    ebot_controller.stable_time += 0.01
                    if ebot_controller.stable_time >= 1.0:
                        ebot_controller.stable_time = 0.0
                        ebot_controller.index += 1
                        if ebot_controller.index >= len(ebot_controller.x_goals):
                            ebot_controller.index = 0
                        ebot_controller.req.x_goal = ebot_controller.x_goals[ebot_controller.index]
                        ebot_controller.req.y_goal = ebot_controller.y_goals[ebot_controller.index]
                        ebot_controller.req.theta_goal = ebot_controller.theta_goals[ebot_controller.index]
                        ebot_controller.future = ebot_controller.cli.call_async(ebot_controller.req)

        rclpy.spin_once(ebot_controller)

    ebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

