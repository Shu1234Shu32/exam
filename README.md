#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
from tf.transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal

class HBTask1BController(Node):

    def __init__(self):
        super().__init__('hb_task1b_controller')
        
        # Initialize Publisher and Subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)

        # Declare a Twist message
        self.vel = Twist()

        # Initialize the required variables
        self.Kp_linear = 1.0
        self.Kp_angular = 1.0

        # Initialize variables for desired goal poses
        self.x_goals = [4, -4, -4, 4, 0]
        self.y_goals = [4, 4, -4, -4, 0]
        self.theta_goals = [0, 0, 0, 0, 0]

        # Index for tracking the current goal
        self.index = 0

        # Variables to store robot's position and orientation
        self.hb_x = 0.0
        self.hb_y = 0.0
        self.hb_theta = 0.0
        self.stable_time = 0.0
        self.flag = 0

        # Client for the "next_goal" service
        self.cli = self.create_client(NextGoal, 'next_goal')
        self.req = NextGoal.Request()

    def odometry_callback(self, msg):
        self.hb_x = msg.pose.pose.position.x
        self.hb_y = msg.pose.pose.position.y
        self.hb_theta = self.get_yaw_from_odometry(msg)

        if self.flag == 1:
            self.flag = 0
            self.index += 1
            if self.index >= len(self.x_goals):
                self.index = 0
            self.req.x_goal = self.x_goals[self.index]
            self.req.y_goal = self.y_goals[self.index]
            self.req.theta_goal = self.theta_goals[self.index]
            self.future = self.cli.call_async(self.req)

        error_x_global = self.x_goals[self.index] - self.hb_x
        error_y_global = self.y_goals[self.index] - self.hb_y
        error_theta_global = self.theta_goals[self.index] - self.hb_theta

        error_x_body = error_x_global * math.cos(self.hb_theta) + error_y_global * math.sin(self.hb_theta)
        error_y_body = -error_x_global * math.sin(self.hb_theta) + error_y_global * math.cos(self.hb_theta)
        error_theta_body = error_theta_global

        v_x = self.Kp_linear * error_x_body
        v_y = self.Kp_linear * error_y_body
        w = self.Kp_angular * error_theta_body

        self.vel.linear.x = v_x
        self.vel.linear.y = v_y
        self.vel.angular.z = w

        self.cmd_vel_pub.publish(self.vel)

        if abs(error_x_global) < 0.1 and abs(error_y_global) < 0.1 and abs(error_theta_global) < 0.1:
            self.stable_time += 0.01
            if self.stable_time >= 1.0:
                self.stable_time = 0.0
                self.index += 1
                if self.index >= len(self.x_goals):
                    self.index = 0
                self.req.x_goal = self.x_goals[self.index]
                self.req.y_goal = self.y_goals[self.index]
                self.req.theta_goal = self.theta_goals[self.index]
                self.future = self.cli.call_async(self.req)

    def get_yaw_from_odometry(self, odometry_msg):
        orientation = odometry_msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    
    # Create an instance of the HBTask1BController class
    ebot_controller = HBTask1BController()

    # Main loop
    while rclpy.ok():
        rclpy.spin_once(ebot_controller)
    
    # Destroy the node and shut down ROS
    ebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

