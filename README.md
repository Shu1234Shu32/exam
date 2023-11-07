#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
from tf.transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal
from rclpy.qos import qos_profile_sensor_data

class HBTask1BController(Node):
    def __init__(self):
        super().__init__('hb_task1b_controller')

        # Initialize Publisher and Subscriber
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odometry_callback, qos_profile=qos_profile_sensor_data)

        # Initialize a Twist message
        self.vel = Twist()
        # Initialize the required variables to 0
        self.hb_x = 0
        self.hb_y = 0
        self.hb_theta = 0

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

        # Define desired goal poses
        self.x_goals = [4, -4, -4, 4, 0]
        self.y_goals = [4, 4, -4, -4, 0]
        self.theta_goals = [0, 0, 0, 0, 0]

        # Initialize index to track the current goal pose
        self.index = 0

    def odometry_callback(self, msg):
        # Extract the robot's pose information from the /odom topic
        pose = msg.pose.pose
        orientation = pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.hb_x = pose.position.x
        self.hb_y = pose.position.y
        self.hb_theta = yaw

    def main(self):
        # Control loop
        while rclpy.ok():
            # Calculate error in global frame
            x_error = self.x_goals[self.index] - self.hb_x
            y_error = self.y_goals[self.index] - self.hb_y
            theta_error = self.theta_goals[self.index] - self.hb_theta

            # Calculate error in body frame
            cos_theta = math.cos(self.hb_theta)
            sin_theta = math.sin(self.hb_theta)
            x_error_body = cos_theta * x_error + sin_theta * y_error
            y_error_body = -sin_theta * x_error + cos_theta * y_error
            theta_error_body = theta_error

            # P controller gains (you may need to tune these)
            Kp_x = 1.0
            Kp_y = 1.0
            Kp_theta = 1.0

            # Calculate desired linear and angular velocities
            v_x = Kp_x * x_error_body
            v_y = Kp_y * y_error_body
            w = Kp_theta * theta_error_body

            # Safety checks for velocity limits (you may need to adjust these)
            max_linear_velocity = 1.0
            max_angular_velocity = 1.0
            v_x = min(max(v_x, -max_linear_velocity), max_linear_velocity)
            v_y = min(max(v_y, -max_linear_velocity), max_linear_velocity)
            w = min(max(w, -max_angular_velocity), max_angular_velocity)

            # Publish the desired velocities
            self.vel.linear.x = v_x
            self.vel.linear.y = v_y
            self.vel.angular.z = w
            self.publisher_.publish(self.vel)

            # Check if the goal has been reached
            if abs(x_error) < 0.1 and abs(y_error) < 0.1 and abs(theta_error) < 0.1:
                # Stabilize at the goal pose for 1 second
                time.sleep(1)
                # Increment the index if it's less than the length of the goal pose lists
                if self.index < len(self.x_goals) - 1:
                    self.index += 1
                else:
                    # If all goal poses have been reached, you can break out of the loop
                    break

            self.rate.sleep()

        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    ebot_controller = HBTask1BController()
    ebot_controller.main()

if __name__ == '__main__':
    main()
