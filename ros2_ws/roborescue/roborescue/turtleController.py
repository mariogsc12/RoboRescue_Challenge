#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial
import random

class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.poseCallback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.prev_pose = 0.0
        self.pose = Pose()

        self.get_config_parameters()
        self.get_logger().info("Turtle controller created")

    def get_config_parameters(self):
        self.declare_parameter('turtlesim_screen_max', 11.0) 
        self.declare_parameter('turtlesim_screen_min', 0.0) 
        self.declare_parameter('screen_color_limit', 5.0) 

        self.screen_max = self.get_parameter('turtlesim_screen_max').get_parameter_value().double_value
        self.screen_min = self.get_parameter('turtlesim_screen_min').get_parameter_value().double_value
        self.screen_color_limit = self.get_parameter('screen_color_limit').get_parameter_value().double_value


    def poseCallback(self, pose: Pose):
        cmd_vel = Twist()
        self.pose = pose
        self.pose.x = round(self.pose.x,2)
        self.pose.y = round(self.pose.y,2)

        if (self.prev_pose.x <= self.screen_color_limit and pose.x > self.screen_color_limit) or (self.prev_pose.x >= self.screen_color_limit and pose.x < self.screen_color_limit):
            self.pen_service(random.randint(0,255), random.randint(0,255), random.randint(0,20), 3, 0)
            self.prev_pose.x = pose.x 
            self.get_logger().info(f"The turtle crossed the limit. Changing the color.")
        

    def pen_service(self, r, g, b, width, off):
        client = self.create_client(SetPen, "/turtle1/set_pen")
        while not client.wait_for_service(1.0):
            self.get_logger().info("Waiting for service set_pen...")
        
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)
        future.add_done_callback(partial(self.service_callback))

    def service_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" %(e,))

def main():
    rclpy.init()
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()