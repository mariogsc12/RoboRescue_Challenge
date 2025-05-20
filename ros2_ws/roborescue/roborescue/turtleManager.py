#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, TeleportAbsolute, TeleportRelative, Spawn
from functools import partial
from roborescue.action import GoTo
from rclpy.executors import ExternalShutdownException

import random
import math
import sys

class DrawLetters:
    def __init__(self, length, width):
        self.length = length
        self.width = width

    def letter_manager(self, letter_, origin_x, origin_y):
        letter = letter_.upper()
        if letter == "R":
            return self.draw_R(origin_x, origin_y)
        elif letter == "O":
            return self.draw_O(origin_x, origin_y)
        elif letter == "B":
            return self.draw_B(origin_x, origin_y)
        else:
            self.get_logger().error(f"The letter {letter} is not configured.")
            return

    def draw_R(self, origin_x=5.54, origin_y=5.54):
        """ Returns a list of points (x,y) to draw the letter R"""
        l = self.length
        w = self.width
        x0, y0 = origin_x, origin_y

        points = [
            (x0, y0),               # start point
            (x0 - w/2, y0),
            (x0 - (2*w)/3, y0 + l / 2), # diagonal to halfway up the letter height
            (x0 - (2*w)/3, y0),         # vertical line down to the point at y=0 and x=width/3
            (x0 - w, y0),
            (x0 - w, y0 + l),
            (x0, y0 + l),           
            (x0, y0 + l / 2),
            (x0 - w/2, y0 + l / 2),
            (x0, y0)

            # //(x0, y0 + l),           # vertical line upward by letter height
            # (x0 + w, y0 + l),       # horizontal line right by letter width
            # (x0 + w, y0 + l / 2),   # downward line by half the letter height
            # (x0 + w/2, y0 + l / 2), # horizontal line left to half the letter width
            # (x0 + w, y0),           # diagonal to the point at y=0 and x=width
            # (x0 + w / 2, y0),       # horizontal line to half the width
            # (x0 + w/3, y0 + l / 2), # diagonal to halfway up the letter height
            # (x0 + w/3, y0),         # vertical line down to the point at y=0 and x=width/3
            # (x0, y0)                # horizontal line left back to the start point
        ]

        return points

    def draw_O(self, origin_x=5.54, origin_y=5.54):
        """ Returns a list of points (x,y) to draw the letter O"""
        l = self.length
        w = self.width
        x0, y0 = origin_x, origin_y

        points = [
            (x0, y0),               # start point
            (x0-w, y0),             # horizontal line left by letter width
            (x0-w, y0 + l),         # vertical line upward by letter height
            (x0, y0 + l),           # horizontal line right by letter width
            (x0, y0),               # vertical line downward by letter height
        ]

        return points

    def draw_B(self, origin_x=5.54, origin_y=5.54):
        """ Returns a list of points (x,y) to draw the letter B"""
        l = self.length
        w = self.width
        x0, y0 = origin_x, origin_y

        points = [
            (x0, y0),               # start point
            (x0-w, y0),             # horizontal line left by letter width
            (x0-w, y0 + l),         # vertical line upward by letter height
            (x0, y0 + l),           # horizontal line right by letter width
            (x0-w/2, y0 + l / 2),   # diagonal line downward by half the letter height
            (x0, y0),               # diagonal line right back to the start point
            # (x0, y0 + l),           # vertical line upward by letter height
            # (x0 + w, y0 + l),       # horizontal line right by letter width
            # (x0 + w/2, y0 + l/2),   # diagonal to the center point
            # (x0 + w, y0),           # diagonal to the right down point
            # (x0, y0)                # horizontal line left back to the start point
        ]

        return points

class TurtleManager(Node):
    def __init__(self):
        super().__init__("turtle_manager")

        self._action_client = ActionClient(self, GoTo, 'GoTo')

        self.get_config_parameters()

        self.letter_info = DrawLetters(self.letter_height, self.letter_width)
        self.get_logger().info("Turtle manager created.")

        self.trajectory_planner()

    def get_config_parameters(self):
        self.declare_parameter('word', "RO") 
        self.declare_parameter('screen_color_limit', 5.0) 
        self.declare_parameter('letter_width', 1.0) 
        self.declare_parameter('letter_height', 3.0) 
        self.declare_parameter('screen_color', [0,0,255]) 
        self.declare_parameter('line_default_color', [255,255,255]) 
        self.declare_parameter('line_width', 3) 
        self.declare_parameter('letter_offset', [0.25,0.0]) 

        self.word = self.get_parameter('word').get_parameter_value().string_value
        self.screen_color_limit = self.get_parameter('screen_color_limit').get_parameter_value().double_value
        self.letter_width = self.get_parameter('letter_width').get_parameter_value().double_value
        self.letter_height = self.get_parameter('letter_height').get_parameter_value().double_value
        self.screen_color = self.get_parameter('screen_color').get_parameter_value().integer_array_value
        self.line_default_color = self.get_parameter('line_default_color').get_parameter_value().integer_array_value
        self.line_width = self.get_parameter('line_width').get_parameter_value().integer_value
        letter_offset = self.get_parameter('letter_offset').get_parameter_value().double_array_value

        self.letter_offset_x = letter_offset[0]
        self.letter_offset_y = letter_offset[1]

    
    def trajectory_planner(self):
        """ Wrapper function to manage the turtle trajectory"""
        
        initial_x = 2.0
        initial_y = 5.0

        for letter in self.word:
            self.get_logger().info(f'--- Starting to draw {letter} at {initial_x,initial_y} ---')
            self.teleport_absoulte_service(initial_x,initial_y,0.0)
            waypoints = self.letter_info.letter_manager(letter,initial_x,initial_y)

            # Loop to move the turtle to the waypoints of the letter (skip the first one to keep the "hide" line)
            for wp in waypoints[1:]:
                goal_future = self.send_goal(wp[0], wp[1], 0.0)
                rclpy.spin_until_future_complete(self, goal_future)

            initial_x = initial_x + self.letter_width + self.letter_offset_x
            initial_y = initial_y + self.letter_offset_y

    # -------- ACTIONS -------
    def send_goal(self, x, y, theta):
        # Wait server
        self._action_client.wait_for_server()

        self.get_logger().info(f"Sending goal: {x, y, theta}")
        goal_msg = GoTo.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.theta = theta  

        # Send goal
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Wait for the goal to be accepted and get result future
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return None

        self.get_logger().info('Goal accepted.')

        # Wait for result
        get_result_future = goal_handle.get_result_async()
        return get_result_future

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: success={result.result}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        #self.get_logger().info(f'Current position: x={feedback.x:.2f}, y={feedback.y:.2f}')

    # ---------- SERVICES ---------
    def spawn_turtle_service(self, x, y, theta, name):
        client = self.create_client(Spawn, "/turtle1/spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().info("Waiting for service spawn...")

        req = Spawn.Request()
        req.x = x
        req.y = y
        req.theta = theta
        req.name = name
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f"Spawned {future.result().name} at ({x},{y})")

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
        
        self.get_logger().info(f"Color changed to {r,g,b}")

    def teleport_relative_service(self, linear, angular):
        """ Service wrapper to move the turtle to a relative position"""

        # Change the color to the screen color to hide the line
        self.pen_service(self.screen_color[0],self.screen_color[1],self.screen_color[2],self.line_width, 0)

        client = self.create_client(TeleportRelative, "/turtle1/teleport_relative")
        while not client.wait_for_service(1.0):
            self.get_logger().info("Waiting for service teleport relative...")
        
        request = TeleportRelative.Request()
        request.linear = linear
        request.angular = angular

        future = client.call_async(request)
        future.add_done_callback(partial(self.service_callback))

        # Change the color to the default line color to hide the line
        self.pen_service(self.line_default_color[0],self.line_default_color[1],self.line_default_color[2],self.line_width, 0)
        
        self.get_logger().info(f"Turtle teleported to the {linear, angular} relative position")

    def teleport_absoulte_service(self, x, y , theta):
        """ Service wrapper to move the turtle to an absolute position"""

        # Change the color to the screen color to hide the line
        self.pen_service(self.screen_color[0],self.screen_color[1],self.screen_color[2],self.line_width, 0)

        client = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")
        while not client.wait_for_service(1.0):
            self.get_logger().info("Waiting for service teleport absolute...")
        
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta

        future = client.call_async(request)
        future.add_done_callback(partial(self.service_callback))

        # Change the color to the default line color to hide the line
        self.pen_service(self.line_default_color[0],self.line_default_color[1],self.line_default_color[2],self.line_width, 0)
        
        self.get_logger().info(f"Turtle teleported to {x,y,theta}")

    def service_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" %(e,))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleManager()  

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()