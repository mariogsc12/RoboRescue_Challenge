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
from drawer_manager import DrawerManager, DIRECTION
from utils import *

import time
import random
import math
import sys

SCREEN_SIZE = 11

class TurtleManager(Node):
    def __init__(self):
        super().__init__("turtle_manager")

        self.get_logger().info(INITIAL_MESSAGE)

        self._action_client = ActionClient(self, GoTo, 'GoTo')

        self.get_config_parameters()

        self.drawer_manager = DrawerManager(self.letter_height, self.letter_width)
        self.get_logger().info("Turtle manager created.")

        self.trajectory_planner()

        self.get_logger().info(FINAL_MESSAGE)
        

    def get_config_parameters(self):
        self.declare_parameter('word_up', "R") 
        self.declare_parameter('word_down', "R") 
        self.declare_parameter('draw_robot', True)
        self.declare_parameter('initial_pos_word_up', [2.0,6.0]) 
        self.declare_parameter('initial_pos_word_down', [8.9,1.5]) 
        self.declare_parameter('initial_pos_robot', [9.75, 6.5])
        self.declare_parameter('word_line_red', [0, 255]) 
        self.declare_parameter('word_line_green', [0, 255]) 
        self.declare_parameter('word_line_blue', [0, 255]) 
        self.declare_parameter('letter_width', 1.0) 
        self.declare_parameter('letter_height', 3.0) 
        self.declare_parameter('screen_color', [0,0,255]) 
        self.declare_parameter('line_default_color', [255,255,255]) 
        self.declare_parameter('line_width', 3) 
        self.declare_parameter('letter_offset', [0.25,0.0]) 

        self.word_up = self.get_parameter('word_up').get_parameter_value().string_value
        self.word_down = self.get_parameter('word_down').get_parameter_value().string_value
        self.draw_robot_config = self.get_parameter('draw_robot').get_parameter_value().bool_value
        self.initial_pos_word_up = self.get_parameter('initial_pos_word_up').get_parameter_value().double_array_value
        self.initial_pos_word_down = self.get_parameter('initial_pos_word_down').get_parameter_value().double_array_value
        self.initial_pos_robot = self.get_parameter('initial_pos_robot').get_parameter_value().double_array_value
        self.word_line_red = self.get_parameter('word_line_red').get_parameter_value().integer_array_value
        self.word_line_green = self.get_parameter('word_line_green').get_parameter_value().integer_array_value
        self.word_line_blue = self.get_parameter('word_line_blue').get_parameter_value().integer_array_value
        self.letter_width = self.get_parameter('letter_width').get_parameter_value().double_value
        self.letter_height = self.get_parameter('letter_height').get_parameter_value().double_value
        self.screen_color = self.get_parameter('screen_color').get_parameter_value().integer_array_value
        self.line_default_color = self.get_parameter('line_default_color').get_parameter_value().integer_array_value
        self.line_width = self.get_parameter('line_width').get_parameter_value().integer_value
        letter_offset = self.get_parameter('letter_offset').get_parameter_value().double_array_value


        colors = [self.word_line_red, self.word_line_green, self.word_line_blue, self.line_default_color, self.screen_color]

        for color in colors:
            for i in range(len(color)):
                self.constrain(color[i], 0, 255)

        pos_to_check = [self.initial_pos_word_up, self.initial_pos_word_down, letter_offset]
        for pos in pos_to_check:
            for i in range(len(pos)):
                self.constrain(pos[i], 0, SCREEN_SIZE)

        self.letter_offset_x = letter_offset[0]
        self.letter_offset_y = letter_offset[1]

        if self.initial_pos_word_up[1] <= self.initial_pos_word_down[1]:
            self.get_logger().warning(f'\n \n --- Received word up position_y {self.initial_pos_word_up[1]} is greater than word down {self.initial_pos_word_down[1]} --- \n \n')  


    
    def trajectory_planner(self):
        """ Wrapper function to manage the logic """
        
        initial_x_up, initial_y_up = self.initial_pos_word_up
        initial_x_down, initial_y_down = self.initial_pos_word_down
        initial_x_robot, initial_y_robot = self.initial_pos_robot


        self.draw_word(self.word_up, initial_x_up, initial_y_up, "RIGHT")

        if initial_x_up + self.letter_width - 2.15 > SCREEN_SIZE or initial_y_up + self.letter_height > SCREEN_SIZE:
            self.get_logger().warning(f"The configured parameters are invalid: the robot initial position '{initial_x_robot},{initial_y_robot}' exceeds the screen width.")
        else:
            self.draw_robot(initial_x_robot, initial_y_robot)

        self.draw_word(self.word_down, initial_x_down, initial_y_down, "LEFT")
        


    def draw_robot(self, initial_x, initial_y):
        "Logic to draw a robot shilouette whether you want to draw it or not"
        robot_parts = ["MOUTH", "EYE", "EYE", "ROBOT"]
        eye_count = 0

        if not self.draw_robot_config:
            self.get_logger().info(f'\n \n --- Robot not configured.  :''( . Skipping --- \n \n')
            return

        self.get_logger().info(f'\n \n --- Drawing robot :) --- \n \n')
        current_x = initial_x
        current_y = initial_y
        current_theta = 0.0

        for part in robot_parts:
            if part == "MOUTH":
                current_x, current_y, current_theta = initial_x, initial_y, 0.0
                self.teleport_absoulte_service(current_x, current_y, current_theta)

            elif part == "EYE" and eye_count == 0:
                self.get_logger().info("Teleporting up for first eye")
                current_x, current_y, current_theta = initial_x - 0.5, initial_y + 0.75, math.pi / 2
                self.teleport_absoulte_service(current_x, current_y, current_theta)
                eye_count = 1

            elif part == "EYE" and eye_count == 1:
                self.get_logger().info("Teleporting left for second eye")
                current_x, current_y, current_theta = initial_x - 1.5, initial_y + 0.75, math.pi / 2
                self.teleport_absoulte_service(current_x, current_y, current_theta)

            elif part == "ROBOT":
                self.get_logger().info("Teleporting for face")
                current_x, current_y, current_theta = initial_x - 2.15, initial_y + 0.5, math.pi / 2
                self.teleport_absoulte_service(current_x, current_y, current_theta)

            # Generar waypoints desde la posición actual
            waypoints_up = self.drawer_manager.robot_manager(part, current_x, current_y)
            if waypoints_up is None:
                self.get_logger().warning(f"Robot part {part} is not configured. Skipping")
                continue

            for wp_up in waypoints_up:
                goal_future = self.send_goal(wp_up[0], wp_up[1], 0.0)
                rclpy.spin_until_future_complete(self, goal_future)


                
    def draw_word(self, word, initial_x, initial_y, direction):
        """ Logic to draw a word using the initial position and the direction of the turtle """

        if not self.check_configuration(word, initial_x, initial_y, direction):
            return
        
        if word:
            self.get_logger().info(f'\n \n --- Starting to draw {word} --- \n \n')
            if direction == "LEFT":
                word = reversed(word)
            for letter in word:
                self.get_logger().info(f'\n \n --- Starting to draw {letter} at {round(initial_x,2),round(initial_y,2)} --- \n \n')
                self.teleport_absoulte_service(initial_x,initial_y,0.0)
                self.pen_service(random.randint(self.word_line_red[0],self.word_line_red[1]),
                                 random.randint(self.word_line_green[0], self.word_line_green[1]),
                                 random.randint(self.word_line_blue[0], self.word_line_blue[1]))
                
                waypoints_up = self.drawer_manager.manager(letter,initial_x,initial_y, direction)

                # Loop to move the turtle to the waypoints of the letter 
                if waypoints_up is None:
                    self.get_logger().warning(f"Letter {letter} is not configured. Skipping")
                    continue

                for wp_up in waypoints_up:
                    goal_future = self.send_goal(wp_up[0], wp_up[1], 0.0)
                    rclpy.spin_until_future_complete(self, goal_future)

                if direction == "RIGHT":
                    initial_x = initial_x + self.letter_width + self.letter_offset_x
                    initial_y = initial_y + self.letter_offset_y
                elif direction == "LEFT":
                    initial_x = initial_x - self.letter_width - self.letter_offset_x
                    initial_y = initial_y + self.letter_offset_y
        else:
            self.get_logger().info(f'\n \n --- Word not configured. Skipping --- \n \n')


    # -------- ACTIONS -------
    def send_goal(self, x, y, theta):
        # Wait server
        self._action_client.wait_for_server()

        self.get_logger().info(f"Sending goal: {round(x,2), round(y,2), round(theta,2)}")
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

        self.get_logger().info('Goal accepted from action server')

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

    def pen_service(self, r, g, b, width=None, off=0):
        if width == None:
            width = self.line_width

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
        rclpy.spin_until_future_complete(self, future)
        
        self.get_logger().info(f"Color changed to {r,g,b}")

        # Add small delay to ensure color change
        time.sleep(0.2)

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
        rclpy.spin_until_future_complete(self, future)

        # Change the color to the default line color to hide the line
        self.pen_service(self.line_default_color[0],self.line_default_color[1],self.line_default_color[2],self.line_width, 0)
        
        self.get_logger().info(f"Turtle teleported to the {round(linear,2), round(angular,2)} relative position")

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
        rclpy.spin_until_future_complete(self, future)

        # Change the color to the default line color to hide the line
        self.pen_service(self.line_default_color[0],self.line_default_color[1],self.line_default_color[2],self.line_width, 0)
        
        self.get_logger().info(f"Turtle teleported to {round(x,2),round(y,2),round(theta,2)}")

    def service_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" %(e,))
    
    # ---------- AUXILIAR FUNCTIONS ---------
    def constrain(self, value, min, max):
        if value > max:
            value = max
        elif value < min:
            value = min

    def check_configuration(self, word, initial_x, initial_y, direction):

        direction = direction.upper()
        if direction not in DIRECTION:
            self.get_logger().warning(f'\n \n --- Received direction {direction} is not configured. Please use {DIRECTION} --- \n \n')  
            return False
        
        if direction == "RIGHT":
            if initial_x + (self.letter_width + self.letter_offset_x) * (len(word)-1) > SCREEN_SIZE:
                self.get_logger().warning(
                    f"The configured parameters are invalid: the word '{word}' exceeds the screen width.")
                return False
            if self.letter_offset_y > 0:
                if initial_y + self.letter_height + self.letter_offset_y * (len(word)-1) > SCREEN_SIZE:
                    self.get_logger().warning(
                        f"The configured parameters are invalid: the word '{word}' exceeds the screen height.")
                    return False
            else:
                if initial_y + self.letter_offset_y * (len(word)-1) < SCREEN_SIZE/2:
                    self.get_logger().warning(
                        f"The configured parameters are invalid: the word '{word}' height is too low (above mid screen).")
                    return False

        elif direction == "LEFT":
            if initial_x - (self.letter_width + self.letter_offset_x) * (len(word)-1) < 0:
                self.get_logger().warning(
                    f"The configured parameters are invalid: the word '{word}' exceeds the screen width on the left.")
                return False
            if self.letter_offset_y > 0:
                if initial_y + self.letter_height + self.letter_offset_y * (len(word)-1) > SCREEN_SIZE / 2:
                    self.get_logger().warning(
                        f"The configured parameters are invalid: the word '{word}' height is too low (below mid screen).")
                    return False
            else:
                if initial_y + self.letter_offset_y * (len(word)-1) < 0:
                    self.get_logger().warning(
                        f"The configured parameters are invalid: the word '{word}' exceeds the screen height (above).")
                    return False
                
        return True



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