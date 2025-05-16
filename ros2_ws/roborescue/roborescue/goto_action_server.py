#!/usr/bin/env python3

import time

from roborescue.action import GoTo

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

import math
import asyncio

class GoToActionServer(Node):

    def __init__(self):
        super().__init__('goto_action_server')

        self.goal = GoTo.Goal()
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.poseCallback, 10)

        self._action_server = ActionServer(
            self,
            GoTo,
            'GoTo',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
        self.current_pose = Pose()
        self.current_pose.x = 0.0
        self.current_pose.y = 0.0
        self.current_pose.theta = 0.0
        
        self.get_config_parameters()
        
        self.get_logger().info("GoTo action server has been initialized")

    def get_config_parameters(self):
        self.declare_parameter('angular_tolerance', 0.1) 
        self.declare_parameter('distance_tolerance', 0.1) 
        self.declare_parameter('kp_dist', 1.5) 
        self.declare_parameter('kp_angle', 1.5) 
        self.declare_parameter('max_angular_vel', 2.0) 

        self.kp_dist = self.get_parameter('kp_dist').get_parameter_value().double_value
        self.kp_angle = self.get_parameter('kp_angle').get_parameter_value().double_value
        self.angular_tolerance = self.get_parameter('angular_tolerance').get_parameter_value().double_value
        self.distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
 
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def poseCallback(self, msg:Pose):
        self.current_pose = msg

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        self.goal = goal_request
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def normalize_angle(self,angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    async def execute_callback(self, goal_handle):
        """Execute a goal."""
        self.get_logger().info('Executing goal...')

        goal = goal_handle.request
        self.new_vel = Twist()

        target_x = goal.x
        target_y = goal.y

        # Initialize errors
        linear_x_error = target_x - self.current_pose.x
        linear_y_error = target_y - self.current_pose.y
        dist_error = math.hypot(linear_x_error, linear_y_error)
        goal_angle = math.atan2(linear_y_error, linear_x_error)
        angular_error = goal_angle - self.current_pose.theta
        

        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return GoTo.Result()

            # Update errors
            linear_x_error = target_x - self.current_pose.x
            linear_y_error = target_y - self.current_pose.y
            dist_error = math.hypot(linear_x_error, linear_y_error)
            goal_angle = math.atan2(linear_y_error, linear_x_error)
            angular_error = self.normalize_angle(goal_angle - self.current_pose.theta)

            if (abs(angular_error) < self.angular_tolerance and dist_error < self.distance_tolerance):
                break

            # Proportional control
            self.get_logger().info(f'angular error: {angular_error}, linear: {linear_x_error, linear_y_error}')
            if abs(angular_error) > self.angular_tolerance:
                self.new_vel.angular.z = max(min(self.kp_angle * angular_error, self.max_angular_vel), -self.max_angular_vel)
                self.new_vel.linear.x = 0.0
                self.get_logger().info(f'changing angular vel')
            else:
                if dist_error >= self.distance_tolerance:
                    self.new_vel.linear.x = self.kp_dist * dist_error
                    self.get_logger().info(f'changing linear vel')
                else:
                    self.new_vel.linear.x = 0.0
                    self.new_vel.angular.z = 0.0

            self.cmd_vel_pub.publish(self.new_vel)

            # Update feedback
            feedback = GoTo.Feedback()
            feedback.x = self.current_pose.x
            feedback.y = self.current_pose.y
            feedback.theta = self.current_pose.theta
            goal_handle.publish_feedback(feedback)

            time.sleep(0.05)


        # Stop the turtle
        self.cmd_vel_pub.publish(Twist())

        goal_handle.succeed()
        result = GoTo.Result()
        result.result = True
        self.get_logger().info('Returning result: True')

        return result



def main(args=None):
    rclpy.init(args=args)
    action_server = GoToActionServer()

    executor = MultiThreadedExecutor()
    executor.add_node(action_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        action_server.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()