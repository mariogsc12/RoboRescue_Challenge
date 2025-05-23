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

        self._action_server = ActionServer(
            self,
            GoTo,
            'GoTo',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
        self.goals = {}  
        self.cmd_vel_pubs = {}  
        self.pose_subs = {}     
        self.poses = {}         
        self.current_poses = {}         
        
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


    def pose_callback_factory(self, turtle_id):
        def callback(msg):
            self.poses[turtle_id] = msg
        return callback

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

        # Store input parameters 
        goal = goal_handle.request
        turtle_id = goal.turtle_id
        target_x = goal.x
        target_y = goal.y
        target_theta = goal.theta

        self.get_logger().info('Executing goal...')

        pose = self.poses.get(goal.turtle_id)

        if pose is None:
            self.get_logger().warn(f"No pose yet for turtle id {goal.turtle_id}, waiting...")
            time.sleep(0.1)


        # Create publisher and subscriber if not exist for this turtle 
        if turtle_id not in self.cmd_vel_pubs:
            cmd_vel_topic = f'/turtle{turtle_id}/cmd_vel'
            self.cmd_vel_pubs[turtle_id] = self.create_publisher(Twist, cmd_vel_topic, 10)
            self.get_logger().info(f'Publisher creado en {cmd_vel_topic}')
        
        if turtle_id not in self.pose_subs:
            pose_topic = f'/turtle{turtle_id}/pose'
            self.pose_subs[turtle_id] = self.create_subscription(
                Pose,
                pose_topic,
                self.pose_callback_factory(turtle_id),
                10
            )
            current_pose = self.poses[turtle_id] = Pose()
            self.get_logger().info(f'Subscriber {pose_topic} created for turtle{turtle_id} ')

        publisher = self.cmd_vel_pubs[turtle_id]
        new_vel = Twist()

        if turtle_id not in self.poses:
            self.get_logger().info(f"No pose yet for turtle id {turtle_id}, waiting...")
            return
        current_pose = self.poses[turtle_id]

        # Initialize errors
        linear_x_error = target_x - current_pose.x
        linear_y_error = target_y - current_pose.y
        dist_error = math.hypot(linear_x_error, linear_y_error)
        goal_angle = math.atan2(linear_y_error, linear_x_error)
        angular_error = goal_angle - current_pose.theta
        
        self.get_logger().info(f'Executing goal for turtle{turtle_id}: x={target_x}, y={target_y} ')

        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info(f'Goal canceled for turtle{turtle_id}')
                return GoTo.Result()
            
            current_pose = self.poses[turtle_id]

            # Update errors
            linear_x_error = target_x - current_pose.x
            linear_y_error = target_y - current_pose.y
            dist_error = math.hypot(linear_x_error, linear_y_error)
            goal_angle = math.atan2(linear_y_error, linear_x_error)
            angular_error = self.normalize_angle(goal_angle - current_pose.theta)

            if (abs(angular_error) < self.angular_tolerance and dist_error < self.distance_tolerance):
                break

            # Proportional control
            #self.get_logger().info(f'angular error: {angular_error:.2f}, linear_x: {linear_x_error:.2f}, linear_y: {linear_y_error:.2f}')
            if abs(angular_error) > self.angular_tolerance:
                new_vel.angular.z = max(min(self.kp_angle * angular_error, self.max_angular_vel), -self.max_angular_vel)
                new_vel.linear.x = 0.0
                #self.get_logger().info(f'changing angular vel')
            else:
                if dist_error >= self.distance_tolerance:
                    new_vel.linear.x = self.kp_dist * dist_error
                    #self.get_logger().info(f'changing linear vel')
                else:
                    new_vel.linear.x = 0.0
                    new_vel.angular.z = 0.0

            publisher.publish(new_vel)

            # Update feedback
            feedback = GoTo.Feedback()
            feedback.x = current_pose.x
            feedback.y = current_pose.y
            feedback.theta = current_pose.theta
            goal_handle.publish_feedback(feedback)

            time.sleep(0.05)


        # Stop the turtle
        publisher.publish(Twist())

        goal_handle.succeed()
        result = GoTo.Result()
        result.result = True
        self.get_logger().info(f'Goal completed for turtle{turtle_id}')

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