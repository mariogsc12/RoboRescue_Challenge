#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from turtlesim.srv import Spawn
from roborescue.msg import SpawnMsg

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.get_logger().info("Turtle spawner created.")
        self.nb_turtles = 1
        self.spawner_sub_ = self.create_subscription(SpawnMsg, "/spawn",self.spawnerCallback, 10)
        self.client = self.create_client(Spawn, "/spawn")

    def spawnerCallback(self, msg: SpawnMsg):
        if msg.turtle_id == self.nb_turtles + 1:
            self.spawn_turtle_service(msg.x, msg.y, msg.theta, msg.turtle_id)
            self.nb_turtles = msg.turtle_id


    def spawn_turtle_service(self, x, y, theta, turtle_id):
        req = Spawn.Request()
        req.x = x
        req.y = y
        req.theta = theta
        req.name = f"turtle{turtle_id}"

        future = self.client.call_async(req)
        future.add_done_callback(
            lambda fut: self.spawn_response_callback(fut, x, y, turtle_id)
        )

    def spawn_response_callback(self, future, x, y, turtle_id):
        try:
            result = future.result()
            self.get_logger().info(f"Spawned {result.name} at ({x},{y})")
        except Exception as e:
            self.get_logger().error(f"Failed to spawn turtle{turtle_id}: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()