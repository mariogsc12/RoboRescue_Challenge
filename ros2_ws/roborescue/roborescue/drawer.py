#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class Drawer(Node):
    def __init__(self):
        super().__init__("drawer")

def main():
    rclpy.init()
    drawer = Drawer()
    rclpy.spin(drawer)
    drawer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()