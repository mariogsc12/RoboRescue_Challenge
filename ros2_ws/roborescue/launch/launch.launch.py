from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    
    turtlesim = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    controller = Node(
        package="roborescue",
        executable="turtleController.py",
        parameters=[os.path.join(get_package_share_directory("roborescue"),"config","config.yaml")]
    )

    return LaunchDescription([
        turtlesim,
        controller
    ])