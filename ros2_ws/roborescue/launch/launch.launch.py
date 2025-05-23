from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    
    turtlesim = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    manager = Node(
        package="roborescue",
        executable="drawerManager.py",
        parameters=[os.path.join(get_package_share_directory("roborescue"),"config","config.yaml")]
    )

    spawner = Node(
        package="roborescue",
        executable="turtleSpawner.py",
        parameters=[os.path.join(get_package_share_directory("roborescue"),"config","config.yaml")]
    )

    action_server = Node(
        package="roborescue",
        executable="goto_action_server.py",
        parameters=[os.path.join(get_package_share_directory("roborescue"),"config","config.yaml")]
    )

    return LaunchDescription([
        turtlesim,
        manager,
        spawner,
        action_server
    ])