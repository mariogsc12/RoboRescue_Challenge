from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    turtlesim = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    return LaunchDescription([
        turtlesim
    ])