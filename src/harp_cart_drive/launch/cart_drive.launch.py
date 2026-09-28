"""Standalone launch for bench testing. The per-host rpi4u.launch.py in
harp_bringup should include the same Node definition."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("harp_cart_drive"),
                       "config", "rpi4u_cart_drive.yaml")
    return LaunchDescription([
        Node(package="harp_cart_drive", executable="drive_node",
             name="harp_cart_drive", parameters=[cfg], output="screen"),
    ])
