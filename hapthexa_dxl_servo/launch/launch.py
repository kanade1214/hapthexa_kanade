from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="hapthexa_dxl_servo",
            executable="force_sensor.py",
            name="force_sensor",
            namespace="hapthexa/leg/front_left",
            parameters=[os.path.join(get_package_share_directory('hapthexa_dxl_servo'),'config','front_left.yaml')]
        ),
        Node(
            package="hapthexa_dxl_servo",
            executable="force_sensor.py",
            name="force_sensor",
            namespace="hapthexa/leg/middle_left",
            parameters=[os.path.join(get_package_share_directory('hapthexa_dxl_servo'),'config','middle_left.yaml')]
        ),
        Node(
            package="hapthexa_dxl_servo",
            executable="force_sensor.py",
            name="force_sensor",
            namespace="hapthexa/leg/rear_left",
            parameters=[os.path.join(get_package_share_directory('hapthexa_dxl_servo'),'config','rear_left.yaml')]
        ),
        Node(
            package="hapthexa_dxl_servo",
            executable="force_sensor.py",
            name="force_sensor",
            namespace="hapthexa/leg/front_right",
            parameters=[os.path.join(get_package_share_directory('hapthexa_dxl_servo'),'config','front_right.yaml')]
        ),
        Node(
            package="hapthexa_dxl_servo",
            executable="force_sensor.py",
            name="force_sensor",
            namespace="hapthexa/leg/middle_right",
            parameters=[os.path.join(get_package_share_directory('hapthexa_dxl_servo'),'config','middle_right.yaml')]
        ),
        Node(
            package="hapthexa_dxl_servo",
            executable="force_sensor.py",
            name="force_sensor",
            namespace="hapthexa/leg/rear_right",
            parameters=[os.path.join(get_package_share_directory('hapthexa_dxl_servo'),'config','rear_right.yaml')]
        ),
        Node(
            package="hapthexa_dxl_servo",
            executable="forcesensor_read.py",
            name="forcesensor_read"
        ),
        Node(
            package="hapthexa_dxl_servo",
            executable="hapthexa_servo"
        )
    ])