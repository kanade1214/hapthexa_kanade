from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="hapthexa_main",
            executable="hapthexa_leg",
            name="hapthexa_leg_front_left",
            namespace="hapthexa/leg/front_left"
        ),
        Node(
            package="hapthexa_main",
            executable="hapthexa_leg",
            name="hapthexa_leg_middle_left",
            namespace="hapthexa/leg/middle_left",
            parameters=[{'leg_install_angle': 1.57079632679}]
        ),
        Node(
            package="hapthexa_main",
            executable="hapthexa_leg",
            name="hapthexa_leg_rear_left",
            namespace="hapthexa/leg/rear_left",
            parameters=[{'leg_install_angle': 3.14159265359}]
        ),
        Node(
            package="hapthexa_main",
            executable="hapthexa_leg",
            name="hapthexa_leg_front_right",
            namespace="hapthexa/leg/front_right"
        ),
        Node(
            package="hapthexa_main",
            executable="hapthexa_leg",
            name="hapthexa_leg_middle_right",
            namespace="hapthexa/leg/middle_right",
            parameters=[{'leg_install_angle': -1.57079632679}]
        ),
        Node(
            package="hapthexa_main",
            executable="hapthexa_leg",
            name="hapthexa_leg_rear_right",
            namespace="hapthexa/leg/rear_right",
            parameters=[{'leg_install_angle': -3.14159265359}]
        ),
        Node(
            package="hapthexa_main",
            executable="attitude_controller.py",
            name="attitude_controller"
        )
    ])