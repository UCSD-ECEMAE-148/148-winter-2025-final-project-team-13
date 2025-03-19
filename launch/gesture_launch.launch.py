#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gestures_package',
            executable='gesture_tracker',
            name='gesture_tracker'
        ),
        Node(
            package='gestures_package',
            executable='gesture_controller',
            name='gesture_controller'
        )
#        Node(
#            package='gestures_package',
#            executable='finger_counter',
#            name='finger_counter'
#        )
    ])
