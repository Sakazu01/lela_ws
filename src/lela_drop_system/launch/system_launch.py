#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            parameters=[{
                'image_size': [640, 480],
                'camera_frame_id': 'camera_frame',
            }],
            output='screen'
        ),
        
        Node(
            package='lela_drop_system',
            executable='color_detector.py',
            name='color_detector',
            parameters=[{'min_area': 3000}],
            output='screen'
        ),
        
        Node(
            package='lela_drop_system',
            executable='mission_monitor.py',
            name='mission_monitor',
            output='screen'
        ),
        
        Node(
            package='lela_drop_system',
            executable='state_manager.py',
            name='state_manager',
            parameters=[{
                'drop_waypoint': 1,
                'detection_timeout': 30.0,
            }],
            output='screen'
        ),
        
        Node(
            package='lela_drop_system',
            executable='servo_controller.py',
            name='servo_controller',
            parameters=[{
                'servo_channel': 2,
                'pwm_drop': 1900,
                'pwm_hold': 1100,
                'reset_delay': 2.0,
            }],
            output='screen'
        ),
    ])
