#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera Node
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
        
        # Color Detector
        Node(
            package='lela_drop_system',
            executable='color_detector.py',
            name='color_detector',
            parameters=[
                {'fallback_color': 'RED'},  # atau 'BLUE'
                {'status_log_period': 1.5}                
            ],
            output='screen'
        ),
        
        # Mission Monitor
        Node(
            package='lela_drop_system',
            executable='mission_monitor.py',
            name='mission_monitor',
            output='screen'
        ),
        
        # State Manager (UPDATED)
        Node(
            package='lela_drop_system',
            executable='state_manager.py',
            name='state_manager',
            parameters=[{
                'drop_waypoint': 0,
                'detection_timeout': 30.0,
            }],
            output='screen'
        ),
        
        # Drop Calculator (UPDATED)
        Node(
            package='lela_drop_system',
            executable='drop_calculator.py',
            name='drop_calculator',
            parameters=[{
                'gravity': 9.81,
            }],
            output='screen'
        ),
        
        # Servo Controller
        Node(
            package='lela_drop_system',
            executable='servo_controller.py',
            name='servo_controller',
            parameters=[{
                'servo_channel': 7,
                'pwm_drop': 2200,
                'pwm_hold': 800,
                'reset_delay': 2.0,
            }],
            output='screen'
        ),
    ])
