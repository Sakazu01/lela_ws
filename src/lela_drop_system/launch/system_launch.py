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
            parameters=[{'min_area': 3000}],
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
                'drop_waypoint': 1,
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
                'target_distance': 2.0,
                'min_altitude': 10.0,
                'max_altitude': 100.0,
                'min_airspeed': 5.0,
            }],
            output='screen'
        ),
        
        # Servo Controller
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