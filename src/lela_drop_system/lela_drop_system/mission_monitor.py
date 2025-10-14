#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import WaypointReached
from std_msgs.msg import Int16

class MissionMonitor(Node):
    def __init__(self):
        super().__init__('mission_monitor')
        
        # Subscribe waypoint reached dari MAVROS
        self.wp_sub = self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.waypoint_callback,
            10
        )
        
        # Publisher waypoint reached ke system
        self.wp_pub = self.create_publisher(Int16, '/mission/waypoint_reached', 10)
        
        self.last_waypoint = -1
        
        self.get_logger().info('✅ Mission Monitor ready (Simple mode)')
    
    def waypoint_callback(self, msg):
        """Callback saat waypoint reached dari MAVROS"""
        wp_num = msg.wp_seq
        
        # Skip duplicate
        if wp_num == self.last_waypoint:
            return
        
        self.last_waypoint = wp_num
        self.get_logger().info(f'📍 Waypoint {wp_num} reached')
        
        # Publish ke State Manager
        wp_msg = Int16()
        wp_msg.data = wp_num
        self.wp_pub.publish(wp_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()