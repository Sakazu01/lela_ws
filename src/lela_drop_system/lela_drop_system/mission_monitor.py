\#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import WaypointReached, WaypointList
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32

class MissionMonitor(Node):
    def __init__(self):
        super().__init__('mission_monitor')

        # ===== STATE =====
        self.current_wp = -1
        self.total_wp = 0
        self.waypoints = []

        # ===== PUBLISHERS =====
        self.wp_pub = self.create_publisher(Int32, '/mission/current_wp', 10)
        self.target_pub = self.create_publisher(NavSatFix, '/target_coords', 10)

        # ===== SUBSCRIBERS =====
        self.wp_reached_sub = self.create_subscription(
            WaypointReached, '/mavros/mission/reached', self.wp_reached_cb, 10
        )
        self.wp_list_sub = self.create_subscription(
            WaypointList, '/mavros/mission/waypoints', self.wp_list_cb, 10
        )

        self.get_logger().info('✅ Mission Monitor aktif (sinkron ke target_coords).')

    # Dapat semua waypoint dari MAVROS
    def wp_list_cb(self, msg: WaypointList):
        self.waypoints = msg.waypoints
        self.total_wp = len(msg.waypoints)
        self.get_logger().info(f"📋 Dapat {self.total_wp} waypoint dari MAVROS.")

    # Saat waypoint tercapai
    def wp_reached_cb(self, msg: WaypointReached):
        wp_num = msg.wp_seq
        self.current_wp = wp_num
        self.get_logger().info(f"🚩 Waypoint {wp_num} tercapai!")

        # Publish info waypoint aktif
        wp_msg = Int32()
        wp_msg.data = wp_num
        self.wp_pub.publish(wp_msg)

        # Jika waypoint valid → kirim koordinatnya ke drop_calculator
        if 0 <= wp_num < len(self.waypoints):
            wp = self.waypoints[wp_num]
            target = NavSatFix()
            target.latitude = wp.x_lat
            target.longitude = wp.y_long
            target.altitude = wp.z_alt
            self.target_pub.publish(target)

            self.get_logger().info(
                f"🎯 Publish target_coords: lat={wp.x_lat:.7f}, lon={wp.y_long:.7f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = MissionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down Mission Monitor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
