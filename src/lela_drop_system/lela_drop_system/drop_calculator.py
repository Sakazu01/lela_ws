#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from mavros_msgs.msg import VfrHud, StatusText
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
import math


class DropCalculatorGPS(Node):
    def __init__(self):
        super().__init__('drop_calculator_gps')

        # ===== PARAMETER =====
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('target_lat', 0.0)
        self.declare_parameter('target_lon', 0.0)

        # ===== STATE =====
        self.altitude = 0.0
        self.groundspeed = 0.0
        self.drone_lat = None
        self.drone_lon = None
        self.calculation_active = False
        self.drop_executed = False

        # ===== SUBSCRIBERS =====
        self.vfr_sub = self.create_subscription(
            VfrHud, '/mavros/vfr_hud', self.vfr_callback, qos_profile_sensor_data
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, qos_profile_sensor_data
        )
        self.calculate_sub = self.create_subscription(
            Bool, '/drop/calculate', self.calculate_callback, 10
        )

        # ===== PUBLISHERS =====
        self.drop_cmd_pub = self.create_publisher(Bool, '/drop/execute', 10)
        self.drop_info_pub = self.create_publisher(Vector3, '/drop/info', 10)
        self.gcs_pub = self.create_publisher(StatusText, '/mavros/statustext/send', 10)

        self.get_logger().info('✅ Drop Calculator GPS-based ready!')

    # ------------------------------------------------------------------
    def gps_callback(self, msg: NavSatFix):
        self.drone_lat = msg.latitude
        self.drone_lon = msg.longitude

    def vfr_callback(self, msg: VfrHud):
        self.altitude = msg.altitude
        self.groundspeed = msg.groundspeed

    def send_to_gcs(self, text):
        msg = StatusText()
        msg.severity = 6
        msg.text = f"DROP: {text}"
        self.gcs_pub.publish(msg)

    # ------------------------------------------------------------------
    def calculate_callback(self, msg):
        if not msg.data or self.calculation_active or self.drop_executed:
            return
        if self.drone_lat is None or self.drone_lon is None:
            self.get_logger().warn('⚠️ Waiting for GPS fix...')
            return

        self.get_logger().info('🚀 Drop calculation triggered!')
        self.send_to_gcs('Calculating GPS-based drop point...')
        self.calculation_active = True

        self.calculate_drop()

    # ------------------------------------------------------------------
    def calculate_drop(self):
        g = self.get_parameter('gravity').value
        h = self.altitude
        v = self.groundspeed

        h = (h * h) + 10
        v = 0.2

        if h <= 0 or v <= 0:
            self.get_logger().warn('⚠️ Invalid flight data')
            self.calculation_active = False
            return

        # Hitung waktu jatuh bebas
        t_fall = math.sqrt((2 * h) / g)
        lead_distance = v * t_fall

        # Hitung jarak ke target dari posisi GPS
        target_lat = self.get_parameter('target_lat').value
        target_lon = self.get_parameter('target_lon').value

        current_distance = self.haversine(self.drone_lat, self.drone_lon, target_lat, target_lon)

        info = Vector3()
        info.x = current_distance
        info.y = lead_distance
        info.z = h
        self.drop_info_pub.publish(info)

        self.get_logger().info(
            f'\n===== DROP CALCULATION =====\n'
            f'Altitude: {h:.1f} m\n'
            f'Groundspeed: {v:.1f} m/s\n'
            f'Lead Distance: {lead_distance:.1f} m\n'
            f'Distance to Target: {current_distance:.1f} m\n'
            f'============================='
        )

        self.send_to_gcs(f'Lead {lead_distance:.0f}m | Dist {current_distance:.0f}m')

        # Jika jarak <= lead distance → drop sekarang
        if current_distance <= lead_distance:
            self.execute_drop()
        else:
            self.get_logger().info('⏳ Target masih jauh, menunggu jarak sesuai...')
            self.create_timer(1.0, self.monitor_distance)

    # ------------------------------------------------------------------
    def monitor_distance(self):
        if self.drop_executed or self.drone_lat is None:
            return

        target_lat = self.get_parameter('target_lat').value
        target_lon = self.get_parameter('target_lon').value
        g = self.get_parameter('gravity').value
        h = self.altitude
        v = self.groundspeed

        if h <= 0 or v <= 0:
            return

        lead_distance = v * math.sqrt((2 * h) / g)
        current_distance = self.haversine(self.drone_lat, self.drone_lon, target_lat, target_lon)
        self.get_logger().info('menunggu drop -1 detik')
        if current_distance <= lead_distance:
            self.execute_drop()

    # ------------------------------------------------------------------
    def execute_drop(self):
        if self.drop_executed:
            return

        self.drop_executed = True
        cmd = Bool()
        cmd.data = True
        self.drop_cmd_pub.publish(cmd)
        self.send_to_gcs('💥 DROP EXECUTED!')
        self.get_logger().info('💥 DROP EXECUTED!')
        self.calculation_active = False

    # ------------------------------------------------------------------
    @staticmethod
    def haversine(lat1, lon1, lat2, lon2):
        """Hitung jarak meter antar dua koordinat"""
        R = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c


def main(args=None):
    rclpy.init(args=args)
    node = DropCalculatorGPS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
