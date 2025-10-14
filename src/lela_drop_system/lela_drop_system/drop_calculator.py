#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Bool
from mavros_msgs.msg import VfrHud, StatusText
from geometry_msgs.msg import Vector3
import math

class DropCalculator(Node):
    def __init__(self):
        super().__init__('drop_calculator')
        
        # Parameters
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('target_distance', 2.0)
        self.declare_parameter('min_altitude', 10.0)
        self.declare_parameter('max_altitude', 100.0)
        self.declare_parameter('min_airspeed', 5.0)
        
        # Validate parameters
        self._validate_parameters()
        
        # State variables
        self.altitude = 0.0
        self.airspeed = 0.0
        self.groundspeed = 0.0
        self.detected_color = None
        self.calculation_enabled = False
        self.calculation_done = False
        self.countdown_timer = None
        
        # Subscribers
        self.vfr_sub = self.create_subscription(
            VfrHud, '/mavros/vfr_hud', self.vfr_callback, qos_profile_sensor_data
        )
        self.state_sub = self.create_subscription(
            String, '/system/state', self.state_callback, 10
        )
        self.color_sub = self.create_subscription(
            String, '/detection/color', self.color_callback, 10
        )
        
        # Publishers
        self.drop_cmd_pub = self.create_publisher(Bool, '/drop/execute', 10)
        self.drop_info_pub = self.create_publisher(Vector3, '/drop/info', 10)
        self.gcs_pub = self.create_publisher(StatusText, '/mavros/statustext/send', 10)
        
        self.get_logger().info('✅ Drop Calculator ready')
    
    def _validate_parameters(self):
        """Validate parameter ranges"""
        min_alt = self.get_parameter('min_altitude').value
        max_alt = self.get_parameter('max_altitude').value
        min_speed = self.get_parameter('min_airspeed').value
        
        if min_alt >= max_alt:
            raise ValueError(
                f'min_altitude ({min_alt}) must be < max_altitude ({max_alt})'
            )
        
        if min_speed <= 0:
            raise ValueError(f'min_airspeed must be > 0')
        
        self.get_logger().info(
            f'📐 Altitude range: {min_alt}-{max_alt}m, Min speed: {min_speed}m/s'
        )
    
    def send_to_gcs(self, text):
        """Send message to GCS"""
        msg = StatusText()
        msg.severity = 6  # INFO level
        msg.text = f"DROP_CALC: {text}"
        self.gcs_pub.publish(msg)
    
    def state_callback(self, msg):
        """Handle state changes from State Manager"""
        state = msg.data
        
        if state == 'DETECTING':
            # Reset saat mulai deteksi baru
            self.detected_color = None
            self.calculation_enabled = False
            self.calculation_done = False
            
            # Cancel countdown jika ada
            if self.countdown_timer:
                self.get_logger().warn('⚠️ Cancelling countdown (new detection)')
                self.countdown_timer.cancel()
                self.countdown_timer = None
        
        elif state == 'DROPPING':
            # State Manager sudah putuskan untuk drop
            # Drop Calculator hanya execute countdown
            pass
        
        else:
            # State lain (IDLE, WAITING_WAYPOINT)
            if self.calculation_enabled:
                self.get_logger().info('📴 Drop calculation DISABLED')
            
            self.calculation_enabled = False
            self.calculation_done = False
            self.detected_color = None
            
            # Cancel countdown jika ada
            if self.countdown_timer:
                self.get_logger().warn('⚠️ Cancelling countdown (state change)')
                self.countdown_timer.cancel()
                self.countdown_timer = None
    
    def color_callback(self, msg):
        """Handle color detection from Color Detector"""
        color = msg.data
        self.detected_color = color.upper()
        
        # Enable calculation setelah warna terdeteksi
        if not self.calculation_enabled:
            self.calculation_enabled = True
            self.calculation_done = False
            self.get_logger().info(f'🎨 Color detected: {self.detected_color} - Calculation ENABLED')
            self.send_to_gcs(f'{self.detected_color} detected - Ready to calculate')
    
    def vfr_callback(self, msg):
        """Update vehicle flight data"""
        self.altitude = msg.altitude
        self.airspeed = msg.airspeed
        self.groundspeed = msg.groundspeed
        
        # Only calculate if:
        # 1. Calculation enabled (warna sudah terdeteksi)
        # 2. Belum pernah calculate (prevent re-calculation)
        if self.calculation_enabled and not self.calculation_done:
            self.calculate_drop()
    
    def calculate_drop(self):
        """Calculate drop timing and execute countdown"""
        
        # Get parameters
        g = self.get_parameter('gravity').value
        target_dist = self.get_parameter('target_distance').value
        min_alt = self.get_parameter('min_altitude').value
        max_alt = self.get_parameter('max_altitude').value
        min_speed = self.get_parameter('min_airspeed').value
        
        # Validate altitude
        if self.altitude < min_alt:
            self.get_logger().warn(
                f'⚠️ Altitude too low: {self.altitude:.1f}m < {min_alt}m'
            )
            self.send_to_gcs(f'Alt too low: {self.altitude:.1f}m')
            return
        
        if self.altitude > max_alt:
            self.get_logger().warn(
                f'⚠️ Altitude too high: {self.altitude:.1f}m > {max_alt}m'
            )
            self.send_to_gcs(f'Alt too high: {self.altitude:.1f}m')
            return
        
        # Select speed (prefer airspeed)
        speed = self.airspeed if self.airspeed >= min_speed else self.groundspeed
        
        if speed < min_speed:
            self.get_logger().warn(
                f'⚠️ Speed too low: {speed:.1f}m/s < {min_speed}m/s'
            )
            self.send_to_gcs(f'Speed too low: {speed:.1f}m/s')
            return
        
        # Calculate free fall time: t = sqrt(2h/g)
        t_fall = math.sqrt((2 * self.altitude) / g)
        
        # Calculate horizontal distance during fall: d = v * t
        d_drop = speed * t_fall
        
        # Calculate time until drop point
        if d_drop <= target_dist:
            self.get_logger().warn(
                f'⚠️ Drop distance {d_drop:.1f}m ≤ target {target_dist}m - '
                f'payload will overshoot!'
            )
            self.send_to_gcs(f'Drop dist {d_drop:.1f}m too short!')
            return
        
        time_to_drop = (d_drop - target_dist) / speed
        
        # Sanity check
        if time_to_drop <= 0:
            self.get_logger().error('❌ Calculated time_to_drop ≤ 0, skipping')
            return
        
        # Publish drop info (untuk monitoring/debugging)
        info = Vector3()
        info.x = time_to_drop
        info.y = d_drop
        info.z = self.altitude
        self.drop_info_pub.publish(info)
        
        self.get_logger().info(
            f'📊 Drop calculation: '
            f't_fall={t_fall:.2f}s, '
            f'd_drop={d_drop:.1f}m, '
            f'countdown={time_to_drop:.2f}s'
        )
        
        self.send_to_gcs(
            f'Countdown {time_to_drop:.1f}s (alt={self.altitude:.0f}m, spd={speed:.1f}m/s)'
        )
        
        # Mark calculation as done to prevent re-calculation
        self.calculation_done = True
        
        # Start countdown timer
        self.start_countdown(time_to_drop)
    
    def start_countdown(self, delay):
        """Start countdown timer"""
        
        # Double-check no active timer (safety)
        if self.countdown_timer:
            self.get_logger().error('❌ Timer already active! This should not happen.')
            return
        
        self.get_logger().info(f'🚀 Countdown started: {delay:.2f} seconds')
        self.countdown_timer = self.create_timer(delay, self.execute_drop)
    
    def execute_drop(self):
        """Execute drop command"""
        self.get_logger().info('💥 EXECUTING DROP!')
        self.send_to_gcs(f'DROP NOW! ({self.detected_color})')
        
        # Send drop command
        cmd = Bool()
        cmd.data = True
        self.drop_cmd_pub.publish(cmd)
        
        # Cleanup
        if self.countdown_timer:
            self.countdown_timer.cancel()
            self.countdown_timer = None
        
        self.calculation_enabled = False
        self.calculation_done = False

def main(args=None):
    rclpy.init(args=args)
    node = DropCalculator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down Drop Calculator...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()