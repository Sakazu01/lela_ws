#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int16
from mavros_msgs.msg import StatusText
from enum import Enum

class SystemState(Enum):
    IDLE = 0
    WAITING_WAYPOINT = 1
    DETECTING = 2
    CALCULATING = 3
    DROPPING = 4

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')
        
        self.declare_parameter('drop_waypoint', 1)
        self.declare_parameter('detection_timeout', 30.0)
        
        self.drop_waypoint = self.get_parameter('drop_waypoint').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        
        # Publishers
        self.state_pub = self.create_publisher(String, '/system/state', 10)
        self.gcs_pub = self.create_publisher(StatusText, '/mavros/statustext/send', 10)
        self.detection_enable_pub = self.create_publisher(Bool, '/detection/enable', 10)
        self.calculate_pub = self.create_publisher(Bool, '/drop/calculate', 10)  # NEW
        
        # Subscribers
        self.wp_sub = self.create_subscription(
            Int16, '/mission/waypoint_reached', self.waypoint_callback, 10
        )
        self.color_sub = self.create_subscription(
            String, '/detection/color', self.color_callback, 10
        )
        self.drop_done_sub = self.create_subscription(
            Bool, '/drop/completed', self.drop_done_callback, 10
        )
        
        self.current_state = SystemState.IDLE
        self.current_waypoint = -1
        self.timeout_timer = None
        self.detected_color = None
        
        self.get_logger().info(f'✅ State Manager ready (Drop WP: {self.drop_waypoint})')
        self.transition_to(SystemState.WAITING_WAYPOINT)
    
    def send_to_gcs(self, text):
        msg = StatusText()
        msg.severity = 6
        msg.text = f"LELA: {text}"
        self.gcs_pub.publish(msg)
        self.get_logger().info(f'📡 GCS: {text}')
    
    def enable_detection(self, enable):
        msg = Bool()
        msg.data = enable
        self.detection_enable_pub.publish(msg)
    
    def trigger_calculation(self):
        """Kirim sinyal ke Drop Calculator untuk mulai hitung"""
        msg = Bool()
        msg.data = True
        self.calculate_pub.publish(msg)
        self.get_logger().info('📊 Triggering Drop Calculator...')
    
    def transition_to(self, new_state):
        if self.timeout_timer:
            self.timeout_timer.cancel()
            self.timeout_timer = None
        
        old = self.current_state.name
        self.current_state = new_state
        
        self.get_logger().info(f'🔄 State: {old} → {new_state.name}')
        
        msg = String()
        msg.data = new_state.name
        self.state_pub.publish(msg)
        
        if new_state == SystemState.WAITING_WAYPOINT:
            self.current_waypoint = -1
            self.detected_color = None
            self.enable_detection(False)
            self.get_logger().info('⏳ Waiting for next waypoint...')
        
        elif new_state == SystemState.DETECTING:
            self.enable_detection(True)
            self.send_to_gcs(f"WP{self.current_waypoint}: Detecting...")
            self.timeout_timer = self.create_timer(
                self.detection_timeout, 
                self.on_detection_timeout
            )
        
        elif new_state == SystemState.CALCULATING:
            self.enable_detection(False)  # Matikan kamera saat hitung
            self.send_to_gcs(f"WP{self.current_waypoint}: Calculating drop...")
    
    def on_detection_timeout(self):
        if self.current_state == SystemState.DETECTING:
            self.get_logger().warn(f'⏱️ Detection timeout at WP{self.current_waypoint}')
            self.send_to_gcs(f"WP{self.current_waypoint}: No target (timeout)")
            
            # Kembali ke waiting (tidak force drop)
            self.transition_to(SystemState.WAITING_WAYPOINT)
    
    def waypoint_callback(self, msg):
        if self.current_state != SystemState.WAITING_WAYPOINT:
            return
        
        self.current_waypoint = msg.data
        self.get_logger().info(f'📍 Reached WP{self.current_waypoint}')
        self.send_to_gcs(f"WP{self.current_waypoint} reached")
        
        # Langsung mulai deteksi
        self.transition_to(SystemState.DETECTING)
    
    def color_callback(self, msg):
        if self.current_state != SystemState.DETECTING:
            return
        
        color = msg.data.upper()
        self.detected_color = color
        
        self.get_logger().info(f'🎨 Detected: {color} at WP{self.current_waypoint}')
        self.send_to_gcs(f"WP{self.current_waypoint}: {color} detected")
        
        if self.current_waypoint == self.drop_waypoint:
            # Transisi ke CALCULATING dan kirim sinyal ke Drop Calculator
            self.transition_to(SystemState.CALCULATING)
            self.trigger_calculation()
        else:
            self.get_logger().info(f'ℹ️ WP{self.current_waypoint} is not drop point')
            self.transition_to(SystemState.WAITING_WAYPOINT)
    
    def drop_done_callback(self, msg):
        if not msg.data:
            return
        
        # Terima konfirmasi dari Servo Controller
        if self.current_state == SystemState.CALCULATING or self.current_state == SystemState.DROPPING:
            self.get_logger().info('✅ Drop completed successfully')
            self.send_to_gcs(f"WP{self.current_waypoint}: Drop successful ({self.detected_color})")
            self.transition_to(SystemState.WAITING_WAYPOINT)

def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()