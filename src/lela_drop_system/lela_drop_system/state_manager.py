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

        # Hanya deteksi di WP 2, 4, 6, 8
        self.detect_waypoints = [2, 4, 6, 8]
        # Drop hanya dilakukan di WP 1
        self.drop_waypoint = 1

        # Waktu maksimal deteksi (detik)
        self.declare_parameter('detection_timeout', 20.0)
        self.detection_timeout = self.get_parameter('detection_timeout').value

        # Publishers
        self.state_pub = self.create_publisher(String, '/system/state', 10)
        self.gcs_pub = self.create_publisher(StatusText, '/mavros/statustext/send', 10)
        self.detection_enable_pub = self.create_publisher(Bool, '/detection/enable', 10)
        self.calculate_pub = self.create_publisher(Bool, '/drop/calculate', 10)

        # Subscribers
        self.wp_sub = self.create_subscription(Int16, '/mission/waypoint_reached', self.waypoint_callback, 10)
        self.color_sub = self.create_subscription(String, '/detection/color', self.color_callback, 10)
        self.drop_done_sub = self.create_subscription(Bool, '/drop/completed', self.drop_done_callback, 10)

        # Internal state
        self.current_state = SystemState.IDLE
        self.current_waypoint = -1
        self.timeout_timer = None
        self.detected_colors = {}  # simpan hasil deteksi dari WP2/4/6/8
        self.get_logger().info(f'✅ State Manager ready (Detect WPs={self.detect_waypoints}, Drop WP={self.drop_waypoint}, Timeout={self.detection_timeout:.1f}s)')

        self.transition_to(SystemState.WAITING_WAYPOINT)

    # --- Utilitas dasar ---
    def send_to_gcs(self, text):
        msg = StatusText()
        msg.severity = 6
        msg.text = f"LELA: {text}"[:127]
        self.gcs_pub.publish(msg)
        self.get_logger().info(f'📡 GCS: {text}')

    def enable_detection(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self.detection_enable_pub.publish(msg)
        self.get_logger().info(f'🎥 detection/enable={enable}')

    def trigger_calculation(self):
        msg = Bool()
        msg.data = True
        self.calculate_pub.publish(msg)
        self.get_logger().info('📊 Triggering Drop Calculator...')

    # --- Transisi antar state ---
    def transition_to(self, new_state: SystemState):
        if self.timeout_timer:
            self.timeout_timer.cancel()
            self.timeout_timer = None

        old_state = self.current_state.name
        self.current_state = new_state
        self.get_logger().info(f'🔄 State: {old_state} → {new_state.name}')

        msg = String()
        msg.data = new_state.name
        self.state_pub.publish(msg)

        if new_state == SystemState.WAITING_WAYPOINT:
            self.enable_detection(False)
            self.get_logger().info('⏳ Waiting for next waypoint...')

        elif new_state == SystemState.DETECTING:
            self.enable_detection(True)
            self.send_to_gcs(f"WP{self.current_waypoint}: Detecting color ({self.detection_timeout:.0f}s window)")
            self.timeout_timer = self.create_timer(self.detection_timeout, self.on_detection_timeout)

        elif new_state == SystemState.CALCULATING:
            self.enable_detection(False)
            self.send_to_gcs(f"WP{self.current_waypoint}: Calculating drop point using previous detections...")
            self.trigger_calculation()

    # --- Event Handler ---
    def waypoint_callback(self, msg: Int16):
        if self.current_state != SystemState.WAITING_WAYPOINT:
            return

        self.current_waypoint = int(msg.data)
        self.send_to_gcs(f"Reached WP{self.current_waypoint}")

        # Deteksi hanya di WP 2,4,6,8
        if self.current_waypoint in self.detect_waypoints:
            self.transition_to(SystemState.DETECTING)

        # Drop hanya di WP 1
        elif self.current_waypoint == self.drop_waypoint:
            self.transition_to(SystemState.CALCULATING)

        else:
            self.send_to_gcs(f"WP{self.current_waypoint}: No action (passing through)")
            self.transition_to(SystemState.WAITING_WAYPOINT)

    def color_callback(self, msg: String):
        if self.current_state != SystemState.DETECTING:
            return

        color = msg.data.strip().upper()
        if color not in ("RED", "BLUE"):
            return

        self.detected_colors[self.current_waypoint] = color
        self.get_logger().info(f'🎨 Detected {color} at WP{self.current_waypoint}')
        self.send_to_gcs(f"WP{self.current_waypoint}: {color} detected")

        # Setelah warna terdeteksi, langsung lanjut ke waypoint berikutnya
        self.transition_to(SystemState.WAITING_WAYPOINT)

    def on_detection_timeout(self):
        if self.timeout_timer:
            self.timeout_timer.cancel()
            self.timeout_timer = None

        if self.current_state != SystemState.DETECTING:
            return

        self.get_logger().warn(f'⏱️ Detection timeout at WP{self.current_waypoint}')
        self.enable_detection(False)
        self.send_to_gcs(f"WP{self.current_waypoint}: Detection timeout, continue mission")
        self.transition_to(SystemState.WAITING_WAYPOINT)

    def drop_done_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.current_state in (SystemState.CALCULATING, SystemState.DROPPING):
            self.get_logger().info('✅ Drop completed successfully')
            self.send_to_gcs(f"WP{self.current_waypoint}: Drop completed")
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
