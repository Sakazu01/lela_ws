# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Bool, Int16
# from mavros_msgs.msg import StatusText
# from enum import Enum

# class SystemState(Enum):
#     IDLE = 0
#     WAITING_WAYPOINT = 1
#     DETECTING = 2
#     CALCULATING = 3
#     DROPPING = 4

# class StateManager(Node):
#     def __init__(self):
#         super().__init__('state_manager')
        
#         # Default detection window = 20s
#         self.declare_parameter('drop_waypoint', 1)
#         self.declare_parameter('detection_timeout', 20.0)
        
#         self.drop_waypoint = self.get_parameter('drop_waypoint').value
#         self.detection_timeout = self.get_parameter('detection_timeout').value
        
#         # Publishers
#         self.state_pub = self.create_publisher(String, '/system/state', 10)
#         self.gcs_pub = self.create_publisher(StatusText, '/mavros/statustext/send', 10)
#         self.detection_enable_pub = self.create_publisher(Bool, '/detection/enable', 10)
#         self.calculate_pub = self.create_publisher(Bool, '/drop/calculate', 10)
        
#         # Subscribers
#         self.wp_sub = self.create_subscription(
#             Int16, '/mission/waypoint_reached', self.waypoint_callback, 10
#         )
#         self.color_sub = self.create_subscription(
#             String, '/detection/color', self.color_callback, 10
#         )
#         self.drop_done_sub = self.create_subscription(
#             Bool, '/drop/completed', self.drop_done_callback, 10
#         )
        
#         self.current_state = SystemState.IDLE
#         self.current_waypoint = -1
#         self.timeout_timer = None
#         self.detected_color = None
        
#         self.get_logger().info(f'✅ State Manager ready (Drop WP: {self.drop_waypoint}, detect_window={self.detection_timeout:.1f}s)')
#         self.transition_to(SystemState.WAITING_WAYPOINT)
    
#     def send_to_gcs(self, text):
#         msg = StatusText()
#         msg.severity = 6
#         msg.text = f"LELA: {text}"[:127]
#         self.gcs_pub.publish(msg)
#         self.get_logger().info(f'📡 GCS: {text}')
    
#     def enable_detection(self, enable: bool):
#         msg = Bool()
#         msg.data = enable
#         self.detection_enable_pub.publish(msg)
#         self.get_logger().info(f'🎥 detection/enable={enable}')
    
#     def trigger_calculation(self):
#         """Kirim sinyal ke Drop Calculator untuk mulai hitung"""
#         msg = Bool()
#         msg.data = True
#         self.calculate_pub.publish(msg)
#         self.get_logger().info('📊 Triggering Drop Calculator...')

#     def transition_to(self, new_state: SystemState):
#         if self.timeout_timer:
#             self.timeout_timer.cancel()
#             self.timeout_timer = None
        
#         old = self.current_state.name
#         self.current_state = new_state
        
#         self.get_logger().info(f'🔄 State: {old} → {new_state.name}')
        
#         msg = String()
#         msg.data = new_state.name
#         self.state_pub.publish(msg)
        
#         if new_state == SystemState.WAITING_WAYPOINT:
#             self.current_waypoint = -1
#             self.detected_color = None
#             self.enable_detection(False)
#             self.get_logger().info('⏳ Waiting for next waypoint...')
        
#         elif new_state == SystemState.DETECTING:
#             # Start detection window; we will stop at timeout regardless of color arrival
#             self.detected_color = None  # clear previous
#             self.enable_detection(True)
#             self.send_to_gcs(f"WP{self.current_waypoint}: Detecting for {self.detection_timeout:.0f}s...")
#             self.timeout_timer = self.create_timer(
#                 float(self.detection_timeout),
#                 self.on_detection_timeout
#             )
        
#         elif new_state == SystemState.CALCULATING:
#             # Ensure camera/detector off when calculating
#             self.enable_detection(False)
#             self.send_to_gcs(f"WP{self.current_waypoint}: Calculating drop...")
#             # Always trigger calculation on entering CALCULATING
#             self.trigger_calculation()
    
#     def on_detection_timeout(self):
#         # One-shot timer; clear reference
#         if self.timeout_timer:
#             self.timeout_timer.cancel()
#             self.timeout_timer = None

#         if self.current_state != SystemState.DETECTING:
#             return

#         self.get_logger().warn(f'⏱️ Detection window ended at WP{self.current_waypoint}')
#         # Turn camera off first — detector may publish final color on disable
#         self.enable_detection(False)

#         # If this is the drop waypoint, proceed regardless of whether color arrived
#         if self.current_waypoint == self.drop_waypoint:
#             self.send_to_gcs(f"WP{self.current_waypoint}: Window ended → continue to CALCULATING")
#             self.transition_to(SystemState.CALCULATING)
#         else:
#             # Non-drop waypoint → just continue mission
#             self.send_to_gcs(f"WP{self.current_waypoint}: Window ended (non-drop WP) → continue")
#             self.transition_to(SystemState.WAITING_WAYPOINT)
    
#     def waypoint_callback(self, msg: Int16):
#         if self.current_state != SystemState.WAITING_WAYPOINT:
#             return
        
#         self.current_waypoint = int(msg.data)
#         self.get_logger().info(f'📍 Reached WP{self.current_waypoint} (drop_wp={self.drop_waypoint})')
#         self.send_to_gcs(f"WP{self.current_waypoint} reached")
        
#         # Immediately open detection window
#         self.transition_to(SystemState.DETECTING)
    
#     def color_callback(self, msg: String):
#         # Accept color while DETECTING or CALCULATING (some detectors publish on disable)
#         if self.current_state not in (SystemState.DETECTING, SystemState.CALCULATING):
#             return
        
#         color = msg.data.upper().strip()
#         if color not in ("RED", "BLUE"):
#             self.get_logger().info(f'🎨 Ignored color: "{msg.data}"')
#             return

#         self.detected_color = color
#         self.get_logger().info(f'🎨 Detected: {color} at WP{self.current_waypoint} (state={self.current_state.name})')
#         self.send_to_gcs(f"WP{self.current_waypoint}: {color} detected")

#         # If still DETECTING and this is the drop waypoint, proceed immediately.
#         if self.current_state == SystemState.DETECTING and self.current_waypoint == self.drop_waypoint:
#             self.transition_to(SystemState.CALCULATING)
#         elif self.current_state == SystemState.DETECTING:
#             # Non-drop waypoint: finish window immediately and move on
#             self.transition_to(SystemState.WAITING_WAYPOINT)
    
#     def drop_done_callback(self, msg: Bool):
#         if not bool(msg.data):
#             return
        
#         # Confirm from Servo Controller
#         if self.current_state in (SystemState.CALCULATING, SystemState.DROPPING):
#             self.get_logger().info('✅ Drop completed successfully')
#             color_txt = self.detected_color if self.detected_color else "UNKNOWN"
#             self.send_to_gcs(f"WP{self.current_waypoint}: Drop successful ({color_txt})")
#             self.transition_to(SystemState.WAITING_WAYPOINT)

# def main(args=None):
#     rclpy.init(args=args)
#     node = StateManager()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('🛑 Shutting down...')
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


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
        
        # Hard-coded drop waypoints (can be parameterized later)
        self.drop_waypoints = [3, 6, 9]  # 🧩 Example list of drop waypoints
        
        # Default detection window = 20s
        self.declare_parameter('detection_timeout', 20.0)
        self.detection_timeout = self.get_parameter('detection_timeout').value
        
        # Publishers
        self.state_pub = self.create_publisher(String, '/system/state', 10)
        self.gcs_pub = self.create_publisher(StatusText, '/mavros/statustext/send', 10)
        self.detection_enable_pub = self.create_publisher(Bool, '/detection/enable', 10)
        self.calculate_pub = self.create_publisher(Bool, '/drop/calculate', 10)
        
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
        self.is_drop_waypoint = False
        
        self.get_logger().info(f'✅ State Manager ready (Drop WPs: {self.drop_waypoints}, detect_window={self.detection_timeout:.1f}s)')
        self.transition_to(SystemState.WAITING_WAYPOINT)
    
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

    def transition_to(self, new_state: SystemState):
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
            self.detected_color = None
            self.enable_detection(True)
            self.send_to_gcs(f"WP{self.current_waypoint}: Detecting for {self.detection_timeout:.0f}s...")
            self.timeout_timer = self.create_timer(
                float(self.detection_timeout),
                self.on_detection_timeout
            )
        
        elif new_state == SystemState.CALCULATING:
            self.enable_detection(False)
            self.send_to_gcs(f"WP{self.current_waypoint}: Calculating drop...")
            self.trigger_calculation()
    
    def on_detection_timeout(self):
        if self.timeout_timer:
            self.timeout_timer.cancel()
            self.timeout_timer = None

        if self.current_state != SystemState.DETECTING:
            return

        self.get_logger().warn(f'⏱️ Detection window ended at WP{self.current_waypoint}')
        self.enable_detection(False)

        if self.is_drop_waypoint:
            self.send_to_gcs(f"WP{self.current_waypoint}: Detection timeout → proceed to CALCULATING")
            self.transition_to(SystemState.CALCULATING)
        else:
            self.send_to_gcs(f"WP{self.current_waypoint}: Non-drop WP → continue mission")
            self.transition_to(SystemState.WAITING_WAYPOINT)
    
    def waypoint_callback(self, msg: Int16):
        if self.current_state != SystemState.WAITING_WAYPOINT:
            return
        
        self.current_waypoint = int(msg.data)
        self.is_drop_waypoint = self.current_waypoint in self.drop_waypoints
        
        self.get_logger().info(f'📍 Reached WP{self.current_waypoint} (drop_wp_list={self.drop_waypoints})')
        self.send_to_gcs(f"WP{self.current_waypoint} reached")
        
        # Immediately open detection window for all WPs
        self.transition_to(SystemState.DETECTING)
    
    def color_callback(self, msg: String):
        if self.current_state not in (SystemState.DETECTING, SystemState.CALCULATING):
            return
        
        color = msg.data.upper().strip()
        if color not in ("RED", "BLUE"):
            self.get_logger().info(f'🎨 Ignored color: "{msg.data}"')
            return

        self.detected_color = color
        self.get_logger().info(f'🎨 Detected: {color} at WP{self.current_waypoint} (state={self.current_state.name})')
        self.send_to_gcs(f"WP{self.current_waypoint}: {color} detected")

        if self.current_state == SystemState.DETECTING:
            if self.is_drop_waypoint:
                self.transition_to(SystemState.CALCULATING)
            else:
                self.transition_to(SystemState.WAITING_WAYPOINT)
    
    def drop_done_callback(self, msg: Bool):
        if not bool(msg.data):
            return
        
        if self.current_state in (SystemState.CALCULATING, SystemState.DROPPING):
            self.get_logger().info('✅ Drop completed successfully')
            color_txt = self.detected_color if self.detected_color else "UNKNOWN"
            self.send_to_gcs(f"WP{self.current_waypoint}: Drop successful ({color_txt})")
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
