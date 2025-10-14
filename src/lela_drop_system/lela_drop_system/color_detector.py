#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from mavros_msgs.msg import StatusText
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        
        # HSV Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('red_h_min_1', 0), ('red_s_min_1', 100), ('red_v_min_1', 100),
                ('red_h_max_1', 10), ('red_s_max_1', 255), ('red_v_max_1', 255),
                ('red_h_min_2', 170), ('red_s_min_2', 100), ('red_v_min_2', 100),
                ('red_h_max_2', 180), ('red_s_max_2', 255), ('red_v_max_2', 255),
                ('blue_h_min', 100), ('blue_s_min', 100), ('blue_v_min', 100),
                ('blue_h_max', 130), ('blue_s_max', 255), ('blue_v_max', 255),
                ('min_area', 3000),
            ]
        )
        
        self.bridge = CvBridge()
        
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos
        )
        
        # NEW: Subscribe ke enable signal dari State Manager
        self.enable_sub = self.create_subscription(
            Bool, '/detection/enable', self.enable_callback, 10
        )
        
        # Publisher
        self.color_pub = self.create_publisher(String, '/detection/color', 10)
        
        # NEW: Publish ke GCS via MAVROS
        self.gcs_pub = self.create_publisher(StatusText, '/mavros/statustext/send', 10)
        
        self.kernel = np.ones((5, 5), np.uint8)
        self.last_color = None
        self.same_color_count = 0
        self.confirm_threshold = 5
        
        # NEW: Kamera aktif/tidak aktif
        self.detection_enabled = False
        
        self.get_logger().info('✅ Color Detector ready (INACTIVE)')
    
    def enable_callback(self, msg):
        """Enable/disable deteksi berdasarkan signal dari State Manager"""
        self.detection_enabled = msg.data
        
        if self.detection_enabled:
            self.get_logger().info('🎥 Detection ENABLED')
            self.send_to_gcs('Detection started')
            # Reset counter saat mulai deteksi baru
            self.last_color = None
            self.same_color_count = 0
        else:
            self.get_logger().info('⏸️  Detection DISABLED')
    
    def send_to_gcs(self, text):
        """Kirim pesan ke GCS (Mission Planner)"""
        msg = StatusText()
        msg.severity = 6  # INFO level
        msg.text = f"COLOR: {text}"
        self.gcs_pub.publish(msg)
    
    def get_hsv_ranges(self):
        return {
            'red_low1': np.array([
                self.get_parameter('red_h_min_1').value,
                self.get_parameter('red_s_min_1').value,
                self.get_parameter('red_v_min_1').value
            ]),
            'red_up1': np.array([
                self.get_parameter('red_h_max_1').value,
                self.get_parameter('red_s_max_1').value,
                self.get_parameter('red_v_max_1').value
            ]),
            'red_low2': np.array([
                self.get_parameter('red_h_min_2').value,
                self.get_parameter('red_s_min_2').value,
                self.get_parameter('red_v_min_2').value
            ]),
            'red_up2': np.array([
                self.get_parameter('red_h_max_2').value,
                self.get_parameter('red_s_max_2').value,
                self.get_parameter('red_v_max_2').value
            ]),
            'blue_low': np.array([
                self.get_parameter('blue_h_min').value,
                self.get_parameter('blue_s_min').value,
                self.get_parameter('blue_v_min').value
            ]),
            'blue_up': np.array([
                self.get_parameter('blue_h_max').value,
                self.get_parameter('blue_s_max').value,
                self.get_parameter('blue_v_max').value
            ])
        }
    
    def image_callback(self, msg):
        # NEW: Skip jika deteksi tidak aktif
        if not self.detection_enabled:
            return
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            if frame is None or frame.size == 0:
                self.get_logger().warn('⚠️ Frame kosong, skip...')
                return
            
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            ranges = self.get_hsv_ranges()
            
            # Deteksi merah (2 range karena wraparound HSV)
            red_mask1 = cv2.inRange(hsv, ranges['red_low1'], ranges['red_up1'])
            red_mask2 = cv2.inRange(hsv, ranges['red_low2'], ranges['red_up2'])
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            red_mask = self.clean_mask(red_mask)
            
            # Deteksi biru
            blue_mask = cv2.inRange(hsv, ranges['blue_low'], ranges['blue_up'])
            blue_mask = self.clean_mask(blue_mask)
            
            # Cek area minimum
            min_area = self.get_parameter('min_area').value
            red_detected = self.has_object(red_mask, min_area)
            blue_detected = self.has_object(blue_mask, min_area)
            
            # Tentukan warna yang terdeteksi
            detected_color = None
            if red_detected and not blue_detected:
                detected_color = 'red'
            elif blue_detected and not red_detected:
                detected_color = 'blue'
            elif red_detected and blue_detected:
                # Jika keduanya terdeteksi, pilih yang area lebih besar
                red_area = cv2.countNonZero(red_mask)
                blue_area = cv2.countNonZero(blue_mask)
                detected_color = 'red' if red_area > blue_area else 'blue'
            
            # Konfirmasi: warna harus sama selama N frame
            if detected_color == self.last_color:
                self.same_color_count += 1
            else:
                self.same_color_count = 0
                self.last_color = detected_color
            
            # Publish hanya jika sudah terkonfirmasi
            if self.same_color_count == self.confirm_threshold and detected_color:
                msg = String()
                msg.data = detected_color
                self.color_pub.publish(msg)
                
                self.get_logger().info(f'🎨 Detected: {detected_color.upper()}')
                
                # NEW: Kirim ke GCS
                self.send_to_gcs(f'{detected_color.upper()} detected!')
        
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
    
    def clean_mask(self, mask):
        """Bersihkan noise dari mask"""
        mask = cv2.medianBlur(mask, 5)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)
        return mask
    
    def has_object(self, mask, min_area):
        """Cek apakah ada objek dengan area >= min_area"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) >= min_area:
                return True
        return False

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()