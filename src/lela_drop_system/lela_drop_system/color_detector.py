




#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from mavros_msgs.msg import StatusText
from cv_bridge import CvBridge
import numpy as np
import cv2
import time
from collections import deque

class SimpleColorDetector:
    def __init__(self):
        # Fixed HSV ranges (adjusted for low saturation camera)
        self.red1_lower = np.array([0, 50, 80])     # Lowered S: 100→50, V: 100→80
        self.red1_upper = np.array([10, 255, 255])
        self.red2_lower = np.array([170, 50, 80])   # Lowered S: 100→50, V: 100→80
        self.red2_upper = np.array([180, 255, 255])
        
        self.blue_lower = np.array([90, 50, 80])    # Lowered S: 100→50, V: 100→80
        self.blue_upper = np.array([130, 255, 255])
        
        self.min_area = 500
        self.blur_size = 5
        self.kernel_size = 5
        
        # Smoothing
        self.position_history = deque(maxlen=5)
        self.color_history = deque(maxlen=3)

    def detect_colors(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blur = cv2.GaussianBlur(hsv, (self.blur_size, self.blur_size), 0)

        # Red mask (2 ranges)
        red_mask1 = cv2.inRange(blur, self.red1_lower, self.red1_upper)
        red_mask2 = cv2.inRange(blur, self.red2_lower, self.red2_upper)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        
        # Blue mask
        blue_mask = cv2.inRange(blur, self.blue_lower, self.blue_upper)

        # Morphological operations
        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter by area
        red_areas = [(cv2.contourArea(c), c) for c in red_contours if cv2.contourArea(c) > self.min_area]
        blue_areas = [(cv2.contourArea(c), c) for c in blue_contours if cv2.contourArea(c) > self.min_area]

        red_areas.sort(key=lambda x: x[0], reverse=True)
        blue_areas.sort(key=lambda x: x[0], reverse=True)

        return red_areas, blue_areas

    def smooth_position(self, position):
        if position is not None:
            self.position_history.append(position)
            if len(self.position_history) >= 3:
                return int(np.mean(self.position_history))
        return position

    def smooth_color(self, color):
        self.color_history.append(color)
        if len(self.color_history) >= 2:
            colors = list(self.color_history)
            return max(set(colors), key=colors.count)
        return color

    def process_frame(self, frame):
        red_areas, blue_areas = self.detect_colors(frame)
        frame_center_x = frame.shape[1] // 2

        color_detected = "None"
        area_detected = 0
        position = None
        direction = "CENTER"

        # Priority: RED first, then BLUE
        if red_areas:
            area, cnt = red_areas[0]
            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w // 2
            color_detected = "RED"
            area_detected = area
            position = cx
            
        elif blue_areas:
            area, cnt = blue_areas[0]
            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w // 2
            color_detected = "BLUE"
            area_detected = area
            position = cx

        # Apply smoothing
        position = self.smooth_position(position)
        color_detected = self.smooth_color(color_detected)
        
        # Determine direction
        if position is not None:
            threshold = 50
            if position < frame_center_x - threshold:
                direction = "LEFT"
            elif position > frame_center_x + threshold:
                direction = "RIGHT"
            else:
                direction = "CENTER"

        return color_detected, area_detected, position, direction


class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_simple')
        
        # Parameters
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('enable_topic', '/detection/enable')
        self.declare_parameter('color_topic', '/detection/color')
        self.declare_parameter('status_log_period', 2.0)
        self.declare_parameter('fallback_color', 'RED')  # Default color if nothing detected
        
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        enable_topic = self.get_parameter('enable_topic').get_parameter_value().string_value
        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        
        # QoS for camera
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        self.detector = SimpleColorDetector()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos
        )
        self.enable_sub = self.create_subscription(
            Bool, enable_topic, self.enable_callback, 10
        )
        
        # Publishers
        self.color_pub = self.create_publisher(String, color_topic, 10)
        self.gcs_pub = self.create_publisher(StatusText, '/mavros/statustext/send', 10)
        
        # State
        self.detection_enabled = False
        self.red_frames = 0
        self.blue_frames = 0
        self.red_area_sum = 0
        self.blue_area_sum = 0
        self.frames_processed = 0
        self.last_status_log = 0.0
        self.status_period = self.get_parameter('status_log_period').value
        
        self.get_logger().info(f'✅ Simple Color Detector ready (INACTIVE)')
        self.get_logger().info(f'   Image: {image_topic}')
        self.get_logger().info(f'   Enable: {enable_topic}')
        self.get_logger().info(f'   Color: {color_topic}')
    
    def send_to_gcs(self, text: str, severity: int = 6):
        msg = StatusText()
        msg.severity = severity
        msg.text = f'COLOR: {text}'[:127]
        self.gcs_pub.publish(msg)
    
    def reset_session(self):
        self.red_frames = 0
        self.blue_frames = 0
        self.red_area_sum = 0
        self.blue_area_sum = 0
        self.frames_processed = 0
        self.detector.position_history.clear()
        self.detector.color_history.clear()
    
    def finalize_session(self):
        """Finalize detection session and publish result"""
        winner = None
        confidence = 0.0
        
        # Use area-weighted voting
        total_area = self.red_area_sum + self.blue_area_sum
        if total_area > 0:
            if self.red_area_sum > self.blue_area_sum:
                winner = 'RED'
                confidence = self.red_area_sum / total_area
            elif self.blue_area_sum > self.red_area_sum:
                winner = 'BLUE'
                confidence = self.blue_area_sum / total_area
        else:
            # Fallback to frame count
            total_votes = self.red_frames + self.blue_frames
            if total_votes > 0:
                if self.red_frames > self.blue_frames:
                    winner = 'RED'
                    confidence = self.red_frames / total_votes
                elif self.blue_frames > self.red_frames:
                    winner = 'BLUE'
                    confidence = self.blue_frames / total_votes
        
        # FALLBACK: If no color detected, use default
        if winner is None:
            winner = self.get_parameter('fallback_color').get_parameter_value().string_value
            confidence = 0.0
            self.get_logger().warn(
                f'⚠️  No color detected! Using fallback: {winner} | '
                f'frames={self.frames_processed}'
            )
            self.send_to_gcs(f'No detection! Fallback: {winner}')
        
        # Always publish result (even if fallback)
        msg = String()
        msg.data = winner
        self.color_pub.publish(msg)
        
        if confidence > 0.0:
            self.get_logger().info(
                f'✅ Session result: {winner} | conf={confidence:.2f} | '
                f'votes R/B={self.red_frames}/{self.blue_frames} | '
                f'area R/B={self.red_area_sum}/{self.blue_area_sum} | '
                f'frames={self.frames_processed}'
            )
            self.send_to_gcs(f'Result: {winner} (conf={confidence:.2f})')
        else:
            self.get_logger().info(
                f'⚠️  Fallback result: {winner} | frames={self.frames_processed}'
            )
    
    def enable_callback(self, msg: Bool):
        enable = msg.data
        if enable == self.detection_enabled:
            return
        
        self.detection_enabled = enable
        if enable:
            self.reset_session()
            self.last_status_log = 0.0
            self.get_logger().info('🎥 Detection ENABLED (session started)')
            self.send_to_gcs('Detection started')
        else:
            self.get_logger().info('⏸️  Detection DISABLED → finalizing...')
            self.finalize_session()
    
    def image_callback(self, msg: Image):
        if not self.detection_enabled:
            return
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if frame is None or frame.size == 0:
                return
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}')
            return
        
        # Process frame
        color, area, pos, direction = self.detector.process_frame(frame)
        
        # Count detections
        if color == "RED":
            self.red_frames += 1
            self.red_area_sum += area
        elif color == "BLUE":
            self.blue_frames += 1
            self.blue_area_sum += area
        
        self.frames_processed += 1
        
        # Periodic status logging
        if self.status_period > 0.0:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_status_log >= self.status_period:
                self.last_status_log = now
                self.get_logger().info(
                    f'[Session] frames={self.frames_processed} | '
                    f'last={color} (area={area}) | '
                    f'votes R/B={self.red_frames}/{self.blue_frames} | '
                    f'area R/B={self.red_area_sum}/{self.blue_area_sum}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()