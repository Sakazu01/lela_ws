#!/usr/bin/env python3
# Detect red/blue tarp using ratio+HSV masks + geometry scoring.
# - Subscribe:  /camera/image_raw (sensor_msgs/Image)
# - Enable:     /detection/enable (std_msgs/Bool) → start/stop a "session"
# - Publish:    /detection/color (std_msgs/String) when session ends
# - GCS msg:    /mavros/statustext/send (mavros_msgs/StatusText)
# Tunables: hfov_deg, altitude_m, tarp_size_m, area_tol, ratio_percent, hue windows, etc.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from mavros_msgs.msg import StatusText
from cv_bridge import CvBridge
import numpy as np
import cv2
from typing import Optional, Tuple

class ColorDetectorTarp(Node):
    def __init__(self):
        super().__init__('color_detector_tarp')

        # ---------- Parameters ----------
        self.declare_parameters('', [
            # Topics
            ('image_topic', '/image_raw'),
            ('enable_topic', '/detection/enable'),
            ('color_topic', '/detection/color'),

            # Camera & scene geometry
            ('hfov_deg', 78.0),        # horizontal FOV (deg)
            ('altitude_m', 40.0),      # altitude (m); update at runtime if you can
            ('tarp_size_m', 5.0),      # tarp side (m)
            ('area_tol', 2.5),         # TIGHTENED: accept [1/area_tol .. area_tol] × expected area

            # Color/shape thresholds - SIGNIFICANTLY TIGHTENED
            ('ratio_percent', 125),    # TIGHTENED: 125 => 1.25 (was 105)
            
            # BLUE: Pure blue range only (avoid cyan/green)
            ('blue_h_min', 100),       # TIGHTENED: was 90
            ('blue_h_max', 130),       # TIGHTENED: was 150
            
            # RED: Tighter red ranges
            ('red_h1_max', 10),        # TIGHTENED: was 15
            ('red_h2_min', 170),       # TIGHTENED: was 165
            
            # Higher saturation/value for vivid colors only
            ('sat_min', 100),          # TIGHTENED: was 60
            ('val_min', 80),           # TIGHTENED: was 50
            
            ('min_area', 3000),        # INCREASED: minimum blob area (was 3000)
            ('kernel_size', 7),        # INCREASED: stronger morphology
            
            # Decision behavior
            ('decision_margin', 0.0),  # NO MARGIN: immediate response
            ('fallback_area_factor', 3.0),  # INCREASED: stricter fallback
            
            # Additional filtering
            ('min_rectangularity', 0.7),    # NEW: minimum rectangularity
            ('aspect_ratio_min', 0.7),      # NEW: tighter aspect ratio
            ('aspect_ratio_max', 1.4),      # NEW: tighter aspect ratio

            # Status logging
            ('status_log_period', 2.0),
            ('use_area_weight', True),
        ])

        image_topic  = self.get_parameter('image_topic').get_parameter_value().string_value
        enable_topic = self.get_parameter('enable_topic').get_parameter_value().string_value
        color_topic  = self.get_parameter('color_topic').get_parameter_value().string_value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, image_topic, self.image_cb, qos)
        self.enable_sub = self.create_subscription(Bool, enable_topic, self.enable_cb, 10)
        self.color_pub = self.create_publisher(String, color_topic, 10)
        self.gcs_pub = self.create_publisher(StatusText, '/mavros/statustext/send', 10)

        k = int(self.get_parameter('kernel_size').value)
        k = max(3, k | 1)  # ensure odd >= 3
        self.kernel = np.ones((k, k), np.uint8)

        self.status_period = float(self.get_parameter('status_log_period').value)

        # Session state
        self.detection_enabled: bool = False
        self.red_frames = 0
        self.blue_frames = 0
        self.red_area_sum = 0
        self.blue_area_sum = 0
        self._frames_processed = 0
        self._frames_counted = 0
        self._last_status_log = 0.0

        self.get_logger().info(
            f'✅ Tarp detector ready (INACTIVE). image="{image_topic}", enable="{enable_topic}"'
        )

    # ---------- Utilities ----------
    def send_to_gcs(self, text: str, severity: int = 6):
        m = StatusText()
        m.severity = severity  # 6=INFO
        m.text = f'COLOR: {text}'[:127]
        self.gcs_pub.publish(m)

    def _expected_pixel_area(self, frame_w: int) -> float:
        H = float(self.get_parameter('altitude_m').value)
        hfov_deg = float(self.get_parameter('hfov_deg').value)
        L = float(self.get_parameter('tarp_size_m').value)
        G = 2.0 * H * np.tan(np.deg2rad(hfov_deg / 2.0))  # ground width in view (m)
        ppm = frame_w / max(G, 1e-6)                      # pixels per meter
        return (L * ppm) ** 2

    def _best_component(self, mask: np.ndarray, frame_w: int) -> Tuple[Optional[Tuple[int,int,int,int,int]], float]:
        """Return ((x,y,w,h,area), score) for best blob passing the Aexp gate; else (None, 0.0)."""
        num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if num <= 1:
            return None, 0.0

        Aexp = self._expected_pixel_area(frame_w)
        tol = float(self.get_parameter('area_tol').value)
        Amin, Amax = Aexp / tol, Aexp * tol
        A_min_param = int(self.get_parameter('min_area').value)
        Amin = max(Amin, A_min_param)  # absolute minimum area
        
        # Get tighter aspect ratio and rectangularity limits
        aspect_min = float(self.get_parameter('aspect_ratio_min').value)
        aspect_max = float(self.get_parameter('aspect_ratio_max').value)
        min_rect = float(self.get_parameter('min_rectangularity').value)

        best = None
        best_score = -1.0
        for i in range(1, num):
            x, y, w, h, area = stats[i]
            if area < Amin or area > Amax:
                continue
            aspect = w / float(h)
            if not (aspect_min <= aspect <= aspect_max):
                continue
            rectangularity = area / float(w*h + 1e-6)  # 1.0 ~ perfect rectangle
            if rectangularity < min_rect:
                continue
            # Score: prefer close-to-expected area + rectangularity
            area_score = 1.0 - abs(area - Aexp) / max(Aexp, area)
            score = 0.7 * area_score + 0.3 * rectangularity
            if score > best_score:
                best_score = score
                best = (x, y, w, h, int(area))
        return best, float(best_score if best is not None else 0.0)

    def _reset_session(self):
        self.red_frames = 0
        self.blue_frames = 0
        self.red_area_sum = 0
        self.blue_area_sum = 0
        self._frames_processed = 0
        self._frames_counted = 0

    def _finalize_session(self):
        use_area = bool(self.get_parameter('use_area_weight').value)

        # Decide winner by area or votes
        if use_area:
            total_area = self.red_area_sum + self.blue_area_sum
            if total_area > 0:
                if self.red_area_sum > self.blue_area_sum:
                    winner, confidence = 'red', self.red_area_sum / total_area
                elif self.blue_area_sum > self.red_area_sum:
                    winner, confidence = 'blue', self.blue_area_sum / total_area
                else:
                    # tie → default to red
                    winner, confidence = 'red', 0.0
            else:
                # no counted area at all → default to red
                winner, confidence = 'red', 0.0
        else:
            total_votes = self.red_frames + self.blue_frames
            if total_votes > 0:
                if self.red_frames > self.blue_frames:
                    winner, confidence = 'red', self.red_frames / total_votes
                elif self.blue_frames > self.red_frames:
                    winner, confidence = 'blue', self.blue_frames / total_votes
                else:
                    # tie → default to red
                    winner, confidence = 'red', 0.0
            else:
                # no votes at all → default to red
                winner, confidence = 'red', 0.0

        # Publish result (always)
        out = String(); out.data = winner
        self.color_pub.publish(out)
        self.get_logger().info(
            f'✅ Session result: {winner.upper()} | conf={confidence:.2f} '
            f'| votes R/B={self.red_frames}/{self.blue_frames} '
            f'| area R/B={self.red_area_sum}/{self.blue_area_sum} '
            f'| frames processed={self._frames_processed}, counted={self._frames_counted}'
        )
        self.send_to_gcs(f'Session result: {winner.upper()} (conf={confidence:.2f})')


    # ---------- Callbacks ----------
    def enable_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().info('🔵 Detection ENABLED, starting new session')
            self._reset_session()
            self.detection_enabled = True
        else:
            if self.detection_enabled:
                self.get_logger().info('🔴 Detection DISABLED, finalizing session')
                self._finalize_session()
            self.detection_enabled = False

    def image_cb(self, msg: Image):
        self._frames_processed += 1
        if not self.detection_enabled:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_h, frame_w = frame.shape[:2]

        # -----------------------
        # === DETECTION LOGIC ===
        # (Updated to follow code2 style: tighter HSV/morphology)
        # -----------------------
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        bgr_float = frame.astype(np.float32)

        # --- BLUE ---
        blue_mask = cv2.inRange(hsv,
            (int(self.get_parameter('blue_h_min').value),
             int(self.get_parameter('sat_min').value),
             int(self.get_parameter('val_min').value)),
            (int(self.get_parameter('blue_h_max').value),
             255, 255)
        )
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, self.kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, self.kernel)
        blue_best, blue_score = self._best_component(blue_mask, frame_w)

        # --- RED ---
        red_mask1 = cv2.inRange(hsv,
            (0, int(self.get_parameter('sat_min').value), int(self.get_parameter('val_min').value)),
            (int(self.get_parameter('red_h1_max').value), 255, 255)
        )
        red_mask2 = cv2.inRange(hsv,
            (int(self.get_parameter('red_h2_min').value), int(self.get_parameter('sat_min').value), int(self.get_parameter('val_min').value)),
            (179, 255, 255)
        )
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, self.kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, self.kernel)
        red_best, red_score = self._best_component(red_mask, frame_w)

        # --- Count / sum for session ---
        counted = False
        if blue_best is not None:
            self.blue_frames += 1
            self.blue_area_sum += blue_best[4]
            counted = True
        if red_best is not None:
            self.red_frames += 1
            self.red_area_sum += red_best[4]
            counted = True
        if counted:
            self._frames_counted += 1

        # --- Optional periodic log ---
        import time
        t_now = time.time()
        if t_now - self._last_status_log > self.status_period:
            self.get_logger().info(f'Frame={self._frames_processed}, counted={self._frames_counted}, R/B={self.red_frames}/{self.blue_frames}')
            self._last_status_log = t_now


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorTarp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 KeyboardInterrupt, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
