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
            
            ('min_area', 5000),        # INCREASED: minimum blob area (was 3000)
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
        enable = bool(msg.data)
        if enable == self.detection_enabled:
            return
        self.detection_enabled = enable
        if enable:
            self._reset_session()
            self._last_status_log = 0.0
            self.get_logger().info('🎥 Detection ENABLED (session started)')
            self.send_to_gcs('Detection started')
        else:
            self.get_logger().info('⏸️ Detection DISABLED → finalizing…')
            self._finalize_session()

    def image_cb(self, msg: Image):
        if not self.detection_enabled:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if frame is None or frame.size == 0:
                return
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}')
            return

        # --- TIGHTENED masks: stricter ratio + HSV ---
        ratio_pct = int(self.get_parameter('ratio_percent').value)   # now 125
        sat_min   = int(self.get_parameter('sat_min').value)         # now 100
        val_min   = int(self.get_parameter('val_min').value)         # now 80
        blue_hmin = int(self.get_parameter('blue_h_min').value)      # now 100
        blue_hmax = int(self.get_parameter('blue_h_max').value)      # now 130
        red_h1max = int(self.get_parameter('red_h1_max').value)      # now 10
        red_h2min = int(self.get_parameter('red_h2_min').value)      # now 170

        B, G, R = cv2.split(frame)
        
        # Stronger color dominance required
        blue_ratio = (B.astype(np.int32) * 100) > ((R.astype(np.int32) + G.astype(np.int32)) * ratio_pct)
        red_ratio  = (R.astype(np.int32) * 100) > ((G.astype(np.int32) + B.astype(np.int32)) * ratio_pct)
        
        # Additional filter: Blue must be significantly stronger than Red, and vice versa
        blue_stronger = B.astype(np.int32) > (R.astype(np.int32) * 1.3)  # B > 1.3*R
        red_stronger = R.astype(np.int32) > (B.astype(np.int32) * 1.3)   # R > 1.3*B
        
        blue_ratio = np.logical_and(blue_ratio, blue_stronger).astype(np.uint8) * 255
        red_ratio = np.logical_and(red_ratio, red_stronger).astype(np.uint8) * 255

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        H, S, V = cv2.split(hsv)
        sat_mask = cv2.inRange(S, sat_min, 255)
        val_mask = cv2.inRange(V, val_min, 255)

        # Tighter hue ranges
        red_h1 = cv2.inRange(H, 0, red_h1max)
        red_h2 = cv2.inRange(H, red_h2min, 180)
        blue_h = cv2.inRange(H, blue_hmin, blue_hmax)

        # Combine all conditions
        red_mask  = cv2.bitwise_and(red_ratio,  cv2.bitwise_and(sat_mask, val_mask))
        red_mask  = cv2.bitwise_and(red_mask,   cv2.bitwise_or(red_h1, red_h2))
        blue_mask = cv2.bitwise_and(blue_ratio, cv2.bitwise_and(sat_mask, val_mask))
        blue_mask = cv2.bitwise_and(blue_mask,  blue_h)

        # Stronger morphology for cleaner blobs
        red_mask  = cv2.morphologyEx(red_mask,  cv2.MORPH_OPEN,  self.kernel, iterations=2)
        red_mask  = cv2.morphologyEx(red_mask,  cv2.MORPH_CLOSE, self.kernel, iterations=3)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN,  self.kernel, iterations=2)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, self.kernel, iterations=3)

        # --- pick best candidate per color ---
        frame_w = frame.shape[1]
        best_red,  score_red  = self._best_component(red_mask,  frame_w)
        best_blue, score_blue = self._best_component(blue_mask, frame_w)

        # Stricter fallback
        fallback_factor = float(self.get_parameter('fallback_area_factor').value)
        min_area = int(self.get_parameter('min_area').value)
        if best_red is None:
            red_area_total = int(np.count_nonzero(red_mask))
            if red_area_total > int(fallback_factor * min_area):
                x, y, w, h = cv2.boundingRect((red_mask > 0).astype(np.uint8))
                # Check aspect ratio even in fallback
                aspect = w / float(h) if h > 0 else 0
                if 0.7 <= aspect <= 1.4:
                    best_red, score_red = (x, y, w, h, red_area_total), 0.05  # Lower score for fallback
                    
        if best_blue is None:
            blue_area_total = int(np.count_nonzero(blue_mask))
            if blue_area_total > int(fallback_factor * min_area):
                x, y, w, h = cv2.boundingRect((blue_mask > 0).astype(np.uint8))
                aspect = w / float(h) if h > 0 else 0
                if 0.7 <= aspect <= 1.4:
                    best_blue, score_blue = (x, y, w, h, blue_area_total), 0.05

        # Decision without margin - highest score wins immediately
        margin = float(self.get_parameter('decision_margin').value)  # 0.0 - no margin
        detected = None
        if best_red and (score_red >= score_blue * (1.0 + margin)):
            self.red_frames += 1
            self.red_area_sum += best_red[4]
            self._frames_counted += 1
            detected = 'red'
        elif best_blue and (score_blue >= score_red * (1.0 + margin)):
            self.blue_frames += 1
            self.blue_area_sum += best_blue[4]
            self._frames_counted += 1
            detected = 'blue'
        # else: neither confident → ignore this frame

        self._frames_processed += 1

        # Periodic status
        if self.status_period > 0.0:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_status_log >= self.status_period:
                self._last_status_log = now
                self.get_logger().info(
                    f'[Session] processed={self._frames_processed}, counted={self._frames_counted} '
                    f'| last={detected or "ignored"} | votes R/B={self.red_frames}/{self.blue_frames} '
                    f'| area R/B={self.red_area_sum}/{self.blue_area_sum}'
                )

def main():
    rclpy.init()
    node = ColorDetectorTarp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down…')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
