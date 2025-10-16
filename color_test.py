#!/usr/bin/env python3
import cv2
import numpy as np

class ColorDetectorTarp:
    def __init__(self):
        # ----- Parameter -----
        self.hfov_deg = 78.0
        self.altitude_m = 40.0
        self.tarp_size_m = 5.0
        self.area_tol = 2.5
        self.ratio_percent = 125
        self.blue_h_min, self.blue_h_max = 100, 130
        self.red_h1_max, self.red_h2_min = 10, 170
        self.sat_min, self.val_min = 100, 80
        self.min_area = 5000
        self.kernel_size = 7
        self.min_rectangularity = 0.7
        self.aspect_ratio_min, self.aspect_ratio_max = 0.7, 1.4
        self.fallback_area_factor = 3.0

        k = max(3, self.kernel_size | 1)
        self.kernel = np.ones((k, k), np.uint8)

        # Session state
        self.enabled = False
        self.reset_session()

    def reset_session(self):
        self.red_frames = 0
        self.blue_frames = 0
        self.red_area_sum = 0
        self.blue_area_sum = 0
        self.frames_processed = 0
        self.frames_counted = 0

    def expected_pixel_area(self, frame_w):
        H = self.altitude_m
        hfov_deg = self.hfov_deg
        L = self.tarp_size_m
        G = 2.0 * H * np.tan(np.deg2rad(hfov_deg / 2.0))
        ppm = frame_w / max(G, 1e-6)
        return (L * ppm) ** 2

    def best_component(self, mask, frame_w):
        num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if num <= 1:
            return None, 0.0

        Aexp = self.expected_pixel_area(frame_w)
        tol = self.area_tol
        Amin, Amax = Aexp / tol, Aexp * tol
        Amin = max(Amin, self.min_area)

        best = None
        best_score = -1.0
        for i in range(1, num):
            x, y, w, h, area = stats[i]
            if area < Amin or area > Amax:
                continue
            aspect = w / float(h)
            if not (self.aspect_ratio_min <= aspect <= self.aspect_ratio_max):
                continue
            rectangularity = area / float(w * h + 1e-6)
            if rectangularity < self.min_rectangularity:
                continue
            area_score = 1.0 - abs(area - Aexp) / max(Aexp, area)
            score = 0.7 * area_score + 0.3 * rectangularity
            if score > best_score:
                best_score = score
                best = (x, y, w, h, int(area))
        return best, best_score if best is not None else (None, 0.0)

    def process_frame(self, frame):
        B, G, R = cv2.split(frame)
        ratio_pct = self.ratio_percent
        blue_ratio = (B.astype(np.int32) * 100) > ((R + G) * ratio_pct)
        red_ratio = (R.astype(np.int32) * 100) > ((G + B) * ratio_pct)

        blue_stronger = B.astype(np.int32) > (R.astype(np.int32) * 1.3)
        red_stronger = R.astype(np.int32) > (B.astype(np.int32) * 1.3)

        blue_ratio = np.logical_and(blue_ratio, blue_stronger).astype(np.uint8) * 255
        red_ratio = np.logical_and(red_ratio, red_stronger).astype(np.uint8) * 255

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        H, S, V = cv2.split(hsv)
        sat_mask = cv2.inRange(S, self.sat_min, 255)
        val_mask = cv2.inRange(V, self.val_min, 255)
        red_h1 = cv2.inRange(H, 0, self.red_h1_max)
        red_h2 = cv2.inRange(H, self.red_h2_min, 180)
        blue_h = cv2.inRange(H, self.blue_h_min, self.blue_h_max)

        red_mask = cv2.bitwise_and(red_ratio, cv2.bitwise_and(sat_mask, val_mask))
        red_mask = cv2.bitwise_and(red_mask, cv2.bitwise_or(red_h1, red_h2))
        blue_mask = cv2.bitwise_and(blue_ratio, cv2.bitwise_and(sat_mask, val_mask))
        blue_mask = cv2.bitwise_and(blue_mask, blue_h)

        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, self.kernel, iterations=2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, self.kernel, iterations=3)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, self.kernel, iterations=2)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, self.kernel, iterations=3)

        frame_w = frame.shape[1]
        best_red, score_red = self.best_component(red_mask, frame_w)
        best_blue, score_blue = self.best_component(blue_mask, frame_w)

        fallback_factor = self.fallback_area_factor
        if best_red is None:
            red_area_total = int(np.count_nonzero(red_mask))
            if red_area_total > int(fallback_factor * self.min_area):
                x, y, w, h = cv2.boundingRect((red_mask > 0).astype(np.uint8))
                aspect = w / float(h)
                if 0.7 <= aspect <= 1.4:
                    best_red, score_red = (x, y, w, h, red_area_total), 0.05

        if best_blue is None:
            blue_area_total = int(np.count_nonzero(blue_mask))
            if blue_area_total > int(fallback_factor * self.min_area):
                x, y, w, h = cv2.boundingRect((blue_mask > 0).astype(np.uint8))
                aspect = w / float(h)
                if 0.7 <= aspect <= 1.4:
                    best_blue, score_blue = (x, y, w, h, blue_area_total), 0.05

        detected = None
        if best_red and (score_red > score_blue):
            self.red_frames += 1
            self.red_area_sum += best_red[4]
            self.frames_counted += 1
            detected = 'red'
            cv2.rectangle(frame, (best_red[0], best_red[1]), 
                          (best_red[0]+best_red[2], best_red[1]+best_red[3]), (0,0,255), 3)
        elif best_blue and (score_blue > score_red):
            self.blue_frames += 1
            self.blue_area_sum += best_blue[4]
            self.frames_counted += 1
            detected = 'blue'
            cv2.rectangle(frame, (best_blue[0], best_blue[1]), 
                          (best_blue[0]+best_blue[2], best_blue[1]+best_blue[3]), (255,0,0), 3)

        self.frames_processed += 1
        return frame, detected

    def finalize(self):
        total_area = self.red_area_sum + self.blue_area_sum
        if total_area == 0:
            print("❓ No confident detection.")
            return
        if self.red_area_sum > self.blue_area_sum:
            print(f"✅ Final result: RED ({self.red_area_sum / total_area:.2f} confidence)")
        elif self.blue_area_sum > self.red_area_sum:
            print(f"✅ Final result: BLUE ({self.blue_area_sum / total_area:.2f} confidence)")
        else:
            print("❓ Unable to decide (equal)")

def main():
    cap = cv2.VideoCapture(0)
    detector = ColorDetectorTarp()

    print("Press 'e' to enable detection, 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        key = cv2.waitKey(1) & 0xFF
        if key == ord('e'):
            detector.enabled = not detector.enabled
            detector.reset_session()
            print("Detection ENABLED" if detector.enabled else "Detection DISABLED")
        elif key == ord('q'):
            break

        if detector.enabled:
            frame, detected = detector.process_frame(frame)
            if detected:
                cv2.putText(frame, f"Detected: {detected.upper()}", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3)

        cv2.imshow("Tarp Detector", frame)

    cap.release()
    cv2.destroyAllWindows()
    detector.finalize()

if __name__ == "__main__":
    main()

