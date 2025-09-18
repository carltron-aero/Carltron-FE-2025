#!/usr/bin/env python3
import threading
import time
import math
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2

class ScanImageAnalyzer(Node):
    def __init__(self):
        super().__init__('scan_image_analyzer')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sector_min_deg',   330.0),
                ('sector_max_deg',   30.0),
                ('max_range',        0.3),
                ('cluster_thresh',   0.18),
                ('min_diameter',     0.02),
                ('max_diameter',     0.08),
                ('brightness_thresh',90.0),
            ]
        )

        # Camera setup: 90 FPS, 1900x1200
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1900)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FPS, 90)
        # Disable auto exposure/white balance if supported
        try:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            # Additional camera-specific settings may be needed
        except:
            pass
        if not self.cap.isOpened():
            self.get_logger().fatal("Cannot open camera at /dev/video0")
            rclpy.shutdown()
            return

        # Frame buffer
        self.buffer = deque(maxlen=200)
        self.buffer_lock = threading.Lock()
        self._warned_empty = False
        threading.Thread(target=self._reader_thread, daemon=True).start()

        # Subscribe to LaserScan
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # Window
        cv2.namedWindow("Frame with Sample Overlay", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Frame with Sample Overlay", 1200, 600)

        self.get_logger().info('Scan-image analyzer started.')

    def _reader_thread(self):
        while rclpy.ok() and self.cap.isOpened():
            if self.cap.grab():
                ts = time.time()
                ret, frame = self.cap.retrieve()
                if ret:
                    with self.buffer_lock:
                        self.buffer.append((ts, frame.copy()))
            else:
                time.sleep(0.005)

    def _get_frame(self, target_ts):
        with self.buffer_lock:
            buf = list(self.buffer)
        if not buf:
            return None
        for ts, frame in reversed(buf):
            if ts <= target_ts:
                return frame
        return buf[-1][1]

    def _compute_sampling_circle(self, frame_shape):
        h_f, w_f = frame_shape[:2]
        cx = w_f // 2 + 7 + 32
        cy = h_f // 2 + 20
        outer = min(cx, cy) - 26
        inner = outer - 3
        r_mid_orig = (outer + inner) / 2.0
        r_mid = int(r_mid_orig * 0.75)
        return cx, cy, r_mid

    def _angle_to_image_theta(self, scan_rad):
        # Convert scan angle (rad) to image angle: mirror and rotate to align
        deg = math.degrees(scan_rad) % 360.0
        img_deg = (270.0 + deg) % 360.0
        return math.radians(img_deg)

    def _circular_mean_hue(self, hs):
        # hs: array of H values in [0,179]
        # Convert to radians [0,2pi]
        rad = hs.astype(np.float32) * 2.0 * math.pi / 180.0
        x = np.cos(rad)
        y = np.sin(rad)
        mx = np.mean(x)
        my = np.mean(y)
        mean_ang = math.atan2(my, mx)
        if mean_ang < 0:
            mean_ang += 2 * math.pi
        # Convert back to [0,179]
        mean_deg = math.degrees(mean_ang)
        return mean_deg * 0.5

    def _sample_hsv_half_range(self, frame, ang_min_rad, ang_max_rad):
        """
        Sample HSV over 20% of the cluster angular range around mid-angle.
        Ensure minimum 3x3 pixels.
        Returns mean_hsv (with circular hue mean), sample_coords.
        """
        h_f, w_f = frame.shape[:2]
        cx, cy, r_mid = self._compute_sampling_circle(frame.shape)
        # Normalize angles
        a_min = ang_min_rad % (2*math.pi)
        a_max = ang_max_rad % (2*math.pi)
        # Mid-angle vectorially
        mid_x = math.cos(a_min) + math.cos(a_max)
        mid_y = math.sin(a_min) + math.sin(a_max)
        mid_rad = math.atan2(mid_y, mid_x)
        # Angular difference
        deg_min = math.degrees(a_min) % 360.0
        deg_max = math.degrees(a_max) % 360.0
        diff = (deg_max - deg_min) % 360.0
        if diff > 180:
            diff = 360.0 - diff
        width_rad = math.radians(diff)
        # 20% of cluster range: half-span
        half_span = width_rad * 0.20 / 2.0
        # If width small, ensure at least minimal angular span for 3 samples
        if half_span <= 0:
            scan_angles = [mid_rad]
        else:
            scan_angles = np.linspace(mid_rad - half_span, mid_rad + half_span, num=3)
        pixels = []
        for scan_theta in scan_angles:
            img_theta = self._angle_to_image_theta(scan_theta)
            for dr in [-1, 0, 1]:
                r = r_mid + dr
                x = int(cx + r * math.cos(img_theta))
                y = int(cy + r * math.sin(img_theta))
                if 0 <= x < w_f and 0 <= y < h_f:
                    pixels.append(frame[y, x])
        if not pixels:
            img_theta = self._angle_to_image_theta(mid_rad)
            sample_x = int(cx + r_mid * math.cos(img_theta))
            sample_y = int(cy + r_mid * math.sin(img_theta))
            return np.array([0,0,0]), (sample_x, sample_y)
        arr = np.array(pixels, dtype=np.uint8)
        hsv_px = cv2.cvtColor(arr.reshape(-1,1,3), cv2.COLOR_BGR2HSV).reshape(-1,3)
        # Circular mean for H, median for S and V
        mean_h = self._circular_mean_hue(hsv_px[:,0])
        mean_s = float(np.median(hsv_px[:,1]))
        mean_v = float(np.median(hsv_px[:,2]))
        img_theta = self._angle_to_image_theta(mid_rad)
        sample_x = int(cx + r_mid * math.cos(img_theta))
        sample_y = int(cy + r_mid * math.sin(img_theta))
        return np.array([mean_h, mean_s, mean_v]), (sample_x, sample_y)

    def scan_cb(self, scan: LaserScan):
        t0 = time.time()
        params = {p.name: p.value for p in self.get_parameters([
            'sector_min_deg','sector_max_deg','max_range',
            'cluster_thresh','min_diameter','max_diameter','brightness_thresh'])}
        sector_min = params['sector_min_deg']
        sector_max = params['sector_max_deg']

        a0 = scan.angle_min
        da = scan.angle_increment
        ranges = np.array(scan.ranges)
        ints = np.array(scan.intensities)
        N = len(ranges)
        angles = a0 + np.arange(N) * da
        degs = (np.degrees(angles) % 360.0)
        if sector_min <= sector_max:
            mask = (degs >= sector_min) & (degs <= sector_max)
        else:
            mask = (degs >= sector_min) | (degs <= sector_max)

        clusters = []
        cur = []
        bad = 0
        for i in range(N):
            r = ranges[i]
            in_sector = bool(mask[i])
            valid_r = math.isfinite(r) and (r <= params['max_range'])
            last_r = ranges[cur[-1]] if cur else None
            jump_ok = True
            if cur and valid_r:
                if abs(r - last_r) >= params['cluster_thresh']:
                    jump_ok = False
            if (not in_sector) or (not valid_r) or (cur and not jump_ok):
                bad += 1
                if bad >= 12 and cur:
                    clusters.append(cur)
                    cur = []
                    bad = 0
            else:
                bad = 0
                cur.append(i)
        if cur:
            clusters.append(cur)

        stamp_secs = scan.header.stamp.sec
        stamp_nano = scan.header.stamp.nanosec
        ts_scan = stamp_secs + stamp_nano * 1e-9 - 0.041
        frame = self._get_frame(ts_scan)
        display_frame = frame.copy() if frame is not None else None
        if frame is None:
            if not self._warned_empty:
                self.get_logger().warning("Frame buffer empty; no frames captured yet.")
                self._warned_empty = True
        else:
            self._warned_empty = False

        # Draw sampling circle
        if display_frame is not None:
            cx, cy, r_mid = self._compute_sampling_circle(display_frame.shape)
            overlay = display_frame.copy()
            cv2.circle(overlay, (cx, cy), r_mid, (255,255,255), 2)
            cv2.addWeighted(overlay, 0.5, display_frame, 0.5, 0, display_frame)

        confirmed = []
        # Pre-analysis red dots
        if display_frame is not None:
            for c in clusters:
                if len(c) < 1:
                    continue
                angs = angles[c]
                mid_rad = (angs[0] + angs[-1]) / 2.0
                if frame is not None:
                    _, (x_px, y_px) = self._sample_hsv_half_range(frame, angs[0], angs[-1])
                    cv2.circle(display_frame, (x_px, y_px), 4, (0,0,255), -1)

        # Final analysis and overlay, marking 20% angular region
        if display_frame is not None:
            cx, cy, r_mid = self._compute_sampling_circle(display_frame.shape)
        for c in clusters:
            if len(c) < 2:
                continue
            rs = ranges[c]
            angs = angles[c]
            r_avg = float(rs.mean())
            mid_rad = (angs[0] + angs[-1]) / 2.0
            width = abs(angs[-1] - angs[0]) * r_avg
            if not (params['min_diameter'] <= width <= params['max_diameter']):
                continue
            # Overlay 20% cluster angular range
            if display_frame is not None:
                a_min = angs[0]
                a_max = angs[-1]
                a_min_n = a_min % (2*math.pi)
                a_max_n = a_max % (2*math.pi)
                deg_min = math.degrees(a_min_n) % 360.0
                deg_max = math.degrees(a_max_n) % 360.0
                diff = (deg_max - deg_min) % 360.0
                if diff > 180: diff = 360.0 - diff
                width_rad = math.radians(diff)
                half_span = width_rad * 0.20 / 2.0
                num_pts = 20
                scan_angles = np.linspace(mid_rad - half_span, mid_rad + half_span, num=num_pts)
                pts = []
                for scan_theta in scan_angles:
                    img_theta = self._angle_to_image_theta(scan_theta)
                    x = int(cx + r_mid * math.cos(img_theta))
                    y = int(cy + r_mid * math.sin(img_theta))
                    pts.append((x, y))
                if pts:
                    cv2.polylines(display_frame, [np.array(pts, dtype=np.int32)], False, (255,0,0), 2)
            if frame is None:
                continue
            (h, s, v), (x_px, y_px) = self._sample_hsv_half_range(frame, angs[0], angs[-1])
            if v < params['brightness_thresh']:
                continue
            # Dynamic thresholds
            m_g = (55 - 130) / 0.9; b_g = 130 - m_g * 0.1
            m_r = (55 - 120) / 0.9; b_r = 120 - m_r * 0.1
            s_g = m_g * r_avg + b_g
            s_r = m_r * r_avg + b_r
            if (0 <= h <= 10 or 150 <= h <= 180) and s > s_r and v > 15:
                color = 'RED'
                reason = f"H={h:.1f} in RED range, S={s:.1f}>{s_r:.1f}, V={v:.1f}>15"
            elif 50 <= h <= 100 and s > s_g and v > 10:
                color = 'GREEN'
                reason = f"H={h:.1f} in GREEN range, S={s:.1f}>{s_g:.1f}, V={v:.1f}>10"
            else:
                color = 'NONE'
                reason = f"H={h:.1f}, S={s:.1f}, V={v:.1f} no match"
            self.get_logger().info(f"Final HSV @ {math.degrees(mid_rad)%360:.1f}°: H={h:.1f}, S={s:.1f}, V={v:.1f} -> {color} because {reason}")
            if display_frame is not None:
                cv2.circle(display_frame, (x_px, y_px), 5, (0,255,0), -1)
                cv2.circle(display_frame, (x_px, y_px), 7, (0,255,0), 1)
                angle_text = f"{math.degrees(mid_rad)%360:.1f}° {color}"
                hsv_text2 = f"H:{h:.0f} S:{s:.0f} V:{v:.0f}"
                cv2.putText(display_frame, angle_text, (x_px+8, y_px-8), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0,255,0), 3, cv2.LINE_AA)
                cv2.putText(display_frame, hsv_text2, (x_px+8, y_px+22), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0,255,0), 3, cv2.LINE_AA)
            confirmed.append({
                'angle_min_deg': math.degrees(angs[0]) % 360,
                'angle_max_deg': math.degrees(angs[-1]) % 360,
                'middle_angle_deg': math.degrees(mid_rad) % 360,
                'distance': r_avg,
                'color': color
            })
        if display_frame is not None:
            cv2.imshow("Frame with Sample Overlay", display_frame)
            cv2.waitKey(1)
        for obs in confirmed:
            self.get_logger().info(f"Obs @ {obs['middle_angle_deg']:.1f}° dist {obs['distance']:.2f} m: {obs['color']}")
        elapsed = time.time() - t0
        self.get_logger().debug(f"scan_cb elapsed: {elapsed:.3f}s")

    def destroy_node(self):
        try:
            if self.cap and self.cap.isOpened():
                self.cap.release()
            cv2.destroyAllWindows()
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = ScanImageAnalyzer()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
