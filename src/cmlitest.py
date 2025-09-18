#!/usr/bin/env python3
import threading
import time
import math
from collections import deque
import subprocess

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2

class ScanImageAnalyzer(Node):
    def __init__(self):
        super().__init__('scan_image_analyzer')

        # Declare parameters (tweaked values)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sector_min_deg',   260.0),
                ('sector_max_deg',   100.0),
                ('max_range',        1.6),
                ('cluster_thresh',   0.18),
                ('min_diameter',     0.011),
                ('max_diameter',     0.08),
                ('brightness_thresh',94.0),
                ('sync_delay_ms',    -33.0),  # desired delay in milliseconds
                ('buffer_duration_s',1.0),   # how long to keep frames in buffer
                ('exposure_time_absolute', 41),   # manual exposure time
                ('auto_exposure',    False),  # whether to enable auto exposure
                ('gain',             100),    # manual gain if supported
                ('white_balance_temperature', 4000),
                ('angle_offset_deg',  -1.1),   # angle offset for color analysis
            ]
        )

        # Apply v4l2-ctl settings before opening camera
        dev = "/dev/video0"
        params = {p.name: p.value for p in self.get_parameters([
            'auto_exposure','exposure_time_absolute','gain','white_balance_temperature'])}
        ae = params['auto_exposure']
        exp_val = int(params['exposure_time_absolute'])
        gain_val = int(params['gain'])
        wb_temp = int(params['white_balance_temperature'])
        try:
            if ae:
                subprocess.run(["v4l2-ctl", f"--device={dev}", "--set-ctrl=auto_exposure=3"], check=True)
            else:
                subprocess.run(["v4l2-ctl", f"--device={dev}", "--set-ctrl=auto_exposure=1"], check=True)
                subprocess.run(["v4l2-ctl", f"--device={dev}", f"--set-ctrl=exposure_time_absolute={exp_val}"], check=True)
            # white balance and gain commented out as per tweaks
            # subprocess.run(["v4l2-ctl", f"--device={dev}", "--set-ctrl=white_balance_automatic=0"], check=True)
            # subprocess.run(["v4l2-ctl", f"--device={dev}", f"--set-ctrl=white_balance_temperature={wb_temp}"], check=True)
            # subprocess.run(["v4l2-ctl", f"--device={dev}", f"--set-ctrl=gain={gain_val}"], check=True)
        except Exception as e:
            self.get_logger().warning(f"Could not apply v4l2-ctl settings: {e}")

        # Camera setup: 90 FPS, 1900x1200
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1900)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FPS, 90)
        if not self.cap.isOpened():
            self.get_logger().fatal("Cannot open camera at /dev/video0")
            rclpy.shutdown()
            return

        # Frame buffer: store recent frames (timestamp, frame)
        self.buffer = deque()
        self.buffer_lock = threading.Lock()
        threading.Thread(target=self._reader_thread, daemon=True).start()

        # Subscribe to LaserScan
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # Window
        cv2.namedWindow("Frame with Sample Overlay", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Frame with Sample Overlay", 1200, 600)

        self.get_logger().info('Scan-image analyzer started with tweaked parameters.')

    def _reader_thread(self):
        while rclpy.ok() and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.005)
                continue
            ts = time.time()
            with self.buffer_lock:
                self.buffer.append((ts, frame.copy()))
                buf_dur = self.get_parameter('buffer_duration_s').value
                while self.buffer and (ts - self.buffer[0][0] > buf_dur):
                    self.buffer.popleft()

    def _get_best_frame(self, target_ts):
        best = None
        best_diff = float('inf')
        with self.buffer_lock:
            for ts, frame in self.buffer:
                diff = abs(ts - target_ts)
                if diff < best_diff:
                    best_diff = diff
                    best = (ts, frame)
        if best is None:
            return None, None, None
        return best[0], best[1], best_diff

    def _compute_sampling_circle(self, frame_shape):
        h_f, w_f = frame_shape[:2]
        cx = w_f // 2 + 7 + 26
        cy = h_f // 2 + 10
        outer = min(cx, cy) - 26
        inner = outer - 3
        r_mid_orig = (outer + inner) / 2.0
        r_mid = int(r_mid_orig * 0.73)
        return cx, cy, r_mid

    def _angle_to_image_theta(self, scan_rad):
        deg = math.degrees(scan_rad) % 360.0
        img_deg = (270.0 + deg) % 360.0
        return math.radians(img_deg)

    def _circular_mean_hue(self, hs):
        rad = hs.astype(np.float32) * 2.0 * math.pi / 180.0
        x = np.cos(rad); y = np.sin(rad)
        mx = np.mean(x); my = np.mean(y)
        mean_ang = math.atan2(my, mx)
        if mean_ang < 0: mean_ang += 2*math.pi
        mean_deg = math.degrees(mean_ang)
        return mean_deg * 0.5

    def _sample_hsv_range(self, frame, ang_min_rad, ang_max_rad):
        h_f, w_f = frame.shape[:2]
        cx, cy, r_mid = self._compute_sampling_circle(frame.shape)
        a_min = ang_min_rad % (2*math.pi); a_max = ang_max_rad % (2*math.pi)
        mid_x = math.cos(a_min) + math.cos(a_max)
        mid_y = math.sin(a_min) + math.sin(a_max)
        mid_rad = math.atan2(mid_y, mid_x)
        # apply angle offset
        offset_deg = self.get_parameter('angle_offset_deg').value
        mid_rad += math.radians(offset_deg)
        deg_min = math.degrees(a_min) % 360.0; deg_max = math.degrees(a_max) % 360.0
        diff = (deg_max - deg_min) % 360.0
        if diff > 180: diff = 360.0 - diff
        width_rad = math.radians(diff)
        half_span = width_rad * 0.20 / 2.0
        scan_angles = [mid_rad] if half_span <= 0 else np.linspace(mid_rad-half_span, mid_rad+half_span, num=3)
        pixels = []
        for scan_theta in scan_angles:
            img_theta = self._angle_to_image_theta(scan_theta)
            for dr in [-1,0,1]:
                r = r_mid + dr
                x = int(cx + r * math.cos(img_theta)); y = int(cy + r * math.sin(img_theta))
                if 0 <= x < w_f and 0 <= y < h_f:
                    pixels.append(frame[y, x])
        if not pixels:
            img_theta = self._angle_to_image_theta(mid_rad)
            sx = int(cx + r_mid * math.cos(img_theta)); sy = int(cy + r_mid * math.sin(img_theta))
            return np.array([0,0,0]), (sx, sy)
        arr = np.array(pixels, dtype=np.uint8)
        hsv_px = cv2.cvtColor(arr.reshape(-1,1,3), cv2.COLOR_BGR2HSV).reshape(-1,3)
        mean_h = self._circular_mean_hue(hsv_px[:,0])
        mean_s = float(np.median(hsv_px[:,1])); mean_v = float(np.median(hsv_px[:,2]))
        img_theta = self._angle_to_image_theta(mid_rad)
        sx = int(cx + r_mid * math.cos(img_theta)); sy = int(cy + r_mid * math.sin(img_theta))
        return np.array([mean_h, mean_s, mean_v]), (sx, sy)

    def scan_cb(self, scan: LaserScan):
        t0 = time.time()
        params = {p.name: p.value for p in self.get_parameters([
            'sector_min_deg','sector_max_deg','max_range',
            'cluster_thresh','min_diameter','max_diameter','brightness_thresh',
            'sync_delay_ms','buffer_duration_s','angle_offset_deg'])}
        sector_min = params['sector_min_deg']; sector_max = params['sector_max_deg']
        sync_delay = params['sync_delay_ms'] / 1000.0

        stamp_secs = scan.header.stamp.sec; stamp_nano = scan.header.stamp.nanosec
        lidar_ts = stamp_secs + stamp_nano * 1e-9
        self.get_logger().info(f"LiDAR timestamp: {stamp_secs}.{stamp_nano:09d} ({lidar_ts:.6f}s)")

        target_ts = lidar_ts - sync_delay
        frame_ts, frame, diff = self._get_best_frame(target_ts)
        if frame is None:
            self.get_logger().warning("No camera frame available for sync")
            display_frame = None
        else:
            self.get_logger().info(f"Selected camera frame timestamp: {frame_ts:.6f}s (diff {diff*1000:.1f}ms)")
            display_frame = frame.copy()

        a0 = scan.angle_min; da = scan.angle_increment
        ranges = np.array(scan.ranges); ints = np.array(scan.intensities)
        N = len(ranges); angles = a0 + np.arange(N)*da; degs = (np.degrees(angles)%360.0)
        if sector_min <= sector_max:
            mask = (degs >= sector_min) & (degs <= sector_max)
        else:
            mask = (degs >= sector_min) | (degs <= sector_max)
        clusters = []; cur=[]; bad=0
        for i in range(N):
            r = ranges[i]; in_sector=bool(mask[i]); valid_r=math.isfinite(r) and (r<=params['max_range'])
            last_r = ranges[cur[-1]] if cur else None; jump_ok=True
            if cur and valid_r and abs(r-last_r)>=params['cluster_thresh']: jump_ok=False
            if (not in_sector) or (not valid_r) or (cur and not jump_ok):
                bad+=1
                if bad>=12 and cur: clusters.append(cur); cur=[]; bad=0
            else: bad=0; cur.append(i)
        if cur: clusters.append(cur)

        if display_frame is not None:
            cx, cy, r_mid = self._compute_sampling_circle(display_frame.shape)
            overlay = display_frame.copy()
            cv2.circle(overlay, (cx, cy), r_mid, (255,255,255), 2)
            cv2.addWeighted(overlay, 0.5, display_frame, 0.5, 0, display_frame)
        confirmed=[]
        if display_frame is not None:
            for c in clusters:
                if len(c)<1: continue
                angs=angles[c]; _,(x_px,y_px)=self._sample_hsv_range(frame, angs[0], angs[-1])
                cv2.circle(display_frame,(x_px,y_px),4,(0,0,255),-1)
        if display_frame is not None:
            cx, cy, r_mid = self._compute_sampling_circle(display_frame.shape)
        for c in clusters:
            if len(c)<2: continue
            rs=ranges[c]; angs=angles[c]; r_avg=float(rs.mean()); mid_rad=(angs[0]+angs[-1])/2.0
            width=abs(angs[-1]-angs[0])*r_avg
            if not(params['min_diameter']<=width<=params['max_diameter']): 
                self.get_logger().debug(f"Dropped for width: {width}")
                continue
            if display_frame is not None:
                a_min=angs[0]; a_max=angs[-1]; a_min_n=a_min%(2*math.pi); a_max_n=a_max%(2*math.pi)
                deg_min=math.degrees(a_min_n)%360.0; deg_max=math.degrees(a_max_n)%360.0
                diff_ang=(deg_max-deg_min)%360.0
                if diff_ang>180: diff_ang=360.0-diff_ang
                width_rad=math.radians(diff_ang); half_span=width_rad*0.20/2.0
                offset_rad = math.radians(params['angle_offset_deg'])
                center = mid_rad + offset_rad
                scan_angles = np.linspace(center-half_span, center+half_span, num=20) if half_span>0 else [center]
                pts=[]
                for st in scan_angles:
                    img_theta=self._angle_to_image_theta(st)
                    x=int(cx+r_mid*math.cos(img_theta)); y=int(cy+r_mid*math.sin(img_theta)); pts.append((x,y))
                if pts: cv2.polylines(display_frame,[np.array(pts,np.int32)],False,(255,0,0),2)
            if frame is None: continue
            (h,s,v),(x_px,y_px)=self._sample_hsv_range(frame,angs[0],angs[-1])
            if v<params['brightness_thresh']:
                self.get_logger().debug(f"Dropped for brightness: {v}")
                continue
            m_g=(55-130)/0.9; b_g=130-m_g*0.1; m_r=(55-120)/0.9; b_r=110-m_r*0.1
            s_g=m_g*r_avg+b_g; s_r=m_r*r_avg+b_r
            if (0<=h<=20 or 150<=h<=180) and s>s_r and v>15:
                color='RED'; reason=f"H={h:.1f} in RED, S={s:.1f}>{s_r:.1f}, V={v:.1f}>15"
            elif 38<=h<=100 and s>s_g and v>10:
                color='GREEN'; reason=f"H={h:.1f} in GREEN, S={s:.1f}>{s_g:.1f}, V={v:.1f}>10"
            else:
                color='NONE'; reason=f"H={h:.1f}, S={s:.1f}, V={v:.1f} no match"
            # Only overlay text for RED or GREEN
            if color in ('RED', 'GREEN') and display_frame is not None:
                angle_deg = math.degrees((mid_rad+math.radians(params['angle_offset_deg']))%(2*math.pi))
                self.get_logger().info(f"Final HSV @ {angle_deg:.1f}°: H={h:.1f}, S={s:.1f}, V={v:.1f} -> {color} because {reason}")
                cv2.circle(display_frame,(x_px,y_px),5,(0,255,0),-1)
                cv2.circle(display_frame,(x_px,y_px),7,(0,255,0),1)
                cv2.putText(display_frame,f"{angle_deg:.1f}° {color}",(x_px+8,y_px-8),cv2.FONT_HERSHEY_SIMPLEX,2.0,(0,255,0),3)
                cv2.putText(display_frame,f"H:{h:.0f} S:{s:.0f} V:{v:.0f}",(x_px+8,y_px+22),cv2.FONT_HERSHEY_SIMPLEX,2.0,(0,255,0),3)
            else:
                self.get_logger().debug(f"Ignored obstacle at H={h:.1f},S={s:.1f},V={v:.1f} -> {color}")
            confirmed.append({'angle_min_deg':math.degrees(angs[0])%360,'angle_max_deg':math.degrees(angs[-1])%360,'middle_angle_deg':math.degrees((mid_rad+math.radians(params['angle_offset_deg']))%(2*math.pi)),'distance':r_avg,'color':color})
        if display_frame is not None:
            cv2.imshow("Frame with Sample Overlay", display_frame); cv2.waitKey(1)
        for obs in confirmed:
            if obs['color'] in ('RED','GREEN'):
                self.get_logger().info(f"Obs @ {obs['middle_angle_deg']:.1f}° dist {obs['distance']:.2f} m: {obs['color']}")
        elapsed = time.time()-t0; self.get_logger().debug(f"scan_cb elapsed: {elapsed:.3f}s")

    def destroy_node(self):
        try:
            if self.cap and self.cap.isOpened(): self.cap.release()
            cv2.destroyAllWindows()
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = ScanImageAnalyzer()
    try:
        while rclpy.ok(): rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.")
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
