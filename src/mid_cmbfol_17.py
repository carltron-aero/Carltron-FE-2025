#!/usr/bin/env python3
import os
import math
import time
import threading
import subprocess
from collections import deque, Counter

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from drivebase import DriveBase

class CombinedFollower(Node):
    def __init__(self):
        super().__init__('combined_follower')

        # --- Instantiate DriveBase and IMU setup ---
        self.drive_base = DriveBase()
        try:
            self.drive_base.setup_all()
            self.drive_base.setup_imu()
            self.drive_base.reset_yaw_reference()
        except Exception as e:
            self.get_logger().warning(f"DriveBase/IMU setup failed: {e}")

        # --- State variables ---
        self.mode = 'OBS'
        self.direction = "COUNTER"  # or "COUNTER"
        self.obs_status = "START"
        self.obs_drive_state = None
        self.parking = False

        # For consensus: keep last states, up to 12
        self.state_history = deque(maxlen=12)
        # SEARCH counting
        self.search_count = 0
        self.search_required = 12  # exit SEARCH after 12 scans
        # section counting
        self.section_count = 0

        # Collision avoidance variables
        self.coll_avd = "N"      # "N", "CAU", or "AVD"
        self.coll_count = 0      # consecutive scans triggering primary avoidance
        self.coll_avd_ts = 0.0   # timestamp when AVD was set
        self.prev_speed = 0

        # For OBS PASSED heading target
        self.start_yaw = None           # in radians, if used
        self.target_heading = 0.0       # in degrees
        self.last_d = None

        # Limits in radians for soft limiting
        self.max_left = math.radians(55)
        self.max_right = math.radians(55)

        # Sampling angles for other controllers
        self.left_side_angles   = [math.radians(a) for a in (96,103,108,124)]
        self.left_front_angles  = [math.radians(a) for a in (128,125)]
        self.right_side_angles  = [math.radians(a) for a in (245,242,237)]
        self.right_front_angles = [math.radians(a) for a in (228,235)]

        # Controller tuning variables
        self.v_max = 0.82
        self.obs_v = 0.67
        self.kp_heading = 1.0
        self.kp_lateral = 1.0
        self.angle_limit_corr_gain = 0.05
        self.turn_limiter_enable = True

        # Speed limits (tunable)
        self.speed_max = 0.64
        self.speed_min = 0.46

        # IMU buffer for timed yaw syncing
        self.imu_buffer = deque()  # (timestamp, yaw_deg)
        self.imu_lock = threading.Lock()
        self.imu_sync_delay_ms = 0.0
        self.imu_buffer_rate_hz = 200.0
        self.buffer_duration_s = 1.0
        threading.Thread(target=self._imu_reader_thread, daemon=True).start()

        # Camera & detection tuning variables
        self.sector_min_deg    = 220.0
        self.sector_max_deg    = 140.0
        self.max_range         = 1.6
        self.cluster_thresh    = 0.15
        self.min_diameter      = 0.011
        self.max_diameter      = 0.08
        self.brightness_thresh = 70.0
        self.sync_delay_ms     = -60.0
        self.angle_offset_deg  = -1.1
        # Corridor new_perp range
        self.perp_min_dist = 0.28
        self.perp_max_dist = 0.8

        # ALIGN tuning
        self.align_scans = []
        self.align_max = 10
        self.align_sector_half_deg = 12
        self.align_offset_deg = 1.8  # subtract in alignment
        self.turn_tolerance_deg = 5  # tolerance for stopping turn

        # Calibration & exposure
        self.calibration_mode = False
        self.calib_folder = 'tmp/calibration_frames'
        self.calib_max_frames = 10
        self._calib_count = 0
        self.auto_exposure = False
        self.exposure_time_absolute = 65
        self.gain = 100
        self.white_balance_temperature = 4000

        # Initialize counters
        self.donetimes = 0
        self.stop_times = 0
        self.done_counter = 0
        self.lapstop_counter = 0

        # Camera setup & exposure control
        dev = "/dev/video0"
        try:
            if self.auto_exposure:
                subprocess.run(["v4l2-ctl", f"--device={dev}", "--set-ctrl=auto_exposure=3"], check=True)
            else:
                subprocess.run(["v4l2-ctl", f"--device={dev}", "--set-ctrl=auto_exposure=1"], check=True)
                subprocess.run(
                    ["v4l2-ctl", f"--device={dev}", f"--set-ctrl=exposure_time_absolute={int(self.exposure_time_absolute)}"],
                    check=True)
        except Exception as e:
            self.get_logger().warning(f"Could not apply v4l2-ctl settings: {e}")

        # Open camera
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
        self.buffer = deque()
        self.buffer_lock = threading.Lock()
        threading.Thread(target=self._camera_reader_thread, daemon=True).start()

        # Subscriptions
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(String, '/mode', self.mode_cb, 10)

        # Turn control timer at 50 Hz
        self.create_timer(1.0/50.0, self._turn_control)

        # LED thread
        self._led_thread_stop = False
        threading.Thread(target=self._led_thread_func, daemon=True).start()

        # START logic variables
        self.start_decisions = []
        self.start_batch_count = 0
        self.start_batch_size = 10
        self.wait_pressed_time = None

        self.get_logger().info('CombinedFollower started.')

    # --------------- IMU reader thread ---------------
    def _imu_reader_thread(self):
        interval = 1.0 / self.imu_buffer_rate_hz if self.imu_buffer_rate_hz > 0 else 0.01
        while rclpy.ok():
            ts = time.time()
            try:
                yaw_deg = self.drive_base.get_continuous_yaw()
            except Exception:
                yaw_deg = None
            with self.imu_lock:
                if yaw_deg is not None:
                    self.imu_buffer.append((ts, yaw_deg))
                    while self.imu_buffer and (ts - self.imu_buffer[0][0] > self.buffer_duration_s):
                        self.imu_buffer.popleft()
            time.sleep(interval)

    def _get_best_yaw(self, target_ts):
        best = None; best_diff = float('inf')
        with self.imu_lock:
            for ts, yaw in self.imu_buffer:
                diff = abs(ts - target_ts)
                if diff < best_diff:
                    best_diff = diff; best = yaw
        return best, best_diff

    # --------------- Camera reader thread ---------------
    def _camera_reader_thread(self):
        while rclpy.ok() and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.005)
                continue
            ts = time.time()
            with self.buffer_lock:
                self.buffer.append((ts, frame.copy()))
                while self.buffer and (ts - self.buffer[0][0] > self.buffer_duration_s):
                    self.buffer.popleft()

    def _get_best_frame(self, target_ts):
        best = None; best_diff = float('inf')
        with self.buffer_lock:
            for ts, frame in self.buffer:
                diff = abs(ts - target_ts)
                if diff < best_diff:
                    best_diff = diff; best = (ts, frame)
        if best is None:
            return None, None, None
        return best[0], best[1], best_diff

    # --------------- Utility methods ---------------
    def _compute_sampling_circle(self, frame_shape):
        h_f, w_f = frame_shape[:2]
        cx = w_f // 2 + 7 + 34
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
        mid_rad += math.radians(self.angle_offset_deg)
        deg_min = math.degrees(a_min) % 360.0; deg_max = math.degrees(a_max) % 360.0
        diff = (deg_max - deg_min) % 360.0
        if diff > 180: diff = 360.0 - diff
        width_rad = math.radians(diff)
        half_span = width_rad * 0.20 / 2.0
        if half_span <= 0:
            scan_angles = [mid_rad]
        else:
            scan_angles = np.linspace(mid_rad - half_span, mid_rad + half_span, num=3)
        pixels = []
        for scan_theta in scan_angles:
            img_theta = self._angle_to_image_theta(scan_theta)
            for dr in (-1,0,1):
                r = r_mid + dr
                x = int(cx + r * math.cos(img_theta))
                y = int(cy + r * math.sin(img_theta))
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

    @staticmethod
    def normalize_angle(rad):
        return (rad + math.pi) % (2*math.pi) - math.pi

    def apply_turn_limiter(self, steer, target_heading_rad, yaw_current_rad):
        if not self.turn_limiter_enable:
            return steer
        rel_yaw = self.normalize_angle(target_heading_rad - yaw_current_rad)
        corr_gain = self.angle_limit_corr_gain
        if -self.max_left <= rel_yaw <= self.max_right:
            if steer > 0:
                dist = self.max_right - rel_yaw
                steer *= max(0.0, min(1.0, dist / self.max_right))
            elif steer < 0:
                dist = self.max_left + rel_yaw
                steer *= max(0.0, min(1.0, dist / self.max_left))
        else:
            if rel_yaw > self.max_right:
                if steer > 0: steer = 0.0
                steer += -corr_gain * (rel_yaw - self.max_right)
            elif rel_yaw < -self.max_left:
                if steer < 0: steer = 0.0
                steer += corr_gain * ((-self.max_left) - rel_yaw)
        return steer

    def mode_cb(self, msg: String):
        old = self.mode
        self.mode = msg.data.strip()
        self.get_logger().info(f"Mode changed from {old} to {self.mode}")
        # On mode change, go to WAIT
        self.obs_status = "WAIT"
        self.wait_pressed_time = None

    # --------------- LED background thread ---------------
    def _led_thread_func(self):
        prev_status = None
        led_state = False
        acc = 0.0
        last_time = time.time()
        slow_on = 0.1
        while not self._led_thread_stop and rclpy.ok():
            now = time.time()
            dt = now - last_time
            last_time = now
            status = self.obs_status
            if status != prev_status:
                prev_status = status
                acc = 0.0
                if status == "START":
                    led_state = True
                    try: self.drive_base.set_led(True)
                    except: pass
                else:
                    led_state = False
                    try: self.drive_base.set_led(False)
                    except: pass
            if status == "START":
                # LED on permanently
                if not led_state:
                    led_state = True
                    try: self.drive_base.set_led(True)
                    except: pass
                time.sleep(0.1)
                continue
            elif status == "WAIT":
                # blink every 500ms
                acc += dt
                if acc >= 0.5:
                    led_state = not led_state
                    try: self.drive_base.set_led(led_state)
                    except: pass
                    acc -= 0.5
                time.sleep(0.1)
                continue
            else:
                # blink once every 2 seconds
                acc += dt
                if not led_state:
                    if acc >= 2.0:
                        led_state = True
                        try: self.drive_base.set_led(True)
                        except: pass
                        acc = 0.0
                else:
                    if acc >= slow_on:
                        led_state = False
                        try: self.drive_base.set_led(False)
                        except: pass
                        acc = 0.0
                time.sleep(0.1)
                continue

    # --------------- Main scan callback ---------------
    def scan_cb(self, scan: LaserScan):
        t0 = time.time()
        stamp_secs = scan.header.stamp.sec
        stamp_nano = scan.header.stamp.nanosec
        lidar_ts = stamp_secs + stamp_nano * 1e-9
        self.get_logger().info(f"LiDAR timestamp: {stamp_secs}.{stamp_nano:09d} ({lidar_ts:.6f}s)")

        # Sync camera frame
        target_cam_ts = lidar_ts - (self.sync_delay_ms / 1000.0)
        frame_ts, frame, diff_cam = self._get_best_frame(target_cam_ts)
        if frame is None:
            self.get_logger().warning("No camera frame available for sync")
            display_frame = None
        else:
            self.get_logger().info(f"Selected camera frame ts: {frame_ts:.6f}s (diff {diff_cam*1000:.1f}ms)")
            display_frame = frame.copy()

        # Sync IMU yaw
        target_imu_ts = lidar_ts - (self.imu_sync_delay_ms / 1000.0)
        yaw_val, diff_imu = self._get_best_yaw(target_imu_ts)
        if yaw_val is None:
            self.get_logger().warning("No IMU yaw available for sync; using current yaw")
            try:
                yaw = self.drive_base.get_continuous_yaw()
            except Exception:
                yaw = 0.0
        else:
            yaw = yaw_val
            self.get_logger().info(f"LIDAR-timed yaw: {yaw:.1f}° (diff {diff_imu*1000:.1f}ms)")

        # --- Button check in scan_cb: if waiting started and button released -> stop+skip ---
        if self.wait_pressed_time is not None:
            try:
                btn_state = self.drive_base.get_button_state()
            except Exception:
                btn_state = True  # assume released if error
            # button pressed is when get_button_state() is False
            if btn_state:  # released/high -> stop
                try:
                    self.drive_base.set_speed(0.0)
                    self.drive_base.set_steering(0.0)
                except Exception:
                    pass
                self.get_logger().info("Button released after start: stopping and skipping scan processing")
                return

        # Prepare LIDAR sampling arrays
        ranges = np.array(scan.ranges)
        a0, da = scan.angle_min, scan.angle_increment
        N = len(ranges)
        angs = a0 + np.arange(N)*da
        clean = ranges.copy()
        def sample(a_rad):
            angle_min = a0
            angle_max = a0 + (N-1)*da
            a = a_rad
            span = angle_max - angle_min
            if span >= 2*math.pi - 1e-6:
                a = ((a - angle_min) % (2*math.pi)) + angle_min
            else:
                a_wrapped = ((a - angle_min) % (2*math.pi)) + angle_min
                if angle_min <= a_wrapped <= angle_max:
                    a = a_wrapped
                else:
                    return None
            idx = int((a - a0)/da)
            if idx < 0 or idx >= N:
                return None
            r = clean[idx]
            return r if math.isfinite(r) and r > 0.01 else None

        def sample_range(start_deg, end_deg):
            vals = []
            for a in range(int(start_deg), int(end_deg)):
                r = sample(math.radians(a))
                if r:
                    vals.append(r)
            return (sum(vals)/len(vals)) if vals else float('inf')

        yaw_synced_deg = yaw
        try:
            yaw_current_deg = self.drive_base.get_continuous_yaw()
        except Exception:
            yaw_current_deg = yaw_synced_deg
        yaw_current_rad = math.radians(yaw_current_deg)

        ps = self.prev_speed

                # primary AVD: from 0.11 at speed_min to 0.22 at 0.6
        vmin = self.speed_min
        vmax = 0.6

        # --- Collision avoidance early ---
        # Only check AVD primary rectangle if not already in AVD and moving forward enough:
        if self.obs_status != "START" and self.obs_status != "START" and self.coll_avd != "AVD" and self.prev_speed > 0.2:
            # compute dynamic forward limits based on prev_speed
            

            if ps <= vmin:
                x_max = 0.14
            elif ps >= vmax:
                x_max = 0.22
            else:
                # linear interp
                x_max = 0.14 + (ps - vmin) / (vmax - vmin) * (0.22 - 0.11)



            # Primary rectangle: x in [0, x_max], y in dynamic sideways limit
            valid_pts = 0
            d_mode = self.obs_drive_state or ""
            d_dir = self.direction
            for r, ang in zip(ranges, angs):
                if not (math.isfinite(r) and r >= 0.04):
                    continue
                x = r * math.cos(ang)
                if ((d_dir == "CLOCK" and d_mode == "GREEN_CLOSE") or (d_dir == "COUNTER" and d_mode == "RED_CLOSE")) and self.section_count % 4 == 0 and self.obs_status == "DRIVE":
                    if self.prev_speed > 0.5:
                        x_max = 0.274
                    else:
                        x_max = 0.255
                if x < 0.0 or x > x_max:
                    continue
                y = r * math.sin(ang)
                #print("wait " + str(y))
                # possibly tighter sideways limit in certain state
                if ((d_dir == "CLOCK" and d_mode == "GREEN_CLOSE") or (d_dir == "COUNTER" and d_mode == "RED_CLOSE")) \
                and self.section_count % 4 == 0:
                    if (d_dir == "CLOCK" and d_mode == "GREEN_CLOSE") and (not (-0.065 < y < 0.072)):
                        continue
                    if (d_dir == "COUNTER" and d_mode == "RED_CLOSE") and (not (-0.074 < y < 0.065)):
                        continue
                else:
                    if abs(y) > 0.082:
                        continue
                valid_pts += 1
                if valid_pts >= 10:
                    break
            if valid_pts >= 10:
                # primary avoidance triggered this scan
                self.coll_count += 1
                try:
                    self.drive_base.set_speed(-0.35)
                    self.drive_base.set_steering(0.0)
                except Exception:
                    pass
                if self.coll_count >= 5 and self.coll_avd != "AVD":
                    self.coll_avd = "AVD"
                    self.coll_avd_ts = time.time()
                    self.get_logger().info("Collision avoidance: entering AVD mode")
                return
            else:
                self.coll_count = 0

                # --- Collision avoidance early ---
        # If currently in AVD mode, drive backwards at -speed_min, steering to maintain target_heading, for 1.2 s:
        if self.coll_avd == "AVD":
            if time.time() - self.coll_avd_ts <= 1.6:
                # Drive backwards at minimum speed, steering to maintain heading
                heading_error = (self.target_heading - yaw_current_deg)
                steer_nominal = 0.049 * heading_error
                # Apply turn limiter if desired
                steer_limited = self.apply_turn_limiter(steer_nominal, math.radians(self.target_heading), yaw_current_rad) \
                                if self.turn_limiter_enable else steer_nominal
                speed_cmd = -0.55
                try:
                    self.drive_base.set_speed(speed_cmd)
                    self.drive_base.set_steering(steer_limited)
                except Exception:
                    pass
                self.get_logger().info(f"AVD active: driving backwards at {speed_cmd:.2f}, steer {steer_limited:.3f}")
                return
            else:
                # Exit AVD mode
                self.coll_avd = "N"
                self.coll_count = 0
                self.get_logger().info("Collision avoidance: exiting AVD mode")


        # secondary CAU: from 0.18 at speed_min to 0.36 at 0.6
        if ps <= vmin:
            x_max2 = 0.18
        elif ps >= vmax:
            x_max2 = 0.36
        else:
            x_max2 = 0.18 + (ps - vmin) / (vmax - vmin) * (0.36 - 0.18)
        
        # Secondary rectangle (CAU) if not in AVD:
        # x in [0, x_max2], y in [-0.10, +0.10]
        valid_pts2 = 0
        for r, ang in zip(ranges, angs):
            if not (math.isfinite(r) and r >= 0.04):
                continue
            x = r * math.cos(ang)
            if x < 0.0 or x > x_max2:
                continue
            y = r * math.sin(ang)
            if abs(y) > 0.10:
                continue
            valid_pts2 += 1
            if valid_pts2 >= 10:
                break
        if valid_pts2 >= 10:
            self.coll_avd = "CAU"
        else:
            if self.coll_avd == "CAU":
                self.coll_avd = "N"

        # If TURN mode, skip heavy detection. _turn_control handles turning
        if self.obs_status == "TURN":
            return

        # --- ALIGN mode handling ---
        if self.obs_status == "ALIGN":
            # collect this scan
            self.align_scans.append((ranges, angs))
            if len(self.align_scans) >= self.align_max:
                pts_all = []
                for rg, ag in self.align_scans:
                    rel = ((ag + math.pi) % (2*math.pi)) - math.pi
                    mask = np.abs(rel) <= math.radians(self.align_sector_half_deg)
                    idxs = np.where(mask)[0]
                    if idxs.size:
                        valid = [i for i in idxs if math.isfinite(rg[i]) and rg[i] > 2.0]
                        if valid:
                            xs = rg[valid] * np.cos(ag[valid])
                            ys = rg[valid] * np.sin(ag[valid])
                            pts = np.vstack((xs, ys)).T
                            medx, medy = np.median(pts[:,0]), np.median(pts[:,1])
                            dists = np.hypot(pts[:,0]-medx, pts[:,1]-medy)
                            inliers = pts[dists < 0.15]
                            if inliers.shape[0] >= 5:
                                pts_all.append(inliers)
                if not pts_all:
                    self.get_logger().warning("ALIGN: no valid wall points in ±12° sector over scans")
                else:
                    all_pts = np.vstack(pts_all)
                    if all_pts.shape[0] < 50:
                        self.get_logger().warning(f"ALIGN: too few total points ({all_pts.shape[0]})")
                    else:
                        centroid = all_pts.mean(axis=0)
                        d_all = np.hypot(all_pts[:,0]-centroid[0], all_pts[:,1]-centroid[1])
                        med = np.median(d_all)
                        thresh = max(0.2, 1.5*med)
                        inliers = all_pts[d_all < thresh]
                        if inliers.shape[0] < 40:
                            self.get_logger().warning(f"ALIGN: too few inliers after prune ({inliers.shape[0]})")
                        else:
                            pts0 = inliers - inliers.mean(axis=0)
                            cov = np.dot(pts0.T, pts0) / pts0.shape[0]
                            eigvals, eigvecs = np.linalg.eig(cov)
                            v = eigvecs[:, np.argmax(eigvals)]
                            wall_ang = math.degrees(math.atan2(v[1], v[0])) % 360.0
                            rel = wall_ang - 270
                            if rel > 180: rel -= 360.0
                            rel_adj = rel - self.align_offset_deg
                            theoretical = self.target_heading
                            imu_error = yaw_current_deg - theoretical
                            error_align = rel_adj
                            drift = imu_error + error_align
                            if abs(drift) <= 7.0:
                                new_target = theoretical + drift
                                self.get_logger().info(
                                    f"ALIGN: wall_ang={wall_ang:.2f}°, rel={rel:.2f}°, rel_adj={rel_adj:.2f}°, "
                                    f"theoretical={theoretical:.2f}°, imu={yaw_current_deg:.2f}°, "
                                    f"imu_error={imu_error:.2f}°, error_align={error_align:.2f}°, new_target={new_target:.2f}"
                                )
                                self.target_heading = new_target
                            else:
                                self.get_logger().info(
                                    f"ALIGN: wall_ang={wall_ang:.2f}°, rel={rel:.2f}°, rel_adj={rel_adj:.2f}°, "
                                    f"theoretical={theoretical:.2f}°, imu={yaw_current_deg:.2f}°, "
                                    f"imu_error={imu_error:.2f}°, error_align={error_align:.2f}°"
                                )
                                self.get_logger().warning(
                                    f"ALIGN: alignment error too large ({error_align:.2f}°), using theoretical {theoretical:.2f}°"
                                )
                                self.target_heading = theoretical
                # reset ALIGN → SEARCH
                self.align_scans.clear()
                self.obs_status = "SEARCH"
                self.search_count = 0
                self.state_history.clear()
                self.obs_drive_state = None
                self.section_count += 1
                self.get_logger().info(f"ALIGN → SEARCH; section_count={self.section_count}")
            return

        detected = []
        selected = None

        # Detection & selection only if mode == 'OBS' and in SEARCH or DRIVE, and frame available
        if self.mode == 'OBS' and (self.obs_status in ("SEARCH","DRIVE")) and display_frame is not None:
            if self.obs_status == "SEARCH":
                self.search_count += 1

            # Draw measurement heading
            measure_deg = self.target_heading % 360.0
            rel_deg = (measure_deg - yaw) % 360.0
            rel_rad = math.radians(rel_deg)
            img_theta = self._angle_to_image_theta(rel_rad)
            cx, cy, r_mid = self._compute_sampling_circle(display_frame.shape)
            x_m = int(cx + r_mid * math.cos(img_theta))
            y_m = int(cy + r_mid * math.sin(img_theta))
            cv2.circle(display_frame, (x_m, y_m), 6, (203,192,255), -1)
            cv2.putText(display_frame, f"Tgt:{measure_deg:.1f}°", (x_m+8, y_m-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.6, (203,192,255), 3)

            # Process LaserScan clusters
            degs = (np.degrees(angs) % 360.0)
            if self.sector_min_deg <= self.sector_max_deg:
                mask = (degs >= self.sector_min_deg) & (degs <= self.sector_max_deg)
            else:
                mask = (degs >= self.sector_min_deg) | (degs <= self.sector_max_deg)
            clusters = []
            cur = []
            bad = 0
            for i in range(N):
                r = ranges[i]
                in_sector = bool(mask[i])
                valid_r = math.isfinite(r) and (r <= self.max_range)
                last_r = ranges[cur[-1]] if cur else None
                jump_ok = True
                if cur and valid_r and abs(r - last_r) >= self.cluster_thresh:
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

            # Overlay sampling circle
            cx, cy, r_mid = self._compute_sampling_circle(display_frame.shape)
            overlay = display_frame.copy()
            cv2.circle(overlay, (cx, cy), r_mid, (255,255,255), 2)
            cv2.addWeighted(overlay, 0.5, display_frame, 0.5, 0, display_frame)

            # Pre-calc measure vector
            measure_rad_env = math.radians(self.target_heading % 360.0)
            Lx = math.cos(measure_rad_env); Ly = math.sin(measure_rad_env)

            # Compute side distance once
            if self.direction == "CLOCK":
                start_deg = 85 - yaw_synced_deg + self.target_heading
                end_deg   = 95 - yaw_synced_deg + self.target_heading
                d_side = sample_range(start_deg, end_deg)
            else:
                start_deg = 265 - yaw_synced_deg + self.target_heading
                end_deg   = 275 - yaw_synced_deg + self.target_heading
                d_side = sample_range(start_deg, end_deg)

            for c in clusters:
                if len(c) < 2:
                    continue
                rs = ranges[c]; angs_c = angs[c]
                r_avg = float(rs.mean())
                mid_rad = (angs_c[0] + angs_c[-1]) / 2.0

                obs_global_rad = math.radians(yaw) + mid_rad
                Ox = math.cos(obs_global_rad) * r_avg
                Oy = math.sin(obs_global_rad) * r_avg
                raw_perp = (Lx * Oy - Ly * Ox)
                if self.direction == "CLOCK":
                    new_perp = d_side - raw_perp
                else:
                    new_perp = d_side + raw_perp
                parallel = (Lx * Ox + Ly * Oy)

                # Corridor check
                if not (self.perp_min_dist <= new_perp <= self.perp_max_dist):
                    self.get_logger().debug(
                        f"Skipped cluster outside corridor: raw⊥={raw_perp:.2f}m, new⊥={new_perp:.2f}m "
                        f"not in [{self.perp_min_dist:.2f},{self.perp_max_dist:.2f}]"
                    )
                    continue

                # Width check
                width_m = abs(angs_c[-1] - angs_c[0]) * r_avg
                if not (self.min_diameter <= width_m <= self.max_diameter):
                    self.get_logger().info(
                        f"Dropped inside corridor for width {width_m:.3f}m (cluster at mid {math.degrees(mid_rad)%360:.1f}°)"
                    )
                    if self.calibration_mode and display_frame is not None:
                        (h,s,v),(x_px,y_px) = self._sample_hsv_range(frame, angs_c[0], angs_c[-1])
                        txt = f"H:{h:.0f} S:{s:.0f} V:{v:.0f}"
                        cv2.putText(display_frame, txt, (x_px+8,y_px+22),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (128,128,128), 2)
                        txt2 = f"new⊥:{new_perp:.2f} raw⊥:{raw_perp:.2f}"
                        cv2.putText(display_frame, txt2, (x_px+8, y_px+40),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (128,128,128), 2)
                    continue

                # Overlay angle range
                a_min = angs_c[0]; a_max = angs_c[-1]
                a_min_n = a_min % (2*math.pi); a_max_n = a_max % (2*math.pi)
                deg_min = math.degrees(a_min_n)%360.0; deg_max = math.degrees(a_max_n)%360.0
                diff_ang = (deg_max - deg_min) % 360.0
                if diff_ang > 180: diff_ang = 360.0 - diff_ang
                width_rad = math.radians(diff_ang)
                half_span = width_rad * 0.20 / 2.0
                offset_rad = math.radians(self.angle_offset_deg)
                center = mid_rad + offset_rad
                scan_angles = np.linspace(center-half_span, center+half_span, num=20) if half_span>0 else [center]
                pts = []
                for st in scan_angles:
                    img_theta = self._angle_to_image_theta(st)
                    x = int(cx + r_mid * math.cos(img_theta))
                    y = int(cy + r_mid * math.sin(img_theta))
                    pts.append((x,y))
                if pts:
                    cv2.polylines(display_frame, [np.array(pts, np.int32)], False, (255,0,0), 2)

                # Sample HSV
                (h,s,v),(x_px,y_px) = self._sample_hsv_range(frame, angs_c[0], angs_c[-1])
                hsv_info = f"H:{h:.0f} S:{s:.0f} V:{v:.0f}"
                if v < self.brightness_thresh:
                    self.get_logger().info(
                        f"Dropped inside corridor for brightness V={v:.1f} (cluster at mid {math.degrees(mid_rad)%360:.1f}°)"
                    )
                    if self.calibration_mode:
                        cv2.putText(display_frame, hsv_info, (x_px+8,y_px+22),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (128,128,128), 2)
                        txt2 = f"new⊥:{new_perp:.2f} raw⊥:{raw_perp:.2f}"
                        cv2.putText(display_frame, txt2, (x_px+8, y_px+40),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (128,128,128), 2)
                    continue

                # saturation thresholds
                m_g = (55-110)/0.9; b_g = 110 - m_g*0.1
                m_r = (55-120)/0.9; b_r = 110 - m_r*0.1
                s_g = m_g * r_avg + b_g
                s_r = m_r * r_avg + b_r
                color = None
                if (0 <= h <= 20 or 150 <= h <= 180) and s > s_r and v > 15:
                    color = 'RED'
                elif 38 <= h <= 100 and s >=0 and v > 10:
                    color = 'GREEN'
                else:
                    self.get_logger().info(
                        f"Dropped inside corridor for color thresholds H={h:.1f},S={s:.1f},V={v:.1f} "
                        f"(s_g={s_g:.1f}, s_r={s_r:.1f})"
                    )
                    if self.calibration_mode:
                        cv2.putText(display_frame, hsv_info, (x_px+8,y_px+22),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (128,128,128), 2)
                        txt2 = f"new⊥:{new_perp:.2f} raw⊥:{raw_perp:.2f}"
                        cv2.putText(display_frame, txt2, (x_px+8, y_px+40),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (128,128,128), 2)
                    continue

                detected.append({
                    'mid_rad': mid_rad,
                    'middle_angle_deg': math.degrees(mid_rad) % 360,
                    'distance': r_avg,
                    'color': color,
                    'x_px': x_px, 'y_px': y_px,
                    'hsv': (h,s,v),
                    'raw_perp': raw_perp,
                    'new_perp': new_perp,
                    'parallel': parallel,
                })
                if self.calibration_mode:
                    txt1 = f"{color} {hsv_info}"
                    txt2 = f"new⊥:{new_perp:.2f} raw⊥:{raw_perp:.2f}"
                    txt3 = f"par:{parallel:.2f}"
                    cv2.putText(display_frame, txt1, (x_px+8, y_px-8),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,0), 2)
                    cv2.putText(display_frame, txt2, (x_px+8, y_px+22),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,0), 2)
                    cv2.putText(display_frame, txt3, (x_px+8, y_px+40),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,0), 2)

            # SEARCH → DRIVE after enough scans
            if self.obs_status == "SEARCH":
                self.get_logger().debug("SEARCH count: %d" % self.search_count)
                if self.search_count >= self.search_required:
                    self.obs_status = "DRIVE"
                    self.get_logger().info("obs_status set to DRIVE after SEARCH scans")
                    self.search_count = 0

            # Selection among detected obstacles
            if detected:
                if len(detected) == 1:
                    selected = detected[0]
                else:
                    best_val = None; best_obs = None
                    for obs in detected:
                        obs_global_rad = math.radians(yaw) + obs['mid_rad']
                        Ox_u = math.cos(obs_global_rad)
                        Oy_u = math.sin(obs_global_rad)
                        cross = Lx*Oy_u - Ly*Ox_u
                        val = cross if self.direction == 'CLOCK' else -cross
                        if best_val is None or val > best_val:
                            best_val = val; best_obs = obs
                    selected = best_obs

                # Determine candidate obs_drive_state
                color = selected['color']
                np_ = selected['new_perp']
                state_cand = f"{color}_{'CLOSE' if np_ < 0.50 else 'FAR'}"
                if self.obs_status in ("SEARCH","DRIVE"):
                    self.state_history.append(state_cand)
                    cnt = Counter(self.state_history)
                    if self.obs_drive_state is None:
                        if len(self.state_history) >= 8:
                            for st, count in cnt.items():
                                if st is not None and count >= 6:
                                    if self.obs_drive_state != st:
                                        prev = self.obs_drive_state
                                        self.obs_drive_state = st
                                        self.get_logger().info(f"obs_drive_state changed from {prev} to {st} by consensus")
                                    break
                    else:
                        if len(self.state_history) >= 12:
                            for st, count in cnt.items():
                                if st is not None and count >= 10:
                                    if self.obs_drive_state != st:
                                        prev = self.obs_drive_state
                                        self.obs_drive_state = st
                                        self.get_logger().info(f"obs_drive_state changed from {prev} to {st} by consensus")
                                    break

                self.get_logger().info(
                    f"Selected @ {selected['middle_angle_deg']:.1f}°, r={selected['distance']:.2f}m, "
                    f"raw⊥={selected['raw_perp']:.2f}m, new⊥={selected['new_perp']:.2f}m, "
                    f"par={selected['parallel']:.2f}m, color {selected['color']}, hsv={selected['hsv']}"
                )

                # Determine obs_status PASSED if parallel distance < pass_thresh
                pass_thresh = 0.04
                if self.obs_drive_state is not None:
                    if self.section_count % 4 == 0:
                        if (self.direction == "CLOCK" and self.obs_drive_state == "GREEN_CLOSE"):
                            pass_thresh = -0.1
                        elif (self.direction == "COUNTER" and self.obs_drive_state == "RED_CLOSE"):
                            pass_thresh = -0.015
                if self.obs_status not in ("PASSED","TURN"):
                    if selected['parallel'] < pass_thresh:
                        self.obs_status = "PASSED"
                        self.get_logger().info(f"obs_status set to PASSED (parallel {selected['parallel']:.2f} < {pass_thresh})")

                self.get_logger().info(f"Current selected color: {selected['color']}")
            else:
                if self.mode == 'OBS':
                    self.get_logger().info("No detected obstacle in corridor")

            if self.calibration_mode and display_frame is not None:
                os.makedirs(self.calib_folder, exist_ok=True)
                fname = os.path.join(self.calib_folder,
                                     f"frame_{self._calib_count % self.calib_max_frames:02d}.png")
                cv2.imwrite(fname, display_frame)
                self._calib_count += 1

        # Control logic
        speed = 0.0; steer = 0.0; m = self.obs_status

        # Mode transitions for FREE if needed...
        if self.mode == "FREE":
            try:
                yaw_now = self.drive_base.get_continuous_yaw()
            except:
                yaw_now = 0.0
            if self.direction == "CLOCK" and (yaw_now < -1070):
                self.donetimes += 1
                if self.donetimes >= 6:
                    self.obs_status = "DONE"
            elif self.direction == "COUNTER" and (yaw_now > 1065):
                self.donetimes += 1
                if self.donetimes >= 8:
                    self.obs_status = "DONE"

        # START mode logic
        if self.obs_status == "START":
            d_l = sample_range(85, 95)
            d_r = sample_range(265, 275)
            d_f = sample_range(357, 360)
            result = None
            if self.mode == "FREE":
                if d_l < d_r:
                    result = "COUNTER"
                elif d_l > d_r:
                    result = "CLOCK"
                else:
                    result = "UNKNOWN"
            elif self.mode == "OBS":
                if d_f < 0.25:
                    if d_l < d_r:
                        result = "PARK_CLOCK"
                    elif d_l > d_r:
                        result = "PARK_COUNTER"
                    else:
                        result = "PARK_UNKNOWN"
                else:
                    if d_l < d_r:
                        result = "COUNTER"
                    elif d_l > d_r:
                        result = "CLOCK"
                    else:
                        result = "UNKNOWN"
            else:
                result = "UNKNOWN"
            self.start_decisions.append(result)
            self.start_batch_count += 1
            if self.start_batch_count >= self.start_batch_size:
                if len(set(self.start_decisions)) == 1:
                    dec = self.start_decisions[0]
                    self.get_logger().info(f"START decision: {dec}")
                    if dec.startswith("PARK_"):
                        self.parking = True
                        self.direction = dec.split("_")[1]
                    elif dec in ("CLOCK","COUNTER"):
                        self.parking = False
                        self.direction = dec
                    self.obs_status = "WAIT"
                    self.wait_pressed_time = None
                else:
                    self.get_logger().info(f"START mixed results {self.start_decisions}, retrying")
                self.start_decisions.clear()
                self.start_batch_count = 0
            speed = 0.0; steer = 0.0

        # WAIT mode
        elif self.obs_status == "WAIT":
            # Do nothing until button pressed for ≥3s
            try:
                pressed = not self.drive_base.get_button_state()
            except Exception:
                pressed = False
            if pressed:
                if self.wait_pressed_time is None:
                    self.wait_pressed_time = time.time()
                else:
                    if time.time() - self.wait_pressed_time >= 3.0:
                        if self.mode == "OBS":
                            if self.parking:
                                self.obs_status = "UNPARK_1"
                            else:
                                self.obs_status = "SEARCH"
                        elif self.mode == "FREE":
                            self.obs_status = "N"
                        self.get_logger().info(f"WAIT → {self.obs_status}")
            else:
                self.wait_pressed_time = None
            speed = 0.0; steer = 0.0

        elif m == "W_CLCW":
            rights  = [sample(a) for a in self.right_side_angles]
            fronts  = [sample(a) for a in self.right_front_angles]
            fin_r   = [d for d in rights  if d]
            fin_f   = [d for d in fronts if d]
            d_r     = sum(fin_r)/len(fin_r) if fin_r else float('inf')
            error   = 0.40 - d_r
            ang     = self.kp_lateral * error
            factor = max(0.6, 1.0 - min(abs(error)/1.4, 1.0))
            speed = self.v_max * factor
            steer = ang

        elif m == "W_COUW":
            lefts  = [sample(a) for a in self.left_side_angles]
            fronts = [sample(a) for a in self.left_front_angles]
            fin_l  = [d for d in lefts  if d]
            fin_f  = [d for d in fronts if d]
            d_l    = sum(fin_l)/len(fin_l) if fin_l else float('inf')
            d_f    = sum(fin_f)/len(fin_f) if fin_f else float('inf')
            error  = -1*(0.26 - d_l)
            gain   = 0.7 if abs(error) < 0.32 else 0.2
            ang    = self.kp_heading * gain * error
            factor = max(0.4, 1.0 - min(abs(error)/1.3, 1.0)) * 0.8
            if d_f < 0.20:
                factor = 0.2
            speed = self.v_max * factor
            steer = ang

        elif self.mode == "FREE" and self.direction == "CLOCK" and self.obs_status != "DONE":
            threshold = 0.90; corner = 0.0
            lefts  = [sample(math.radians(a)) for a in range(28,60)]
            fronts = [sample(math.radians(a)) for a in range(355,360)]
            backs  = [sample(math.radians(a)) for a in range(33,62)]
            fin_l = [d for d in lefts if d]
            fin_f = [d for d in fronts if d]
            fin_b = [d for d in backs if d]
            d_l = sum(fin_l)/len(fin_l) if fin_l else float('inf')
            d_f = sum(fin_f)/len(fin_f) if fin_f else float('inf')
            self.get_logger().info(f"seeing yaw_synced_deg: {yaw_synced_deg}")
            if d_f < threshold: corner = threshold - d_f
            error = -0.8*(0.46 + 1.3*corner - d_l)
            ang = self.kp_lateral * error
            cap = 0.6 if d_f >= 1.05 else min(0.1 + 0.3*d_f, 0.6)
            ang = min(ang, cap) if ang >= 0 else ang
            ang -= corner * 1.77
            if d_f <= 0.32: ang -= 0.12
            factor = max(0.8, min(1.0, 0.13 / abs(0.9 * error))) if error != 0 else 1.0
            speed = self.v_max * factor
            steer = -ang

        elif self.mode == "FREE" and self.direction == "COUNTER" and self.obs_status != "DONE":
            threshold = 0.90; corner = 0.0
            rights  = [sample(math.radians(a)) for a in range(300,332)]
            fronts = [sample(math.radians(a)) for a in range(0,6)]
            backs  = [sample(math.radians(a)) for a in range(33,62)]
            fin_r = [d for d in rights if d]
            fin_f = [d for d in fronts if d]
            fin_b = [d for d in backs if d]
            d_r = sum(fin_r)/len(fin_r) if fin_r else float('inf')
            d_f = sum(fin_f)/len(fin_f) if fin_f else float('inf')
            self.get_logger().info(f"seeing yaw_synced_deg: {yaw_synced_deg}")
            if d_f < threshold: corner = threshold - d_f
            error = -0.8*(0.49 + 1.24*corner - d_r)
            ang = self.kp_lateral * error
            cap = 0.6 if d_f >= 1.05 else min(0.1 + 0.3*d_f, 1.0)
            ang = min(ang, cap) if ang >= 0 else ang
            ang -= corner * 1.77
            if d_f <= 0.32: ang -= 0.12
            factor = max(0.8, min(1.0, 0.13 / abs(0.9 * error))) if error != 0 else 1.0
            speed = self.v_max * factor
            steer = ang

        elif self.mode == "OBS" and self.obs_status == "DRIVE":
            self.max_left = math.radians(65)
            self.max_right = math.radians(55)
            turn_limiter_enable = True

            d_mode = self.obs_drive_state or ""
            d_dir = self.direction
            # New target_distance logic:
            if (d_dir == "CLOCK" and d_mode == "RED_FAR") or (d_dir == "COUNTER" and d_mode == "GREEN_FAR"):
                target_distance = 0.72
            elif (d_dir == "CLOCK" and d_mode == "RED_CLOSE") or (d_dir == "COUNTER" and d_mode == "GREEN_CLOSE"):
                target_distance = 0.62
            elif (d_dir == "CLOCK" and d_mode == "GREEN_FAR") or (d_dir == "COUNTER" and d_mode == "RED_FAR"):
                target_distance = 0.38
            elif (d_dir == "CLOCK" and d_mode == "GREEN_CLOSE") or (d_dir == "COUNTER" and d_mode == "RED_CLOSE"):
                if self.section_count % 4 == 0:
                    if (d_dir == "CLOCK" and d_mode == "GREEN_CLOSE"):
                        target_distance = 0.284
                    else:
                        target_distance = 0.291
                else:
                    target_distance = 0.24
            else:
                target_distance = 0.5

            if self.direction == "CLOCK":
                start_deg = 85 - yaw_synced_deg + self.target_heading
                end_deg   = 95 - yaw_synced_deg + self.target_heading
                d_l = sample_range(start_deg, end_deg)
                lateral_error = target_distance - d_l
            else:
                start_deg = 265 - yaw_synced_deg + self.target_heading
                end_deg   = 275 - yaw_synced_deg + self.target_heading
                d_l = sample_range(start_deg, end_deg)
                lateral_error = -(target_distance - d_l)

            self.get_logger().debug("dl_: %.2f" % d_l)
            if self.last_d is None:
                self.last_d = lateral_error
            else:
                if abs(lateral_error - self.last_d) > 0.09:
                    lateral_error = self.last_d + (lateral_error - self.last_d)/2
                self.last_d = lateral_error

            heading_error = -(self.target_heading - yaw_current_deg)

            

            if abs(lateral_error) > 0.07:
                steer_nominal = 4.1 * lateral_error
            else:
                if ((d_dir == "CLOCK" and d_mode == "GREEN_CLOSE") or (d_dir == "COUNTER" and d_mode == "RED_CLOSE")) and self.section_count % 4 == 0:
                    if (selected is not None) and abs(selected['parallel']) < 0.35:
                        steer_nominal = 0.035 * heading_error + 6.8 * lateral_error
                    else:
                        steer_nominal = 0.033 * heading_error + 5.8 * lateral_error
                else:
                    steer_nominal = 0.039 * heading_error + 2.8 * lateral_error
            abs_st = abs(steer_nominal)
            if abs_st <= 0.1:
                speed_steer = self.speed_max
            elif abs_st >= 0.5:
                speed_steer = self.speed_min
            else:
                frac = (abs_st - 0.1) / (0.5 - 0.1)
                speed_steer = self.speed_max + (self.speed_min - self.speed_max) * frac

            # front distance dependent speed capping, similar to PASSED
            start_f = 357 - yaw_synced_deg + self.target_heading
            end_f   = 360 - yaw_synced_deg + self.target_heading
            d_front = sample_range(start_f, end_f)
            if d_front >= 0.9:
                speed_front = self.speed_max
            elif d_front <= 0.2:
                speed_front = self.speed_min
            else:
                frac2 = (d_front - 0.2) / (1.3 - 0.0)
                speed_front = self.speed_min + (self.speed_max - self.speed_min) * frac2
            speed = min(speed_steer, speed_front)
            steer = steer_nominal

            print("le " + str(lateral_error) + " he " + str(heading_error) + " sn " + str(steer_nominal) + " td " + str(target_distance))

            # front obstacle brake-to-PASSED
            if d_front < 0.8 and abs(steer_nominal) < 0.22 and selected is None:
                speed = -0.42
                if self.stop_times < 6:
                    self.stop_times += 1
                else:
                    self.stop_times = 0
                    self.obs_status = "PASSED"
            else:
                self.stop_times = 0

            # dynamic capping for GREEN_CLOSE/RED_CLOSE in certain sections
            if (d_dir == "CLOCK" and d_mode == "GREEN_CLOSE") or (d_dir == "COUNTER" and d_mode == "RED_CLOSE"):
                if self.section_count % 4 == 0 and selected is not None:
                    if abs(selected['parallel']) < 0.5:
                        speed_cap = 0.47
                    elif abs(selected['parallel']) > 0.7:
                        speed_cap = self.speed_max
                    else:
                        frac_sp = (abs(selected['parallel']) - 0.35) / (0.7 - 0.35)
                        speed_cap = 0.52 + (self.speed_max - 0.52) * frac_sp
                    speed = min(speed, speed_cap)

            # Apply CAU speed cap if in CAU mode
            if self.coll_avd == "CAU":
                speed = min(speed, self.speed_min + 0.04)

            if turn_limiter_enable:
                steer = self.apply_turn_limiter(steer, math.radians(self.target_heading), yaw_current_rad)

        elif self.mode == "OBS" and self.obs_status == "PASSED":
            heading_error = -(self.target_heading - yaw_current_deg)
            if self.direction == "CLOCK":
                start_deg = 85 - yaw_synced_deg + self.target_heading
                end_deg   = 94 - yaw_synced_deg + self.target_heading
                d_l = sample_range(start_deg, end_deg)
                lateral_error = 0.50 - d_l
            else:
                start_deg = 265 - yaw_synced_deg + self.target_heading
                end_deg   = 275 - yaw_synced_deg + self.target_heading
                d_l = sample_range(start_deg, end_deg)
                lateral_error = -(0.50 - d_l)

            if abs(lateral_error) > 0.07:
                steer_nominal = 5.4 * lateral_error
            else:
                steer_nominal = 0.039 * heading_error
            abs_st = abs(steer_nominal)
            if abs_st <= 0.1:
                speed_steer = self.speed_max
            elif abs_st >= 0.5:
                speed_steer = self.speed_min
            else:
                frac = (abs_st - 0.1) / (0.5 - 0.1)
                speed_steer = self.speed_max + (self.speed_min - self.speed_max) * frac

            start_f = 355 - yaw_synced_deg + self.target_heading
            end_f   = 360 - yaw_synced_deg + self.target_heading
            d_front = sample_range(start_f, end_f)
            if d_front >= 0.8:
                speed_front = self.speed_max
            elif d_front <= 0.1:
                speed_front = self.speed_min
            else:
                frac2 = (d_front - 0.2) / (1.2 - 0.0)
                speed_front = self.speed_min + (self.speed_max - self.speed_min) * frac2
            speed = min(speed_steer, speed_front)
            steer = steer_nominal

            if d_front < 0.18:
                speed = -0.4
                if self.stop_times < 5:
                    self.stop_times += 1
                else:
                    self.stop_times = 0
                    self.obs_status = "TURN"
            else:
                self.stop_times = 0

            self.max_left = math.radians(55)
            self.max_right = math.radians(55)
            turn_limiter_enable = True

            # Apply CAU cap if active
            if self.coll_avd == "CAU":
                speed = min(speed, self.speed_min + 0.02)

            if turn_limiter_enable:
                steer = self.apply_turn_limiter(steer, math.radians(self.target_heading), yaw_current_rad)

        elif self.obs_status == "DONE":
            if self.done_counter <= 12:
                speed = -0.32
                steer = 0.0
                self.done_counter += 1
            else:
                speed = 0.0
                steer = 0.0
        else:
            speed = 0.0; steer = 0.0

        # Section 12 stop behavior (preserved from before)
        if self.mode == "OBS" and self.section_count == 12:
            start_f = 90 - yaw_synced_deg + self.target_heading
            end_f   = 110 - yaw_synced_deg + self.target_heading
            d_lb = sample_range(start_f, end_f)
            start_f = 250 - yaw_synced_deg + self.target_heading
            end_f   = 270 - yaw_synced_deg + self.target_heading
            d_rb = sample_range(start_f, end_f)
            if 0.5 < (d_lb + d_rb) < 1.05:
                speed = -0.22
                if self.lapstop_counter <= 11:
                    self.lapstop_counter += 1
                else:
                    speed = 0
            else:
                self.lapstop_counter = 0

        # Clamp and apply
        speed = max(min(speed, 1.0), -1.0)
        steer = max(min(steer, 1.0), -1.0)

        self.prev_speed = speed

        elapsed = time.time() - lidar_ts
        self.get_logger().info(
            f"Mode: {self.mode}, obs_drive_state: {self.obs_drive_state}, obs_status: {self.obs_status}, "
            f"Coll_avd: {self.coll_avd}, Speed: {speed:.3f}, Steer: {steer:.3f}; "
            f"scan_cb elapsed: {elapsed:.3f}s"
        )
        try:
            self.drive_base.set_speed(speed)
            self.drive_base.set_steering(steer)
        except Exception as e:
            self.get_logger().error(f"DriveBase command failed: {e}")

    # --------------- TURN control method ---------------
    def _turn_control(self):
        # --- Button check in turn_control ---
        if self.wait_pressed_time is not None:
            try:
                btn_state = self.drive_base.get_button_state()
            except Exception:
                btn_state = True  # assume released if error
            if btn_state:  # released/high -> stop
                try:
                    self.drive_base.set_speed(0.0)
                    self.drive_base.set_steering(0.0)
                except Exception:
                    pass
                self.get_logger().info("Button released after start: stopping and skipping turn processing")
                return

        if not rclpy.ok(): return
        if self.obs_status != "TURN": return
        try:
            yaw_current_deg = self.drive_base.get_continuous_yaw()
        except Exception:
            return
        dir_c = self.direction
        angle_offset = 90
        if dir_c == "CLOCK":
            desired = self.target_heading - angle_offset
            steer = -1.0
            err = yaw_current_deg - desired
        else:
            desired = self.target_heading + angle_offset
            steer = 1.0
            err = desired - yaw_current_deg

        # Speed control
        turn_min_speed = 0.52
        turn_max_speed = 0.6
        k_turn = 0.01
        spd = k_turn * abs(err)
        if spd < turn_min_speed:
            spd = turn_min_speed
        elif spd > turn_max_speed:
            spd = turn_max_speed
        speed = -spd  # always backwards

        # Stop if within tolerance or overshot
        if self.direction == "CLOCK":
            if yaw_current_deg <= desired + self.turn_tolerance_deg:
                stop_turn = True
            else:
                stop_turn = False
        else:
            if yaw_current_deg >= desired - self.turn_tolerance_deg:
                stop_turn = True
            else:
                stop_turn = False

        if stop_turn:
            speed = 0.3
            steer = 0.0
            if self.stop_times < 5:
                self.stop_times += 1
            else:
                self.stop_times = 0
                old_th = self.target_heading
                if dir_c == "CLOCK":
                    self.target_heading = old_th - angle_offset
                else:
                    self.target_heading = old_th + angle_offset
                self.obs_status = "ALIGN"
                self.align_scans.clear()
                self.get_logger().info(f"TURN complete: set obs_status=ALIGN, theoretical target_heading={self.target_heading:.1f}°")

        speed = max(min(speed, 1.0), -1.0)
        steer = max(min(steer, 1.0), -1.0)
        try:
            self.drive_base.set_speed(speed)
            self.drive_base.set_steering(steer)
        except Exception as e:
            self.get_logger().error(f"TURN control DriveBase failed: {e}")
        self.get_logger().debug(f"TURN loop: yaw={yaw_current_deg:.1f}°, desired={desired:.1f}°, err={err:.1f}°, speed={speed:.2f}, steer={steer:.2f}")

    def destroy_node(self):
        self._led_thread_stop = True
        time.sleep(0.1)
        try:
            self.drive_base.set_steering(0)
            time.sleep(0.2)
            self.drive_base.cleanup_all()
            self.get_logger().info("DriveBase cleaned up")
            if self.cap and self.cap.isOpened():
                self.cap.release()
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")
        try:
            self.drive_base.set_led(False)
        except:
            pass
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CombinedFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
