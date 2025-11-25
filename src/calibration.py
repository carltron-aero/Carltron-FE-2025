
#  This script is exactly the same as combinedfollower, just for tuning


#!/usr/bin/env python3
import os
import math
import time
import threading
from collections import deque, Counter

import numpy as np
import cv2
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from drivebase import DriveBase
from cammanager import CamManager

class CombinedFollower(Node):
    def __init__(self):
        super().__init__('combined_follower')

        self.imu_sync_delay_ms = 0.0

        #  --- Instantiate DriveBase and IMU setup ---
        try:
            self.drive_base = DriveBase()
            self.drive_base.start()
            self.drive_base.set_yaw_zero()
        except Exception as e:
            self.get_logger().warning(f"DriveBase/IMU setup failed: {e}")
    
        PREVIEW_LIVE = False

        ANGLE_DEG = 180.0
        WIDTH_DEG = 10.0

        self.mgr = CamManager(preview_on=PREVIEW_LIVE,
                     patch_angle_deg=ANGLE_DEG,
                     patch_width_deg=WIDTH_DEG,
                     debug_save_dir="/home/carl/tmp/cammanager_debug",
                     debug_max_previews=33)

        # --- State variables ---
        self.mode = 'OBS' # OBS or FREE
        self.direction = "CLOCK"  # or "COUNTER"
        self.obs_status = "SEARCH"
        self.obs_drive_state = None
        self.parking = False
        self.starttime_ts = 0
        # For consensus: keep last states, up to 12
        self.state_history = deque(maxlen=12)
        self.prev_obs_d_state = None
        # SEARCH counting
        self.search_count = 0
        self.search_required = 8  # exit SEARCH after 12 scans
        # section counting
        self.section_count = 0    # Reset to base value 0
        self.free_r_target_d = 0.28
        self.free_r_ac_d = 0

        # Collision avoidance
        self.coll_avd = "N"
        self.coll_count = 0
        self.coll_avd_ts = None

        # For OBS PASSED heading target
        self.start_yaw = None           # in radians
        self.target_heading = 0.0       # in degrees
        self.turn_park_target = 0
        self.last_d = None
        self.passed_confirm = False

        # Limits in radians for soft limiting
        self.max_left = math.radians(55)
        self.max_right = math.radians(55)

        # Sampling angles for other controllers
        self.left_side_angles   = [math.radians(a) for a in (96,103,108,124)]
        self.left_front_angles  = [math.radians(a) for a in (128,125)]
        self.right_side_angles  = [math.radians(a) for a in (245,242,237)]
        self.right_front_angles = [math.radians(a) for a in (228,235)]

        # Controller tuning variables
        self.v_max_free_r = 0.65
        self.obs_v = 0.67
        self.kp_heading = 1.0
        self.kp_lateral = 1.0
        self.angle_limit_corr_gain = 4.55 #previously 0.55
        self.turn_limiter_enable = True

        # Speed limits (tunable)
        self.speed_max = 0.65
        self.speed_min = 0.22

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
        self.perp_min_dist = 0.33
        self.perp_max_dist = 0.67

        # ALIGN tuning
        self.align_scans = []
        self.align_max = 8
        self.align_sector_half_deg = 12
        self.align_offset_deg = 1.6  # subtract in alignment
        self.turn_tolerance_deg = 6  # tolerance for stopping turn

        # Initialize counters
        self.donetimes = 0
        self.stop_times = 0
        self.stops_times = 0
        self.done_counter = 0
        self.lapstop_counter = 0
        self.parkstart_times = 0
        self.parkstop_times = 0
        self.turn_starttimes = 0
        self.save_status = ""

        # START logic variables
        self.start_decisions = []
        self.start_batch_count = 0
        self.start_batch_size = 10
        self.wait_pressed_time = None
        self.stop_time = None
        self.park_active  = False

        # Cached scan-angle arrays
        self.prev_angle_min = None
        self.prev_angle_increment = None
        self.cached_angs = None
        self.cached_degs = None
        self.cached_sector_mask = None

        # Subscriptions
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(String, '/mode', self.mode_cb, 10)

        # Turn control timer at 50 Hz
        self.create_timer(1.0/140.0, self._turn_control)

        # LED thread
        self._led_thread_stop = False
        threading.Thread(target=self._led_thread_func, daemon=True).start()

        self.prev_speed = 0.0

        self.get_logger().info('CombinedFollower started.')




    def _angle_to_image_theta(self, scan_rad):
        deg = math.degrees(scan_rad) % 360.0
        img_deg = (270.0 + deg) % 360.0
        return math.radians(img_deg)


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
        #self.wait_pressed_time = None

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
                # Optionally reset LED state here if you want
                # led_state = False

            if status == "START":
                # LED on permanently
                if not led_state:
                    led_state = True
                    try:
                        self.drive_base.set_rear_led(1.0)
                    except:
                        pass
                time.sleep(0.1)
                continue

            elif status == "WAIT":
                # Smooth breathing: 1s from off -> full, 1s from full -> off
                acc += dt
                period = 2.2  # total period: 2s (1s up, 1s down)
                phase = (acc % period) / period  # 0..1

                if phase < 0.5:
                    # first 1s: go 0 -> 1
                    brightness = phase * 2.0
                else:
                    # second 1s: go 1 -> 0
                    brightness = (1.0 - phase) * 2.0

                try:
                    self.drive_base.set_rear_led(brightness)
                    self.drive_base.set_front_led(brightness*0.3)
                except:
                    pass

                # smaller sleep for smoother animation
                time.sleep(0.02)
                continue

            else:
                # blink once every 2 seconds
                if status not in ("SEARCH", "ALIGN"):
                    try:
                        self.drive_base.set_front_led(0.03)
                    except:
                        pass

                acc += dt
                if not led_state:
                    if acc >= 2.0:
                        led_state = True
                        try:
                            self.drive_base.set_rear_led(1.0)
                        except:
                            pass
                        acc = 0.0
                else:
                    if acc >= slow_on:
                        led_state = False
                        try:
                            self.drive_base.set_rear_led(0.1)
                        except:
                            pass
                        acc = 0.0

                time.sleep(0.1)
                continue

    # --------------- Main scan callback ---------------
    def scan_cb(self, scan: LaserScan):
        t0 = time.time()
        # LiDAR timestamp
        stamp_secs = scan.header.stamp.sec
        stamp_nano = scan.header.stamp.nanosec
        lidar_ts = stamp_secs + stamp_nano * 1e-9
        #self.get_logger().info(f"LiDAR timestamp: {stamp_secs}.{stamp_nano:09d} ({lidar_ts:.6f}s)")

        # Button check: if waiting context and button released, skip
        #print(self.wait_pressed_time)
        if self.wait_pressed_time is not None:
            try:
                btn_state = self.drive_base.get_switch_state()
            except Exception:
                btn_state = False
            if not btn_state:
                try:
                    self.drive_base.set_target_speed(0.0)
                    self.drive_base.brake(1)
                    self.drive_base.set_steering(0.0)
                except:
                    pass
                self.get_logger().info("Button released after start: stopping and skipping scan processing")
                return

        # Prepare LIDAR sampling arrays, with caching if same scan geometry
        ranges = np.array(scan.ranges)
        a0, da = scan.angle_min, scan.angle_increment
        N = len(ranges)
        if (self.prev_angle_min == a0) and (self.prev_angle_increment == da) and (self.cached_angs is not None):
            angs = self.cached_angs
            degs = self.cached_degs
            sector_mask = self.cached_sector_mask
        else:
            angs = a0 + np.arange(N)*da
            degs = (np.degrees(angs) % 360.0)
            # sector mask
            if self.sector_min_deg <= self.sector_max_deg:
                sector_mask = (degs >= self.sector_min_deg) & (degs <= self.sector_max_deg)
            else:
                sector_mask = (degs >= self.sector_min_deg) | (degs <= self.sector_max_deg)
            # cache
            self.prev_angle_min = a0
            self.prev_angle_increment = da
            self.cached_angs = angs
            self.cached_degs = degs
            self.cached_sector_mask = sector_mask

        clean = ranges.copy()  # no cleaning logic now
        self.da = da  # for possible external use

        # Define sample() using clean (unchanged)
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
            i_start = int(start_deg)
            i_end = int(end_deg)
            for a in range(i_start, i_end):
                r = sample(math.radians(a))
                if r:
                    vals.append(r)
            return (sum(vals)/len(vals)) if vals else float('inf')

        # Get synced yaw & current yaw
        target_imu_ts = lidar_ts - (self.imu_sync_delay_ms / 1000.0)
        yaw_val = self.drive_base.get_yaw_at(target_imu_ts)
        if yaw_val is None:
            self.get_logger().warning("No IMU yaw available for sync; using current yaw")
            try:
                yaw = self.drive_base.get_continuous_yaw()
            except Exception:
                yaw = 0.0
        else:
            yaw = yaw_val
            #self.get_logger().info(f"LIDAR-timed yaw: {yaw:.1f}° (diff {diff_imu*1000:.1f}ms)")
        yaw_synced_deg = yaw
        try:
            yaw_current_deg = self.drive_base.get_continuous_yaw()
        except Exception:
            yaw_current_deg = yaw_synced_deg
        yaw_current_rad = math.radians(yaw_current_deg)

        # --- START mode logic ---
        if self.obs_status == "START":
            # START mode: collect batches of scans, decide direction/parking
            d_l = sample_range(85, 95)
            d_r = sample_range(265, 275)
            d_f = sample_range(357, 360)
            # Determine result
            if self.mode == "FREE":
                if (1.25 <= d_f <= 1.45) or (1.75 <= d_f <= 1.95):
                    result = "COUNTER"
                elif (1.0 <= d_f <= 1.2) or (1.5 <= d_f <= 1.7):
                    result = "CLOCK"
                else:
                    result = "UNKNOWN"
                print(result)
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
                    else:
                        # UNKNOWN or PARK_UNKNOWN: remain in START, will retry
                        pass
                    # Move to WAIT if a definite direction (or parking) decided:
                    if dec not in ("UNKNOWN","PARK_UNKNOWN"):
                        self.obs_status = "WAIT"
                        self.wait_pressed_time = None
                else:
                    self.get_logger().info(f"START mixed results {self.start_decisions}, retrying")
                self.start_decisions.clear()
                self.start_batch_count = 0
            # In START mode, do not proceed further
            try:
                self.drive_base.set_target_speed(0.0)
                self.drive_base.set_steering(0.0)
            except:
                pass
            return

        # --- WAIT mode logic ---
        if self.obs_status == "WAIT":
            # Do nothing until button pressed for ≥3s
            #print("Waiting")

            try:
                btn = self.drive_base.get_switch_state()
                pressed = btn  # get_switch_state() == False means pressed
            except Exception:
                pressed = False
            if pressed:
                if self.wait_pressed_time is None:
                    self.wait_pressed_time = time.time()
                    self.starttime_ts = time.time()
                else:
                    if time.time() - self.wait_pressed_time >= 2.2:
                        # Transition out of WAIT
                        if self.mode == "OBS":
                            if self.parking:
                                self.obs_status = "UNPARK_1"
                            else:
                                self.obs_status = "SEARCH"
                        elif self.mode == "FREE":
                            self.obs_status = "N"
                        self.get_logger().info(f"WAIT → {self.obs_status}")
                        # Clear timestamp so future button logic restarts if re-entered WAIT
                        #self.wait_pressed_time = None
            else:
                # Button released: reset timer
                self.wait_pressed_time = None
            # In WAIT mode, do not proceed further
            try:
                self.drive_base.set_target_speed(0.0)
                self.drive_base.set_steering(0.0)
            except:
                pass
            return

        # --- Collision avoidance early (primary AVD) ---
        ps = self.prev_speed
        if self.mode == "OBS" and self.obs_status not in ("START","WAIT","UNPARK_1","UNPARK_2","PARKING_2") and self.coll_avd != "AVD" and self.prev_speed > 0.2:
            # dynamic forward limit x_max based on prev_speed: from ~0.14 at speed_min to 0.22 at 0.6
            vmin = self.speed_min
            vmax = 0.6
            if ps <= vmin:
                x_max = 0.14
            elif ps >= vmax:
                x_max = 0.22
            else:
                x_max = 0.14 + (ps - vmin) / (vmax - vmin) * (0.22 - 0.14)

            

            correction_deg = 0
            valid_pts = 0
            sum_y = 0.0
            d_mode = self.obs_drive_state or ""
            d_dir = self.direction
            for r, ang in zip(ranges, angs):
                if not (math.isfinite(r) and r >= 0.04):
                    continue
                x = r * math.cos(ang)
                # tighter x_limit in certain state
                if ((d_dir == "CLOCK" and d_mode == "GREEN_CLOSE") or (d_dir == "COUNTER" and d_mode == "RED_CLOSE")) \
                   and self.section_count % 4 == 0 and self.obs_status == "DRIVE":

                    correction_deg = 2.2
                    
                    if self.prev_speed > 0.5:
                        x_limit = 0.284
                    else:
                        x_limit = 0.255
                    if x < 0.0 or x > x_limit:
                        continue
                else:
                    if x < 0.0 or x > x_max:
                        continue
                y = r * math.sin(ang)
                # tighter y limit in certain state
                if ((d_dir == "CLOCK" and d_mode == "GREEN_CLOSE") or (d_dir == "COUNTER" and d_mode == "RED_CLOSE")) \
                   and self.section_count % 4 == 0:
                    if (d_dir == "CLOCK" and d_mode == "GREEN_CLOSE") and (not (-0.063 < y < 0.075)):
                        continue
                    if (d_dir == "COUNTER" and d_mode == "RED_CLOSE") and (not (-0.075 < y < 0.063)):
                        continue
                else:
                    if abs(y) > 0.075:
                        continue
                valid_pts += 1
                sum_y += y
                if valid_pts >= 10:
                    break
            if valid_pts >= 10:
                # primary avoidance triggered
                self.coll_count += 1
                try:
                    self.drive_base.set_target_speed(-0.4)
                    self.drive_base.set_steering(0.0)
                except Exception:
                    pass
                # If reached threshold, enter AVD
                if self.coll_count >= 4 and self.coll_avd != "AVD":
                    avg_y = sum_y / valid_pts if valid_pts > 0 else 0.0
                    if avg_y > 0.01:
                        #self.target_heading -= correction_deg
                        self.get_logger().info(f"AVD adjustment: avg_y={avg_y:.3f}>0 → subtract 2.2° → new target_heading={self.target_heading:.2f}")
                    elif avg_y < -0.01:
                        #self.target_heading += correction_deg
                        self.get_logger().info(f"AVD adjustment: avg_y={avg_y:.3f}<0 → add 2.2° → new target_heading={self.target_heading:.2f}")
                    else:
                        self.get_logger().info(f"AVD adjustment: avg_y={avg_y:.3f}≈0 → no heading change")
                    self.coll_avd = "AVD"
                    self.coll_avd_ts = time.time()
                    self.get_logger().info("Collision avoidance: entering AVD mode")
                return
            else:
                self.coll_count = 0

        # If currently in AVD mode, drive backwards at -speed_min, steering to maintain target_heading, for 1.6 s:
        if self.coll_avd == "AVD":
            if time.time() - self.coll_avd_ts <= 3.86:
                heading_error = (self.target_heading - yaw_current_deg)
                steer_nominal = 0.022 * heading_error
                steer_limited = self.apply_turn_limiter(steer_nominal, math.radians(self.target_heading), yaw_current_rad) \
                                if self.turn_limiter_enable else steer_nominal
                speed_cmd = -0.42
                try:

                    self.drive_base.set_target_speed(speed_cmd)
                    self.drive_base.set_steering(steer_nominal)
                except Exception:
                    pass
                self.get_logger().info(f"AVD active: driving backwards at {speed_cmd:.2f}, steer limited {steer_limited:.3f}, steer raw {steer_nominal:.3f}")
                return
            else:
                # Exit AVD mode
                self.coll_avd = "N"
                self.coll_count = 0
                self.get_logger().info("Collision avoidance: exiting AVD mode")

        # Secondary CAU: dynamic x_max2 from 0.18 at speed_min to 0.36 at 0.6
        vmin = 0.49
        vmax = 0.6
        ps = self.prev_speed
        if ps <= vmin:
            x_max2 = 0.18
        elif ps >= vmax:
            x_max2 = 0.36
        else:
            x_max2 = 0.18 + (ps - vmin) / (vmax - vmin) * (0.36 - 0.18)
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
            if self.mode == "OBS":
                self.coll_avd = "CAU"
        else:
            if self.coll_avd == "CAU":
                self.coll_avd = "N"

        # If TURN mode, skip heavy detection. _turn_control handles turning
        if self.obs_status == "TURN" or self.obs_status == "PARK_TURN": # or self.obs_status == "UNPARK_1" or self.obs_status == "UNPARK_1":
            return

        # --- ALIGN mode handling ---
        if self.obs_status == "ALIGN":
            self.align_scans.append((ranges, angs))
            if len(self.align_scans) >= self.align_max:
                pts_all = []
                for rg, ag in self.align_scans:
                    rel = ((ag + math.pi) % (2*math.pi)) - math.pi
                    mask = np.abs(rel) <= math.radians(self.align_sector_half_deg)
                    idxs = np.where(mask)[0]
                    if idxs.size:
                        valid = [i for i in idxs if math.isfinite(rg[i]) and rg[i] > 2.35]
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
                            if abs(drift) <= 8.0:
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
                self.prev_obs_d_state = None
                self.section_count += 1
                if (self.park_active):
                    self.obs_status = "PARKING"
                    if self.direction == "CLOCK":
                        self.target_heading -= 90
                    else:
                        self.target_heading += 90
                    self.get_logger().info(f"ALIGN → PARKING; section_count={self.section_count}")
                else:
                    self.get_logger().info(f"ALIGN → SEARCH; section_count={self.section_count}")
            

        detected = []
        selected = None

        # Detection & selection only if mode == 'OBS' and in SEARCH or DRIVE, and frame available
        # --- Sync camera frame ---
        target_cam_ts = lidar_ts - (self.sync_delay_ms / 1000.0)
        #frame_ts, frame, diff_cam = self._get_best_frame(target_cam_ts)

        if self.mode == 'OBS' and (self.obs_status in ("SEARCH","DRIVE","PASSED","ALIGN")):
            # SEARCH logic: count scans until enough
            if self.obs_status == "SEARCH" or self.obs_status == "ALIGN":
                self.search_count += 1
            
            # Process LaserScan clusters
            clusters = []
            cur = []
            bad = 0
            mask = self.cached_sector_mask
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

            # Pre-calc measure vector in environment coords
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
                    #self.get_logger().debug(
                    #    f"Skipped cluster outside corridor: raw⊥={raw_perp:.2f}m, new⊥={new_perp:.2f}m "
                    #    f"not in [{self.perp_min_dist:.2f},{self.perp_max_dist:.2f}]"
                    #)
                    continue
                
                

                # Width check
                width_m = abs(angs_c[-1] - angs_c[0]) * r_avg
                if not (self.min_diameter <= width_m <= self.max_diameter):
                    self.get_logger().info(
                        f"Dropped inside corridor for width {width_m:.3f}m (cluster at mid {math.degrees(mid_rad)%360:.1f}°)"
                    )
                    continue

                #print((179+(math.degrees(mid_rad)%360)))
                #print(r_avg)
                
                # Make a reference snapshot
                ref_wall_s = time.time()             # float seconds
                ref_mono_ns = time.monotonic_ns()    # int nanoseconds

                # Compute an offset between the two clocks
                offset_ns = ref_mono_ns - int(ref_wall_s * 1_000_000_000)

                off_ts = (lidar_ts * 1_000_000_000) + offset_ns                

                bfre = time.monotonic_ns()

                label, used_ts, age_ms, h, s, v = self.mgr.analyze_patch_at_time(off_ts+(30*1_000_000), (178.2+(math.degrees(mid_rad)%360)), 2.22/r_avg, preview=True)

                
                are = time.monotonic_ns()
                
                """
                print("frame diff: " + str((used_ts - off_ts)/1_000_000))
                print("look diff: " + str((are - bfre)/1_000_000))
                print(used_ts)s
                print(off_ts)
                print(lidar_ts*1000)
                """

                color = label
                """

                # Sample HSV for color check
                (h,s,v),(x_px,y_px) = self._sample_hsv_range(frame, angs_c[0], angs_c[-1])
                hsv_info = f"H:{h:.0f} S:{s:.0f} V:{v:.0f}"
                if v < self.brightness_thresh:
                    #self.get_logger().info(
                    #    f"Dropped inside corridor for brightness V={v:.1f} (cluster at mid {math.degrees(mid_rad)%360:.1f}°)"
                    #)
                    if self.calibration_mode and display_frame is not None:
                        cv2.putText(display_frame, hsv_info, (x_px+8,y_px+22),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (128,128,128), 2)
                        txt2 = f"new⊥:{new_perp:.2f} raw⊥:{raw_perp:.2f}"
                        cv2.putText(display_frame, txt2, (x_px+8, y_px+40),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (128,128,128), 2)
                    continue"""

                self.get_logger().info(
                        f" inside corridor thresholds H={h:.1f},S={s:.1f},V={v:.1f} "
                    )

                # saturation thresholds
                m_g = (55-110)/0.9; b_g = 110 - m_g*0.1
                m_r = (55-120)/0.9; b_r = 110 - m_r*0.1
                s_g = m_g * r_avg + b_g
                s_r = m_r * r_avg + b_r
                #color = None
                if color == "OTHER":
                    self.get_logger().info(
                        f"Dropped inside corridor for color thresholds H={h:.1f},S={s:.1f},V={v:.1f} "
                        f"(s_g={s_g:.1f}, s_r={s_r:.1f})"
                    )
                    continue
                elif color == "GREEN":
                    color = "GREEN"
                elif color == "T":
                    self.get_logger().info(
                        f"Dropped inside corridor for color thresholds H={h:.1f},S={s:.1f},V={v:.1f} "
                        f"(s_g={s_g:.1f}, s_r={s_r:.1f})"
                    )
                    """
                    if self.calibration_mode and display_frame is not None:
                        cv2.putText(display_frame, hsv_info, (x_px+8,y_px+22),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (128,128,128), 2)
                        txt2 = f"new⊥:{new_perp:.2f} raw⊥:{raw_perp:.2f}"
                        cv2.putText(display_frame, txt2, (x_px+8, y_px+40),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (128,128,128), 2)"""
                    continue

                print(label)

                # Passed detection inside corridor
                detected.append({
                    'mid_rad': mid_rad,
                    'middle_angle_deg': math.degrees(mid_rad) % 360,
                    'distance': r_avg,
                    'color': color,
                    #'x_px': x_px, 'y_px': y_px,
                    'hsv': (h,s,v),
                    'raw_perp': raw_perp,
                    'new_perp': new_perp,
                    'parallel': parallel,
                })

            # SEARCH → DRIVE after enough scans
            if self.obs_status == "SEARCH":
                self.get_logger().debug("SEARCH count: %d" % self.search_count)
                if self.search_count >= self.search_required:
                    self.obs_status = "DRIVE"
                    self.get_logger().info("obs_status set to DRIVE after SEARCH scans")
                    self.search_count = 0

            # Selection among detected obstacles
            if detected:
                if len(detected) == 1 and detected[0]['parallel'] > -0.04:
                    selected = detected[0]
                else:
                    best_val = None; best_obs = None
                    for obs in detected:
                        obs_global_rad = math.radians(yaw) + obs['mid_rad']
                        Ox_u = math.cos(obs_global_rad)
                        Oy_u = math.sin(obs_global_rad)
                        cross = Lx*Oy_u - Ly*Ox_u
                        val = cross if self.direction == 'CLOCK' else -cross
                        val = obs['parallel']
                        if (best_val is None or val < best_val) and val > -0.04:
                            best_val = val; best_obs = obs
                    selected = best_obs

                print("history: " + str(self.state_history))
                
                if (selected is None):                 
                    state_cand = "no_detect"
                    if self.obs_status in ("SEARCH","DRIVE","PASSED","ALIGN"):
                        self.state_history.append(state_cand)

                elif not (selected is None):
                    # Determine candidate obs_drive_state
                    color = selected['color']
                    np_ = selected['new_perp']
                    state_cand = f"{color}_{'CLOSE' if np_ < 0.50 else 'FAR'}"
                    if self.obs_status in ("ALIGN","SEARCH","DRIVE","PASSED"):
                        self.state_history.append(state_cand)
                        cnt = Counter(self.state_history)
                        if self.obs_drive_state is None:
                            if len(self.state_history) >= 5:
                                for st, count in cnt.items():
                                    if st is not None and count >= 3 and (st != "no_detect"):
                                        if self.obs_drive_state != st:
                                            prev = self.obs_drive_state
                                            self.obs_drive_state = st
                                            if self.prev_obs_d_state is None:
                                                self.prev_obs_d_state = st
                                            self.get_logger().info(f"obs_drive_state changed from {prev} to {st} by consensus")
                                            if self.obs_status == "PASSED":
                                                self.obs_status = "DRIVE"
                                        break
                        else:
                            if len(self.state_history) >= 7:
                                for st, count in cnt.items():
                                    if st is not None and count >= 5  and (st != "no_detect"):
                                        if self.obs_drive_state != st:
                                            prev = self.obs_drive_state
                                            self.obs_drive_state = st
                                            self.prev_obs_d_state = st
                                            self.get_logger().info(f"obs_drive_state changed from {prev} to {st} by consensus")
                                            if self.obs_status == "PASSED":
                                                self.obs_status = "DRIVE"                                        
                                        break

                    # Print selected
                    self.get_logger().info(
                        f"Selected @ {selected['middle_angle_deg']:.1f}°, r={selected['distance']:.2f}m, "
                        f"raw⊥={selected['raw_perp']:.2f}m, new⊥={selected['new_perp']:.2f}m, "
                        f"par={selected['parallel']:.2f}m, color {selected['color']}, hsv={selected['hsv']}"
                    )

                    # Determine obs_status PASSED if parallel distance < pass_thresh
                    pass_thresh = 0.02
                    if self.obs_drive_state is not None:
                        if self.section_count % 4 == 0:
                            if (self.direction == "CLOCK" and self.obs_drive_state == "GREEN_CLOSE"):
                                pass_thresh = -0.13
                            elif (self.direction == "COUNTER" and self.obs_drive_state == "RED_CLOSE"):
                                pass_thresh = -0.06
                    if self.obs_status not in ("PASSED","TURN") and not self.park_active:
                        if selected['parallel'] < pass_thresh:
                            if self.passed_confirm:
                                self.obs_status = "PASSED"
                                for _ in range(12):
                                    self.state_history.append("no_detect")
                                self.prev_obs_d_state = self.obs_drive_state
                                self.obs_drive_state = None
                                self.coll_avd = "N"
                                self.get_logger().info(f"obs_status set to PASSED (parallel {selected['parallel']:.2f} < {pass_thresh})")
                            else:
                                self.passed_confirm = True
                        else:
                            self.passed_confirm = False

                    #self.get_logger().info(f"Current selected color: {selected['color']}")
            else:
                if self.mode == 'OBS':
                    self.get_logger().info("No detected obstacle in corridor")

            """
            # Save debug frame if calibration_mode
            if self.calibration_mode and display_frame is not None:
                os.makedirs(self.calib_folder, exist_ok=True)
                fname = os.path.join(self.calib_folder,
                                     f"frame_{self._calib_count % self.calib_max_frames:02d}.png")
                cv2.imwrite(fname, display_frame)
                self._calib_count += 1"""

        # --- Control logic blocks ---
        speed = 0.0
        steer = 0.0
        m = self.obs_status
        if self.save_status != "" and self.obs_status == "PASSED":
            self.obs_status = "DRIVE"

        if (self.obs_status == "SEARCH"):
            self.drive_base.set_front_led(1)

        if (self.obs_status == "ALIGN"):
            self.drive_base.set_front_led(1)
            return

        #print("dist: " + str(self.drive_base.get_distance()) + " diff: " + str((self.drive_base.get_distance() - self.free_r_ac_d)))
        # Mode transitions using yaw_deg when in FREE
        if self.mode == "FREE":
            try:
                cont_yaw = self.drive_base.get_continuous_yaw()
            except Exception:
                cont_yaw = 0.0
            if self.direction == "CLOCK" and (cont_yaw < -1070):
                self.donetimes += 1
                if self.donetimes >= 14:
                    if (self.free_r_ac_d == 0):
                        self.free_r_ac_d = self.drive_base.get_distance()

                    if ((self.drive_base.get_distance() - self.free_r_ac_d) > self.free_r_target_d):
                        m = "DONE"
            elif self.direction == "COUNTER" and (cont_yaw > 1065):
                self.donetimes += 1
                if self.donetimes >= 14:
                    if (self.free_r_ac_d == 0):
                        self.free_r_ac_d = self.drive_base.get_distance()

                    if ((self.drive_base.get_distance() - self.free_r_ac_d) > self.free_r_target_d):
                        m = "DONE"

        if self.obs_status == "START":
            # Handled earlier; but if reached here, ensure speed=0
            speed = 0.0; steer = 0.0

        elif self.obs_status == "WAIT":
            # Handled earlier; but ensure speed=0
            speed = 0.0; steer = 0.0

        elif self.mode == "FREE" and self.direction == "CLOCK":
            threshold = 0.67; corner = 0.0

            lefts  = [sample(math.radians(a)) for a in range(28,60)]
            fronts = [sample(math.radians(a)) for a in range(4,6)]
            backs  = [sample(math.radians(a)) for a in range(33,62)]

            fin_l = [d for d in lefts if d]
            fin_f = [d for d in fronts if d]
            fin_b = [d for d in backs if d]

            d_l = sum(fin_l)/len(fin_l) if fin_l else float('inf')
            d_f = sum(fin_f)/len(fin_f) if fin_f else float('inf')

            #self.get_logger().info(f"seeing yaw_synced_deg: {yaw_synced_deg}")
            if d_f < threshold: corner = threshold - d_f
            error = -0.5*(1.3*corner + 1*(0.54 - d_l))

            ang = self.kp_lateral * error
            cap = 0.6 if d_f >= 1.05 else min(0.1 + 0.3*d_f, 0.6)
            ang = min(ang, cap) if ang >= 0 else ang
            ang -= corner * 1.77

            if d_f <= 0.32: ang -= 0.12

            factor = max(0.72, min(1.0, 0.13 / abs(0.9 * error))) if error != 0 else 1.0

            speed = self.v_max_free_r * factor
            steer = -ang

        elif self.mode == "FREE" and self.direction == "COUNTER":
            threshold = 0.65; corner = 0.0

            rights  = [sample(math.radians(a)) for a in range(300,332)]
            fronts = [sample(math.radians(a)) for a in range(354,356)]
            backs  = [sample(math.radians(a)) for a in range(33,62)]

            fin_r = [d for d in rights if d]
            fin_f = [d for d in fronts if d]
            fin_b = [d for d in backs if d]

            d_r = sum(fin_r)/len(fin_r) if fin_r else float('inf')
            d_f = sum(fin_f)/len(fin_f) if fin_f else float('inf')

            #self.get_logger().info(f"seeing yaw_synced_deg: {yaw_synced_deg}")
            if d_f < threshold: corner = threshold - d_f
            error = -0.5*(1.3*corner + 1*(0.50 - d_r))

            ang = self.kp_lateral * error
            cap = 0.6 if d_f >= 1.05 else min(0.1 + 0.3*d_f, 0.6)
            ang = min(ang, cap) if ang >= 0 else ang
            ang -= corner * 1.77

            if d_f <= 0.32: ang -= 0.12

            factor = max(0.72, min(1.0, 0.13 / abs(0.9 * error))) if error != 0 else 1.0

            speed = self.v_max_free_r * factor
            steer = ang

            """            
            threshold = 0.94; corner = 0.0
            rights  = [sample(math.radians(a)) for a in range(300,322)]
            fronts = [sample(math.radians(a)) for a in range(354,356)]
            backs  = [sample(math.radians(a)) for a in range(33,62)]
            fin_r = [d for d in rights if d]
            fin_f = [d for d in fronts if d]
            fin_b = [d for d in backs if d]
            d_r = sum(fin_r)/len(fin_r) if fin_r else float('inf')
            d_f = sum(fin_f)/len(fin_f) if fin_f else float('inf')
            #self.get_logger().info(f"seeing yaw_synced_deg: {d_r}" + " df " + str(d_f))
            target = 0.48
            if d_f < threshold: 
                corner = threshold - d_f
                target = 0.32
            error = -0.8*(1.1*corner + 2*(target - d_r))
            ang = self.kp_lateral * error
            cap = 0.6 if d_f >= 1.05 else min(0.1 + 0.3*d_f, 1.0)
            ang = min(ang, cap) if ang >= 0 else ang
            ang -= corner * 1.77
            if d_f <= 0.32: ang -= 0.12
            factor = max(0.72, min(1.0, 0.13 / abs(0.9 * error))) if error != 0 else 1.0
            speed = self.v_max_free_r * factor
            steer = ang
            """

        elif self.mode == "OBS" and self.obs_status == "UNPARK_1":
            if self.direction == "CLOCK":
                start_f = 25 - yaw_synced_deg + self.target_heading
                end_f   = 28 - yaw_synced_deg + self.target_heading
                d_front = sample_range(start_f, end_f)
                steer = 0.82
            else:
                start_f = 328 - yaw_synced_deg + self.target_heading
                end_f   = 333 - yaw_synced_deg + self.target_heading
                d_front = sample_range(start_f, end_f)
                steer = -0.82     
            
            print("Parking 1")

            if d_front > 0.23:
                speed = 0
                steer = 0
                self.parkstop_times = 0
                self.parkstart_times = 0
                self.obs_status = "SEARCH"
            else:
                speed = 0.22 

            """
            if self.parkstart_times == 0:
                speed = 0.22        

            if self.parkstart_times < 5:
                self.parkstart_times += 1
            else:
                speed = 0.22   

                if d_front > 0.126:
                    speed = 0.36

                    if self.parkstop_times < 6:
                        self.parkstop_times += 1
                    else:
                        self.parkstop_times = 0
                        self.parkstart_times = 0
                        self.obs_status = "UNPARK_2"
            """

        
        elif self.mode == "OBS" and self.obs_status == "UNPARK_2":
            if self.direction == "CLOCK":
                start_f = 27 - yaw_synced_deg + self.target_heading
                end_f   = 32 - yaw_synced_deg + self.target_heading
                d_front = sample_range(start_f, end_f)
                steer = 0
            else:
                start_f = 333 - yaw_synced_deg + self.target_heading
                end_f   = 338 - yaw_synced_deg + self.target_heading
                d_front = sample_range(start_f, end_f)
                steer = 0            

            #print(d_front)

            if self.parkstart_times < 5:
                speed = 0.11
                self.parkstart_times += 1
            else:
                speed = 0.47

                if self.direction == "CLOCK":
                    target = 0.131
                else:
                    target = 0.133

                if d_front < target:
                    speed = -0.3 
                    self.parkstop_times = 0
                    self.parkstart_times = 0
                    self.obs_status = "UNPARK_1"
                elif d_front > 0.27:
                        self.parkstop_times = 0
                        self.parkstart_times = 0
                        self.obs_status = "SEARCH"


        elif self.mode == "OBS" and self.obs_status == "DRIVE":
            self.max_left = math.radians(90)
            self.max_right = math.radians(90)
            turn_limiter_enable = True

            # compute lateral & steering
            d_mode = self.obs_drive_state or ""
            d_dir = self.direction
            
            if self.save_status != "":
                print("mode to: " + str(self.save_status))
                d_mode = self.save_status

            # New target_distance logic:
            if (d_dir == "CLOCK" and d_mode == "RED_FAR") or (d_dir == "COUNTER" and d_mode == "GREEN_FAR"):
                target_distance = 0.741
            elif (d_dir == "CLOCK" and d_mode == "RED_CLOSE") or (d_dir == "COUNTER" and d_mode == "GREEN_CLOSE"):
                target_distance = 0.62
            elif (d_dir == "CLOCK" and d_mode == "GREEN_FAR") or (d_dir == "COUNTER" and d_mode == "RED_FAR"):
                target_distance = 0.38
            elif (d_dir == "CLOCK" and d_mode == "GREEN_CLOSE") or (d_dir == "COUNTER" and d_mode == "RED_CLOSE"):
                if self.section_count % 4 == 0:
                    target_distance = 0.38
                else:
                    target_distance = 0.238
            else:
                target_distance = 0.5

            #if self.park_active and self.direction == "COUNTER":
            #    target_distance = 0.39

            # rotated sample_range for side:
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

            if target_distance > 0.48:
                if self.direction == "COUNTER":
                    start_deg = 72 - yaw_synced_deg + self.target_heading
                    end_deg   = 75 - yaw_synced_deg + self.target_heading
                    d_d1 = sample_range(start_deg, end_deg)
                    start_deg = 98 - yaw_synced_deg + self.target_heading
                    end_deg   = 101 - yaw_synced_deg + self.target_heading
                    d_d2 = sample_range(start_deg, end_deg)
                else:
                    start_deg = 259 - yaw_synced_deg + self.target_heading
                    end_deg   = 262 - yaw_synced_deg + self.target_heading
                    d_d1 = sample_range(start_deg, end_deg)
                    start_deg = 280 - yaw_synced_deg + self.target_heading
                    end_deg   = 283 - yaw_synced_deg + self.target_heading
                    d_d2 = sample_range(start_deg, end_deg)

                leer = 0

                if target_distance > 0.67:
                    limit = 0.42
                else:    
                    limit = 0.55


                if (d_d1 < limit):
                    leer = 0.94-d_d1
                    if self.direction == "CLOCK":
                        lateral_error = (target_distance - leer)
                    else:     
                        lateral_error = -(target_distance - leer)
                elif (d_d2 < limit):
                    leer = 0.94-d_d2
                    if self.direction == "CLOCK":
                        lateral_error = (target_distance - leer)
                    else:     
                        lateral_error = -(target_distance - leer)

                #print("dl " + str(d_l) + " dd1 " + str(d_d1) + " dd2 " + str(d_d2) + " td " + str(leer))


            self.get_logger().debug("dl_: %.2f" % d_l)
            if self.last_d is None:
                self.last_d = lateral_error
            else:
                if abs(lateral_error - self.last_d) > 0.09:
                    lateral_error = self.last_d + (lateral_error - self.last_d)/2
                self.last_d = lateral_error

            heading_error = -(self.target_heading - yaw_current_deg)


            if self.park_active and abs(lateral_error) > 0.05 and self.direction == "COUNTER":
                steer_nominal = (1.5 * lateral_error)
            elif self.park_active:
                steer_nominal = 0.022 * heading_error + 1.3 * lateral_error
            elif abs(lateral_error) > 0.12:
                steer_nominal = (1.5 * lateral_error)
            elif abs(lateral_error) > 0.06:
                steer_nominal = (0.005 * heading_error + 1.3 * lateral_error)
            else:
                if ((d_dir == "CLOCK" and d_mode == "GREEN_CLOSE") or (d_dir == "COUNTER" and d_mode == "RED_CLOSE")) and self.section_count % 4 == 0:
                    steer_nominal = 0.022 * heading_error + 1.1 * lateral_error
                else:
                    steer_nominal = 0.022 * heading_error + 1.1 * lateral_error
            abs_st = abs(steer_nominal)
            print("abs steer: " + str(abs_st))
            if abs_st >= 0.6:
                speed_steer = 0.40
            if abs(lateral_error) > 0.18:
                speed_steer = 0.46
            elif abs(heading_error) > 8:
                speed_steer = 0.36
            elif abs_st <= 0.12:
                speed_steer = self.speed_max
            else:
                frac = (abs_st - 0.2) / (0.6 - 0.1)
                speed_steer = self.speed_min + (0.51 - self.speed_min) * frac
                speed_steer = 0.52

            # front distance dependent speed capping, similar to PASSED
            start_f = 357 - yaw_synced_deg + self.target_heading
            end_f   = 360 - yaw_synced_deg + self.target_heading
            d_front = sample_range(start_f, end_f)
            if d_front >= 1.2:
                speed_front = self.speed_max
            elif d_front <= 0.3:
                speed_front = self.speed_min
            else:
                frac2 = (d_front) / (1.1 - 0.0)
                speed_front = self.speed_min + (0.62 - self.speed_min) * frac2
            speed = min(speed_steer, speed_front)
            steer = steer_nominal

            

            

            # front obstacle brake-to-PASSED
            if d_front < 0.94 and abs(steer_nominal) < 0.22 and selected is None:
                #self.drive_base.brake(1)
                speed = 0.1
                if self.stop_times < 4:
                    self.stop_times += 1
                else:
                    self.stop_times = 0
                    self.obs_status = "PASSED"
            else:
                self.stop_times = 0

            """
            # dynamic capping for GREEN_CLOSE/RED_CLOSE in certain sections
            if (d_dir == "CLOCK" and d_mode == "GREEN_CLOSE") or (d_dir == "COUNTER" and d_mode == "RED_CLOSE"):
                if self.section_count % 4 == 0 and selected is not None:
                    if abs(selected['parallel']) < 0.9:
                        speed_cap = 0.5
                    elif abs(selected['parallel']) > 1.1:
                        speed_cap = 0.56
                    else:
                        frac_sp = (abs(selected['parallel']) - 0.35) / (0.7 - 0.35)
                        speed_cap = 0.52 + (0.57 - 0.48) * frac_sp
                    speed = min(speed, speed_cap)
            """

            # front distance speed limiter, parking mode
            if self.park_active:
                self.obs_status = "DRIVE"
                if self.direction == "COUNTER":
                    start_deg = 280 - yaw_synced_deg + self.target_heading
                    end_deg   = 283 - yaw_synced_deg + self.target_heading
                    d_d2 = sample_range(start_deg, end_deg)
                    start_f = 355 - yaw_synced_deg + self.target_heading
                    end_f   = 360 - yaw_synced_deg + self.target_heading
                    d_front = sample_range(start_f, end_f)
                    if d_front <= 0.960 and d_front >= 0.885: #and d_d2 < 0.58:
                        if self.stops_times < 3:
                            self.stops_times += 1
                            self.drive_base.brake(1)
                            speed_front = 0
                            print("holding parksss")
                        else:
                            self.stops_times = 0
                            self.obs_status = "PARK_TURN"
                            print("holding park")
                            speed_front = 0
                    elif d_front <= 1.9: #and d_d2 < 0.58:
                        speed_front = 0.101
                    else:
                        frac2 = (d_front - 0.1) / (1.7 - 0.0)
                        speed_front = 0.22
                else:
                    start_f = 0 - yaw_synced_deg + self.target_heading
                    end_f   = 5 - yaw_synced_deg + self.target_heading
                    d_front = sample_range(start_f, end_f)
                    if d_front <= 1.588 and d_front >= 1.125:
                        print("close")
                        if self.stops_times < 3:
                            self.stops_times += 1
                            self.drive_base.brake(1)
                            speed_front = 0
                            print("holding parksss")
                        else:
                            self.stops_times = 0
                            self.obs_status = "PARK_TURN"
                            print("holding park")
                            speed_front = 0
                    elif d_front <= 2.1:
                        speed_front = 0.101
                    else:
                        frac2 = (d_front - 0.1) / (1.7 - 0.0)
                        speed_front = self.speed_min + (self.speed_max - self.speed_min) * frac2
                speed = min(speed_steer, speed_front)

            # Apply CAU speed cap if in CAU mode
            if self.coll_avd == "CAU":
                speed = min(speed, self.speed_min + 0.04)

            if self.section_count == 12:
                speed = min(speed, 0.3)

            if turn_limiter_enable:
                steer = self.apply_turn_limiter(steer, math.radians(self.target_heading), yaw_current_rad)
                steer = 0.78*steer
             
            steer = max(-0.8, (min(0.8,steer)))
            
            print("drive speed set: " + str(speed))

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
                steer_nominal = 1.4 * lateral_error
            else:
                steer_nominal = 0.019 * heading_error  + 0.6 * lateral_error
            abs_st = abs(steer_nominal)
            if abs_st <= 0.1:
                speed_steer = self.speed_max
            elif abs_st >= 0.5:
                speed_steer = self.speed_min
            else:
                frac = (abs_st - 0.1) / (0.5 - 0.1)
                speed_steer = self.speed_max + (self.speed_min - self.speed_max) * frac

            # front distance speed limiter
            start_f = 355 - yaw_synced_deg + self.target_heading
            end_f   = 360 - yaw_synced_deg + self.target_heading
            d_front = sample_range(start_f, end_f)
            if d_front >= 0.8:
                speed_front = 0.55
            elif d_front <= 0.45:
                speed_front = self.speed_min
            else:
                frac2 = (d_front - 0.05) / (0.8 - 0.0)
                speed_front = self.speed_min + (self.speed_max - self.speed_min) * frac2
            speed = min(speed_steer, speed_front)

            if (d_front> 1.6 and abs_st <= 0.1):
                speed = 0.55
            steer = steer_nominal

            if d_front < 0.33:
                self.drive_base.brake(1)
                if self.stop_times < 3:
                    self.stop_times += 1
                else:
                    self.stop_times = 0
                    self.obs_status = "TURN"
                    self.coll_avd = "N"
            else:
                self.stop_times = 0

            self.max_left = math.radians(55)
            self.max_right = math.radians(55)
            turn_limiter_enable = True
            if turn_limiter_enable:
                steer = self.apply_turn_limiter(steer, math.radians(self.target_heading), yaw_current_rad)
                steer = 0.78*steer

        elif self.mode == "OBS" and self.obs_status == "PARKING":
            self.speed_max = 0.62

            target_distance = 0.73

            start_deg = 174 - yaw_synced_deg + self.target_heading
            end_deg   = 186 - yaw_synced_deg + self.target_heading
            d_back = sample_range(start_deg, end_deg)
            

            start_deg = 85 - yaw_synced_deg + self.target_heading
            end_deg   = 95 - yaw_synced_deg + self.target_heading
            d_left = sample_range(start_deg, end_deg)

            start_deg = 265 - yaw_synced_deg + self.target_heading
            end_deg   = 275 - yaw_synced_deg + self.target_heading
            d_right = sample_range(start_deg, end_deg)

            # rotated sample_range for side:
            if self.direction == "COUNTER":
                d_l = d_left
                lateral_error = target_distance - d_l
            else:
                d_l = d_right
                lateral_error = -(target_distance - d_l)

            #print(d_back)
            if target_distance > 0.48:
                if self.direction == "CLOCK":
                    start_deg = 79 - yaw_synced_deg + self.target_heading
                    end_deg   = 82 - yaw_synced_deg + self.target_heading
                    d_d1 = sample_range(start_deg, end_deg)
                    start_deg = 98 - yaw_synced_deg + self.target_heading
                    end_deg   = 101 - yaw_synced_deg + self.target_heading
                    d_d2 = sample_range(start_deg, end_deg)
                else:
                    start_deg = 259 - yaw_synced_deg + self.target_heading
                    end_deg   = 265 - yaw_synced_deg + self.target_heading
                    d_d1 = sample_range(start_deg, end_deg)
                    start_deg = 278 - yaw_synced_deg + self.target_heading
                    end_deg   = 281 - yaw_synced_deg + self.target_heading
                    d_d2 = sample_range(start_deg, end_deg)

                leer = 0


                d_avg = (d_d1+d_d2)/2

                if True:

                    if (d_d1 < 0.42):
                        leer = 0.92-d_d1
                        if self.direction == "CLOCK":
                            lateral_error = -(target_distance - leer)
                        else:     
                            lateral_error = (target_distance - leer)
                    elif (d_d2 < 0.42):
                        leer = 0.92-d_d2
                        if self.direction == "CLOCK":
                            lateral_error = -(target_distance - leer)
                        else:     
                            lateral_error = (target_distance - leer)

                #print("dl " + str(d_l) + " dd1 " + str(d_d1) + " dd2 " + str(d_d2) + " td " + str(leer))


            self.get_logger().debug("dl_: %.2f" % d_l)
            if self.last_d is None:
                self.last_d = lateral_error
            else:
                if abs(lateral_error - self.last_d) > 0.09:
                    lateral_error = self.last_d + (lateral_error - self.last_d)/2
                self.last_d = lateral_error

            heading_error = -(self.target_heading - yaw_current_deg)

            

            if abs(lateral_error) > 0.07:
                steer_nominal = 5.8 * lateral_error
            else:
                steer_nominal = 0.043 * heading_error + 2.4 * lateral_error
            
            abs_st = abs(steer_nominal)
            if abs_st <= 0.1:
                speed_steer = self.speed_max
            elif abs_st >= 0.5:
                speed_steer = self.speed_min
            else:
                frac = (abs_st - 0.1) / (0.5 - 0.1)
                speed_steer = self.speed_max + (self.speed_min - self.speed_max) * frac

            # front distance speed limiter
            if self.direction == "CLOCK":
                start_f = 0 - yaw_synced_deg + self.target_heading
                end_f   = 3 - yaw_synced_deg + self.target_heading
            else:
                start_f = 357 - yaw_synced_deg + self.target_heading
                end_f   = 360 - yaw_synced_deg + self.target_heading                
            d_front = sample_range(start_f, end_f)
            if d_front >= 1.9:
                speed_front = self.speed_max
            elif d_front <= 1.6:
                speed_front = self.speed_min
            else:
                frac2 = (d_front - 1.5) / (0.6- 0.0)
                speed_front = self.speed_min + (self.speed_max - self.speed_min) * frac2
            speed = min(speed_steer, speed_front)
            steer = steer_nominal


            self.max_left = math.radians(58)
            self.max_right = math.radians(58)
            turn_limiter_enable = True
            if turn_limiter_enable:
                steer = self.apply_turn_limiter(steer, math.radians(self.target_heading), yaw_current_rad)

            
            if ((d_front < 1.4 and self.direction == "CLOCK") or (d_front < 1.556 and self.direction == "COUNTER")) and abs(lateral_error) < 0.07:
                speed = -0.5
                if self.stop_times < 3:
                    self.stop_times += 1
                else:
                    self.stop_times = 0
                    self.obs_status = "PARK_TURN"
            else:
                self.stop_times = 0


        elif self.mode == "OBS" and self.obs_status == "PARKING_2":
            th = self.target_heading
            if self.direction == "CLOCK":
                th +=2
            else:
                th +=1

            heading_error = -(th - yaw_current_deg)
            steer_nominal = 0.018 * heading_error
            speed = 0.25

            if self.direction == "CLOCK":
                start_f = 0 - yaw_synced_deg + self.target_heading
                end_f   = 3 - yaw_synced_deg + self.target_heading
            else:
                start_f = 175 - yaw_synced_deg + self.target_heading
                end_f   = 185 - yaw_synced_deg + self.target_heading                
            d_front = sample_range(start_f, end_f)

            print("front: " + str(d_front))

            if d_front >= 0.55:
                speed_front = speed = 0.24
            elif d_front <= 0.362:
                if self.stops_times < 3:
                    self.stops_times += 1
                    self.drive_base.brake(1)
                    speed_front = 0
                else:
                    self.stops_times = 0
                    self.obs_status = "PARK_TURN"
                    self.park_active = False
                    speed_front = 0
                print("parked")
            elif d_front <= 0.5:
                speed_front = 0.09
            else:
                frac2 = (d_front - 0.3) / (0.5- 0.0)
                speed_front = 0.1 + (0.1) * frac2
                #speed_front = speed_front * (-1)
            speed = min(speed_front, 0.5)
            steer = steer_nominal

            if self.direction == "COUNTER":
                speed = -speed-0.02
                steer = -steer

            print("Park2")
        else:
            speed = 0.0; steer = 0.0


        if m == "DONE":
            print("Full run time: " + str((time.time()-self.starttime_ts)))
            if self.parkstop_times <= 6:
                self.drive_base.brake(0.5)
                speed = 0.0; steer = 0.0
                self.parkstop_times += 1
            else:
                speed = 0.0; steer = 0.0

        print("self stat " + str(self.save_status))

        # Section 12 stop behavior (preserved from before)
        if self.mode == "OBS" and self.section_count == 12 and (not self.park_active):

            start_f = 90 - yaw_synced_deg + self.target_heading
            end_f   = 103 - yaw_synced_deg + self.target_heading
            d_lb = sample_range(start_f, end_f)

            start_f = 257 - yaw_synced_deg + self.target_heading
            end_f   = 270 - yaw_synced_deg + self.target_heading
            d_rb = sample_range(start_f, end_f)

            start_f = 357 - yaw_synced_deg + self.target_heading
            end_f   = 360 - yaw_synced_deg + self.target_heading
            d_front = sample_range(start_f, end_f)

            start_f = 184 - yaw_synced_deg + self.target_heading
            end_f   = 188 - yaw_synced_deg + self.target_heading
            d_back = sample_range(start_f, end_f)

            if ((d_lb + d_rb) < 1.05) and self.save_status == "":
                self.save_status = self.prev_obs_d_state

            print("stop clamp: " + str((d_lb + d_rb)) + " front: " + str(d_front))

            if (0.5 < (d_lb + d_rb) < 1.05) and d_front < 1.9: 

                speed = 0.15
                print("stop clamp active")
                if self.lapstop_counter <= 5:
                    if d_front < 1.92:
                        speed = 0
                        self.drive_base.brake(1)
                    self.lapstop_counter += 1
                else:
                    print("STOPPING")
                    speed = 0
                    if self.stop_time is None:
                        self.stop_time = time.time()
                        self.drive_base.brake(1)
                    elif time.time() - self.stop_time >= 4.2:
                        self.park_active = True      
                    elif time.time() - self.stop_time <= 0.5:
                        self.drive_base.brake(1)

            else:
                self.lapstop_counter = 0          
            

        #speed = 0.0; steer = 0.0

        speedfactor = (1+ (0.0033*self.section_count))

        print("speeeeed " + str(speedfactor))

        speed = speed * speedfactor

        # Clamp and apply
        speed = max(min(speed, 1.0), -1.0)
        steer = max(min(steer, 1.0), -1.0)

        self.prev_speed = speed

        
        speed = 0
        steer = 0
        

        if (self.obs_status == "UNPARK_1"):
            steer = steer
        else:
            steer = 1* steer
        

        elapsed = time.time() - t0#lidar_ts
        self.get_logger().info(
            f"Mode: {self.mode}, obs_drive_state: {self.obs_drive_state}, obs_status: {self.obs_status}, "
            f"Coll_avd: {self.drive_base.get_continuous_yaw()}, Speed: {speed:.3f}, Steer: {steer:.3f}; "
            f"scan_cb elapsed: {elapsed:.3f}s"
        )
        try:
            print("setting " + str(speed))
            self.drive_base.set_target_speed(speed)
            self.drive_base.set_steering(steer)
        except Exception as e:
            self.get_logger().error(f"DriveBase command failed: {e}")

    # --------------- TURN control method ---------------
    def _turn_control(self):
        self.coll_avd = "N"
        if not rclpy.ok(): return

        if self.obs_status != "TURN" and self.obs_status != "PARK_TURN": return

        # Button check: if waiting context and button released, skip
        if self.wait_pressed_time is not None:
            try:
                btn_state = self.drive_base.get_switch_state()
            except Exception:
                btn_state = True
            if not btn_state:
                try:
                    #TODO: Why does set speed to 0 result in full send?
                    self.drive_base.set_target_speed(0.0)
                    self.drive_base.brake(1)
                    self.drive_base.set_steering(0.0)
                except:
                    pass
                self.get_logger().info("Button released after start: stopping and skipping turn processing")
                return

        try:
            yaw_current_deg = self.drive_base.get_continuous_yaw()
        except Exception:
            return
        dir_c = self.direction

        ###### Start unpark mode
        #print("s " + str(self.obs_status) + " l "  + str(dir_c))

        
        if self.obs_status == "UNPARK_1" or self.obs_status == "UNPARK_2":
            
            if dir_c == "CLOCK":
                if self.turn_starttimes == 0:
                    if self.obs_status == "UNPARK_1":
                        self.turn_park_target = self.turn_park_target - 1.6
                
                if self.obs_status == "UNPARK_1":
                    steer = -1.0
                    if self.turn_starttimes < 3:
                        speed = -0.2
                        self.turn_starttimes += 1   
                    else:
                        speed = -0.538                  
                else:
                    steer = 0.0
                    if self.turn_starttimes < 3:
                        speed = 0.2
                        self.turn_starttimes += 1   
                    else:
                        speed = 0.46                       
                err = yaw_current_deg - self.turn_park_target
            else:
                if self.turn_starttimes == 0:
                    if self.obs_status == "UNPARK_1":
                        self.turn_park_target = self.turn_park_target + 2
                
                if self.obs_status == "UNPARK_1":
                    steer = 1.0
                    if self.turn_starttimes < 3:
                        speed = -0.2
                        self.turn_starttimes += 1   
                    else:
                        speed = -0.538  
                else:    
                    steer = 0                                  
                err = yaw_current_deg - self.turn_park_target

            #print(self.turn_park_target)

            # Stop if within tolerance or overshot
            if (self.direction == "CLOCK"):
                if yaw_current_deg <= self.turn_park_target:
                    stop_turn = True
                else:
                    stop_turn = False
            else:
                if yaw_current_deg >= self.turn_park_target - self.turn_tolerance_deg:
                    stop_turn = True
                else:
                    stop_turn = False            

            if stop_turn:
                if self.stop_times < 3:
                    if self.obs_status == "UNPARK_1":
                        speed = 0.22
                        steer = 0.0
                    else:
                        speed = -0.33
                        steer = 0.0 
                    self.stop_times += 1
                else:
                    self.turn_starttimes  = 0
                    self.stop_times = 0
                    if self.obs_status == "UNPARK_1":
                        self.obs_status = "UNPARK_2"
                        self.get_logger().info(f"TURN complete: set obs_status=PARKING_2, theoretical target_heading={self.target_heading:.1f}°")
                    else:
                        self.obs_status = "UNPARK_1"
                        self.align_scans.clear()
                        self.get_logger().info(f"TURN complete: set obs_status=ALIGN, theoretical target_heading={self.target_heading:.1f}°")
            
            print(speed)

            speed = max(min(speed, 1.0), -1.0)
            steer = max(min(steer, 1.0), -1.0)
            try:
                print("settint" + str(speed))
                self.drive_base.set_target_speed(speed)
                self.drive_base.set_steering(steer)
            except Exception as e:
                self.get_logger().error(f"TURN control DriveBase failed: {e}")
            
            return

        ###### End unpark mode 



        
        angle_offset = 90
        if dir_c == "CLOCK":
            desired = self.target_heading - angle_offset
            if self.obs_status == "PARK_TURN":
                steer = -0.98
            else:
                steer = -1.0
            err = yaw_current_deg - desired
        else:
            if self.obs_status == "PARK_TURN":
                desired = self.target_heading + angle_offset
                steer = 0.98
            else:
                desired = self.target_heading + angle_offset
                steer = 1.0
            err = desired - yaw_current_deg

        # Speed control
        turn_min_speed = 0.16
        turn_max_speed = 0.44
        if self.obs_status == "PARK_TURN":
            turn_min_speed = 0.15
            turn_max_speed = 0.22        
        k_turn = 0.01
        spd = k_turn * abs(err)
        if spd < turn_min_speed:
            spd = turn_min_speed
        elif spd > turn_max_speed:
            spd = turn_max_speed
        if self.obs_status == "PARK_TURN" and self.direction == "CLOCK":
            speed = -spd
        else:
            speed = -spd  # always backwards

        if self.turn_starttimes < 6:
            speed = 0
            self.drive_base.brake(0.9)
            self.turn_starttimes += 1

        # Stop if within tolerance or overshot
        if self.direction == "CLOCK": #or (self.obs_status == "PARK_TURN" and self.direction == "COUNTER"):
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
            self.drive_base.brake(1)
            self.turn_starttimes  = 0
            if self.obs_status == "PARK_TURN":
                speed = 0
                steer = 0.0
            else:
                speed = 0
                steer = 0.0                
            
            if self.stop_times < 4:
                self.drive_base.brake(1)
                self.stop_times += 1
            else:
                self.stop_times = 0
                old_th = self.target_heading
                if dir_c == "CLOCK":
                    self.target_heading = old_th - angle_offset
                else:
                    
                    if self.obs_status == "PARK_TURN":
                        self.target_heading = old_th + angle_offset
                    else:
                        self.target_heading = old_th + angle_offset

                if self.obs_status == "PARK_TURN":
                    if self.park_active == True:
                        self.obs_status = "PARKING_2"
                        self.direction = "COUNTER"
                        #self.target_heading += 180
                    else:
                        self.obs_status = "DONE"
                    
                    self.get_logger().info(f"TURN complete: set obs_status=PARKING_2, theoretical target_heading={self.target_heading:.1f}°")
                else:
                    self.obs_status = "ALIGN"
                    self.align_scans.clear()
                    self.get_logger().info(f"TURN complete: set obs_status=ALIGN, theoretical target_heading={self.target_heading:.1f}°")

        #speedfactor = (1+ (0.0022*self.section_count))

        print("speeeeed " + str(speed))

        speed = speed #* speedfactor

        speed = max(min(speed, 1.0), -1.0)
        steer = max(min(steer, 1.0), -1.0)


        
        speed = 0
        steer = 0
        
        

        try:
            if(stop_turn):
                self.drive_base.brake(1)
            else:
                self.drive_base.set_target_speed(speed)
                self.drive_base.set_steering(0.6*steer)
        except Exception as e:
            self.get_logger().error(f"TURN control DriveBase failed: {e}")
        self.get_logger().debug(f"TURN loop: yaw={yaw_current_deg:.1f}°, desired={desired:.1f}°, err={err:.1f}°, speed={speed:.2f}, steer={steer:.2f}")

    def destroy_node(self):
        self._led_thread_stop = True
        time.sleep(0.1)
        try:
            self.drive_base.set_steering(0)
            time.sleep(0.2)
            self.drive_base.shutdown()
            self.get_logger().info("DriveBase cleaned up")
            #if self.cap and self.cap.isOpened():
            #    self.cap.release()
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")
        try:
            self.drive_base.set_rear_led(0)
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
