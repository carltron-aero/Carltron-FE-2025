#!/usr/bin/env python3
"""
DriveBase

This module controls a drive base on a Raspberry Pi 5 (Ubuntu 24.04)
using:
- Hardware PWM (via hw_pwm module) for steering servo + H-bridge drive + front LED
- Software PWM (gpiozero.PWMLED) for rear/red/green LEDs
- A wheel encoder (GPIO20) to measure speed and distance
- An IMU (BNO08X via I2C) to read yaw; provides wrapped and continuous yaw (+ history buffer)
- A mode-based speed controller (COAST / BRAKE_TO_STOP / SPEED) using a simple PI loop


Highlights:
- Speed controller uses encoder feedback and a PI controller.
- If the measured speed glitches high beyond a **known physical max** (1.6 m/s),
  it is clamped to **0** for that tick to prevent the controller from fighting a phantom reading.
- Going to target speed 0 ⇒ immediate COAST (no residual torque).
- Target speed sign flip (e.g., + to -) ⇒ full BRAKE until stopped, then ramp to new target.

Test section at the bottom:
- Demonstrates coast on zero target and brake-then-reverse behavior.
- Continuously prints yaw (wrapped + continuous), controller mode, speed, and target.
"""

import os
import time
import math
import threading
from collections import deque
import bisect

# --- try to import HW PWM module ---
# Prefer `hw_pwm.py`, but fall back to a local filename `hw-pwm.py` if needed.
try:
    import hw_pwm as hpwm  # preferred filename: hw_pwm.py
except ModuleNotFoundError:
    import importlib.util
    here = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.join(here, "hw-pwm.py")
    if not os.path.exists(candidate):
        raise
    spec = importlib.util.spec_from_file_location("hpwm", candidate)
    hpwm = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(hpwm)

from gpiozero import DigitalInputDevice, PWMLED, Button

# ===== IMU (BNO08x) =====
# Uses the "game rotation vector" (gravity-free) for stable yaw.
import board
import busio
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

# =======================
# DriveBase configuration
# =======================
# Encoder
ENCODER_PIN_A = 20
PULSES_PER_REV = 104
GEAR_MOTOR = 16
GEAR_WHEEL = 60
GEAR_RATIO = GEAR_MOTOR / GEAR_WHEEL
WHEEL_DIAMETER_M = 0.0432
WHEEL_CIRCUM_M = math.pi * WHEEL_DIAMETER_M

# Control loop (PI controller) — tuned by you
CTRL_HZ = 90.0
KP = 1.62
KI = 2.5
INT_LIMIT = 1.0

# Software-PWM LED pins (rear + status LEDs)
REAR_LED_PIN = 11
RED_LED_PIN  = 5
GREEN_LED_PIN = 6
LED_PWM_FREQ = 1000

# Switch input (pull-up)
SWITCH_PIN = 4

# IMU streaming
IMU_HZ = 200.0
IMU_BUFFER_SECONDS = 1.5
IMU_BUFFER_SIZE = int(IMU_HZ * IMU_BUFFER_SECONDS) + 4
MAX_YAW_RATE_DPS = 720.0                 # For sanity checks if desired in future
_MAX_DELTA_PER_SAMPLE = MAX_YAW_RATE_DPS / IMU_HZ  # (unused but documented)

def _now():
    """Monotonic time helper (seconds)."""
    return time.monotonic()

def _log(msg: str):
    """Small timestamped logger to stdout."""
    print(f"[{_now():.3f}] {msg}")

def _quat_to_euler_deg(x, y, z, w):
    """
    Convert quaternion -> Euler angles in degrees.
    Returns (roll, pitch, yaw) in deg, yaw ∈ (-180..+180].
    """
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

class DriveBase:
    """
    High-level robot base:
      - Steering (hpwm)
      - Open-loop drive & braking/coast (hpwm)
      - Closed-loop speed in m/s via encoder on GPIO20
      - Signed distance (m)
      - LEDs: Front(HW PWM ch1) mirrors → GREEN(GPIO6); Rear(SW PWM GPIO11) mirrors → RED(GPIO5)
      - Switch (GPIO4, pull-up): get_switch_state()
      - IMU (BNO08x I2C): current yaw + 1.5 s yaw buffer (≥150 Hz)
        * get_current_yaw()              -> latest wrapped yaw (relative to last zero)
        * get_yaw_at(t_monotonic)        -> nearest wrapped yaw in buffer (relative to last zero)
        * get_continuous_yaw()           -> unwrapped yaw (…,-360,0,360,…), zeroed
        * set_yaw_zero()                 -> define current heading as 0° baseline
    """

    def __init__(self):
        # --- hardware setup (one-time) ---
        # Steering servo, LED (HW PWM), and drive H-bridge channels
        hpwm.setup_pwm_servo()
        hpwm.setup_pwm_led()
        hpwm.setup_pwm_drive()

        # SW-PWM LEDs: rear brightness and two status LEDs
        self.rear_led  = PWMLED(REAR_LED_PIN, frequency=LED_PWM_FREQ)
        self.red_led   = PWMLED(RED_LED_PIN,  frequency=LED_PWM_FREQ)
        self.green_led = PWMLED(GREEN_LED_PIN, frequency=LED_PWM_FREQ)

        # Switch input (active low if pulled to GND)
        self.switch = Button(SWITCH_PIN, pull_up=True)

        # Encoder: count pulses for speed/distance
        self._enc = DigitalInputDevice(ENCODER_PIN_A, pull_up=True)
        self._pulse_count = 0
        self._pulse_lock = threading.Lock()
        self._enc.when_activated = self._on_pulse

        # Kinematics state
        self._rpm_motor = 0.0
        self._mps = 0.0               # Signed speed at the wheel (m/s)
        self._distance_m = 0.0        # Signed distance (m); + forward, - backward
        self._last_tick = _now()

        # Drive state
        self._target_speed_mps = 0.0  # Commanded target speed (m/s)
        self._integrator = 0.0        # PI controller integral
        self._last_drive_dir = 0.0    # -1 (reverse), 0 (idle), +1 (forward)
        self._ctrl_running = False
        this_thread = None
        self._ctrl_thread = this_thread

        # Mode state machine for safe transitions around 0 speed
        #   COAST -> no torque (immediate on target=0)
        #   BRAKE_TO_STOP -> full brake to stop before reversing
        #   SPEED -> normal PI control
        self._mode = "COAST"
        self._pending_target_mps = 0.0
        self._stop_counter = 0
        self._STOP_EPS_MPS = 0.02
        self._STOP_CONFIRM_TICKS = 3

        # IMU: I2C + BNO08x in "game rotation vector" mode (reduced gravity influence)
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        self._bno = BNO08X_I2C(i2c)
        self._bno.enable_feature(adafruit_bno08x.BNO_REPORT_GAME_ROTATION_VECTOR)

        # IMU yaw buffer (wrapped yaws relative to current zero)
        self._yaw_buf = deque(maxlen=IMU_BUFFER_SIZE)  # stores (timestamp, yaw_deg)
        self._yaw_lock = threading.Lock()

        # Store both raw and bias-adjusted wrapped yaw
        self._yaw_wrapped_raw = 0.0    # last raw fused yaw (-180..180)
        self._current_yaw = 0.0        # wrapped yaw AFTER bias (relative to zero)

        # Continuous yaw (unwrapped) based on bias-adjusted values
        self._yaw_prev = None          # previous adjusted wrapped yaw
        self._yaw_continuous = 0.0
        self._yaw_zero = 0.0           # kept at 0 when zeroing via bias
        self._yaw_bias = 0.0           # raw fused yaw subtracted so heading becomes 0

        # IMU streaming thread + "first sample ready" signal
        self._imu_running = True
        self._imu_ready = threading.Event()
        self._imu_thread = threading.Thread(target=self._imu_loop, daemon=True)
        self._imu_thread.start()

        # Wait for a real IMU sample, then zero heading to that orientation
        if not self._imu_ready.wait(timeout=0.5):
            _log("Warning: IMU did not produce a sample within 0.5 s")
        self.set_yaw_zero()

        _log("DriveBase initialized (control + IMU threads ready)")

    # =====================
    # Encoder pulse handler
    # =====================
    def _on_pulse(self):
        """ISR-like callback for each encoder A rising edge."""
        with self._pulse_lock:
            self._pulse_count += 1

    # =========
    # Switch API
    # =========
    def get_switch_state(self) -> bool:
        """
        Returns True when the switch input is active (pressed), else False.
        Pull-up input → pressed == pulled low to ground.
        """
        return self.switch.is_pressed

    # ===============
    # Public controls
    # ===============
    def set_steering(self, normalized: float):
        """
        Steering command: normalized ∈ [-1, +1], mapped to servo angle by hw_pwm.
        """
        hpwm.set_steering_angle(normalized)

    def set_drive_power(self, throttle: float):
        """
        Open-loop drive power: throttle ∈ [-1, +1].
        Disables speed control (sets mode based on zero/non-zero).
        """
        self._target_speed_mps = 0.0
        hpwm.set_drive(throttle)
        self._last_drive_dir = 0.0 if throttle == 0 else (1.0 if throttle > 0 else -1.0)
        self._mode = "SPEED" if throttle != 0 else "COAST"

    def brake(self, level: float):
        """Active braking (both H-bridge sides driven). level ∈ [0..1]."""
        self._target_speed_mps = 0.0
        hpwm.brake(level)

    def coast(self):
        """Coast (outputs off)."""
        self._target_speed_mps = 0.0
        hpwm.coast()
        self._last_drive_dir = 0.0
        self._mode = "COAST"

    # --- LEDs ---
    def set_front_led(self, level: float):
        """
        Front LED is HW-PWM (channel 1) and mirrors onto GREEN LED (GPIO6) via SW-PWM.
        level ∈ [0..1]
        """
        hpwm.set_led(level)
        lvl = max(0.0, min(1.0, float(level)))
        self.green_led.value = lvl

    def set_rear_led(self, level: float):
        """
        Rear LED (GPIO11) and RED LED (GPIO5) are SW-PWM and always equal to each other.
        level ∈ [0..1]
        """
        lvl = max(0.0, min(1.0, float(level)))
        self.rear_led.value = lvl
        self.red_led.value = lvl

    # --- Closed-loop speed control (m/s) ---
    def set_target_speed(self, mps: float):
        """
        Set desired wheel speed in m/s.

        Special cases:
        - If target is ~0 → immediate COAST (no torque, integrator cleared).
        - If sign flips across 0 while moving → BRAKE_TO_STOP first, then adopt new target.
        """
        # print("req speed: " + str(mps))
        new = float(mps)

        # Immediate coast on zero target
        if abs(new) < 1e-9:
            self._target_speed_mps = 0.0
            self._integrator = 0.0
            self._pending_target_mps = 0.0
            self._mode = "COAST"
            hpwm.coast()
            self._last_drive_dir = 0.0
            return

        # If sign flips across zero (and we're actually moving), brake hard to stop first
        if self._mps * new < 0 and abs(self._mps) > self._STOP_EPS_MPS:
            self._pending_target_mps = new
            self._target_speed_mps = 0.0
            self._integrator = 0.0
            self._mode = "BRAKE_TO_STOP"
            self._stop_counter = 0
            hpwm.brake(1.0)
            return

        # Normal: enter SPEED mode and track with PI
        self._pending_target_mps = 0.0
        self._target_speed_mps = new
        self._mode = "SPEED"
        # print("speed mode: " + str(self._mode))

    def get_speed(self) -> float:
        """Current signed wheel speed estimate (m/s)."""
        return self._mps

    def get_distance(self) -> float:
        """Accumulated signed distance (m). Positive forward, negative reverse."""
        return self._distance_m

    # ==========
    # IMU getters
    # ==========
    def get_current_yaw(self) -> float:
        """
        Latest wrapped yaw (deg, -180..+180) relative to the last zero.
        This is bias-adjusted so that set_yaw_zero() defines current heading as 0°.
        """
        return self._current_yaw

    def get_continuous_yaw(self) -> float:
        """
        Unwrapped yaw (deg) that continues beyond ±180 (…, -360, 0, +360, …),
        relative to the last zero.
        """
        with self._yaw_lock:
            return self._yaw_continuous - self._yaw_zero  # yaw_zero kept at 0 on zero()

    def set_yaw_zero(self):
        """
        Define the current IMU heading as 0° baseline by:
        - Capturing the latest RAW fused yaw as a bias
        - Resetting unwrap state and continuous yaw accumulator
        """
        with self._yaw_lock:
            self._yaw_bias = self._yaw_wrapped_raw
            self._yaw_prev = 0.0
            self._yaw_continuous = 0.0
            self._yaw_zero = 0.0
            self._current_yaw = 0.0  # immediate user feedback

    def get_yaw_at(self, t_monotonic: float) -> float:
        """
        Return the wrapped yaw (relative to zero) closest in time to the given
        monotonic timestamp (seconds). Useful to align IMU samples with other events.
        """
        with self._yaw_lock:
            if not self._yaw_buf:
                return self._current_yaw
            ts = [pair[0] for pair in self._yaw_buf]
            idx = bisect.bisect_left(ts, t_monotonic)
            if idx <= 0:
                return self._yaw_buf[0][1]
            if idx >= len(ts):
                return self._yaw_buf[-1][1]
            t0, y0 = self._yaw_buf[idx-1]
            t1, y1 = self._yaw_buf[idx]
            return y0 if abs(t_monotonic - t0) <= abs(t1 - t_monotonic) else y1

    # ================
    # Control loop impl
    # ================
    def start(self):
        """Start the high-rate control loop thread."""
        if self._ctrl_running:
            return
        self._ctrl_running = True
        self._ctrl_thread = threading.Thread(target=self._ctrl_loop, daemon=True)
        self._ctrl_thread.start()
        _log("Control loop started")

    def stop(self):
        """Stop the control loop thread."""
        self._ctrl_running = False
        if self._ctrl_thread:
            self._ctrl_thread.join(timeout=1.0)
            self._ctrl_thread = None
        _log("Control loop stopped")

    def _ctrl_loop(self):
        """
        Main loop running at CTRL_HZ:
        - Converts encoder pulses → wheel speed (m/s) + signed distance
        - Applies mode logic (COAST, BRAKE_TO_STOP, SPEED)
        - In SPEED mode, runs PI controller to match _target_speed_mps
        - IMPORTANT GUARD: if |mps| > 1.6 (physically impossible here), set to 0 for this tick
          to reject encoder glitches that would destabilize the controller.
        """
        period = 1.0 / CTRL_HZ
        while self._ctrl_running:
            t0 = _now()

            # ----- Consume encoder pulses accumulated since last tick -----
            with self._pulse_lock:
                pulses = self._pulse_count
                self._pulse_count = 0

            # Time delta for this tick
            dt = t0 - self._last_tick
            if dt <= 0:
                dt = period
            self._last_tick = t0

            # ----- Kinematics from encoder -----
            pps = pulses / dt if dt > 0 else 0.0
            rpm_motor = (pps * 60.0) / PULSES_PER_REV
            wheel_rps = (rpm_motor / 60.0) * GEAR_RATIO
            mps_unsigned = wheel_rps * WHEEL_CIRCUM_M

            # Direction sign: last drive command OR target speed sign in SPEED mode
            sign = self._last_drive_dir
            if self._mode == "SPEED" and self._target_speed_mps != 0:
                sign = 1.0 if self._target_speed_mps > 0 else -1.0

            self._rpm_motor = rpm_motor
            self._mps = mps_unsigned * sign
            self._distance_m += (mps_unsigned * sign) * dt

            # ===== Drive mode handling =====
            if self._mode == "COAST":
                # Ensure coast (idempotent)
                hpwm.coast()
                self._last_drive_dir = 0.0

            elif self._mode == "BRAKE_TO_STOP":
                # Full brake until speed is near zero for a few ticks
                hpwm.brake(1.0)
                if abs(self._mps) < self._STOP_EPS_MPS:
                    self._stop_counter += 1
                else:
                    self._stop_counter = 0

                if self._stop_counter >= self._STOP_CONFIRM_TICKS:
                    # Transition to SPEED mode with pending target
                    self._integrator = 0.0
                    self._last_drive_dir = 0.0
                    self._target_speed_mps = self._pending_target_mps
                    self._pending_target_mps = 0.0
                    self._mode = "SPEED"

            else:  # "SPEED"
                if self._target_speed_mps == 0.0:
                    # Safety: shouldn't happen (set_target_speed(0) forces COAST)
                    hpwm.coast()
                    self._last_drive_dir = 0.0
                else:
                    # ---- Physical clamp against encoder glitches ----
                    if abs(self._mps) > 1.6:
                        # Known impossible on this platform → treat as 0 for this tick.
                        self._mps = 0

                    # ---- PI controller ----
                    error = self._target_speed_mps - self._mps
                    self._integrator += error * dt * KI
                    self._integrator = max(-INT_LIMIT, min(INT_LIMIT, self._integrator))
                    u = KP * error + self._integrator
                    u = max(-1.0, min(1.0, u))

                    # ---- Limit opposite torque vs target direction ----
                    # If PI asks for torque opposite to the *target* direction, cap it to ±0.1
                    desired_dir = 1.0 if self._target_speed_mps > 0 else -1.0
                    if u * desired_dir < 0:
                        if abs(u) > 0.1:
                            u = 0.0 if u > 0 else 0.0

                    hpwm.set_drive(u)
                    self._last_drive_dir = 0.0 if u == 0 else (1.0 if u > 0 else -1.0)

            # Maintain loop rate
            elapsed = _now() - t0
            to_sleep = period - elapsed
            if to_sleep > 0:
                time.sleep(to_sleep)

    # ================
    # IMU streamer impl
    # ================
    def _imu_loop(self):
        """
        IMU thread at IMU_HZ:
        - Reads quaternion from BNO08X game rotation vector
        - Converts to wrapped yaw (deg)
        - Applies bias (set_yaw_zero) so current heading is 0°
        - Maintains a 1.5s buffer of wrapped yaw samples
        - Builds a continuous yaw (unwrapped) by accumulating small wrapped deltas
        """
        period = 1.0 / IMU_HZ
        while self._imu_running:
            t0 = _now()
            try:
                qi, qj, qk, qr = self._bno.game_quaternion
                _, _, yaw_wrapped = _quat_to_euler_deg(qi, qj, qk, qr)

                with self._yaw_lock:
                    # keep raw for future biasing
                    self._yaw_wrapped_raw = yaw_wrapped

                    # apply current bias to get RELATIVE wrapped yaw in (-180,180]
                    yaw_rel = ((yaw_wrapped - self._yaw_bias + 180.0) % 360.0) - 180.0

                    # update current yaw and add to buffer
                    self._current_yaw = yaw_rel
                    self._yaw_buf.append((t0, yaw_rel))

                    # first-sample signal
                    if not hasattr(self, "_imu_ready") or self._imu_ready is None:
                        pass
                    else:
                        self._imu_ready.set()

                    # Unwrap to continuous yaw on the relative values
                    if self._yaw_prev is None:
                        self._yaw_prev = yaw_rel
                        # continuous yaw was set to 0 on zeroing
                    else:
                        delta = (yaw_rel - self._yaw_prev + 180.0) % 360.0 - 180.0
                        self._yaw_continuous += delta
                        self._yaw_prev = yaw_rel

            except Exception as e:
                _log(f"IMU read error: {e}")

            elapsed = _now() - t0
            to_sleep = period - elapsed
            if to_sleep > 0:
                time.sleep(to_sleep)

    # ==========
    # Housekeeping
    # ==========
    def shutdown(self):
        """
        Gracefully stop threads and put hardware in a safe state.
        """
        try:
            self.stop()
            self._imu_running = False
            if self._imu_thread:
                self._imu_thread.join(timeout=1.0)
                self._imu_thread = None

            self.set_target_speed(0.0)
            self.set_drive_power(0.0)
            self.coast()
            self.set_front_led(0.0)
            self.set_rear_led(0.0)
            hpwm.cleanup_pwm_servo()
            for p in (hpwm.PWM1_PATH, hpwm.PWM2_PATH, hpwm.PWM3_PATH):
                try:
                    hpwm.write(f"{p}/enable", 0)
                except:
                    pass
        finally:
            _log("Shutdown complete")

# -----------------------
# Minimal yaw + drive test (prints until Ctrl+C)
# -----------------------
if __name__ == "__main__":
    """
    Demo:
      1) Start controller + zero yaw
      2) Accelerate to +0.5 m/s for 3s, then set target=0 → should coast instantly
      3) Accelerate to +0.4 m/s for 3s, then request -0.1 m/s → should brake-to-stop, then reverse
      4) Print yaw (wrapped & continuous), current mode, speed, and target every 50 ms
    """
    db = DriveBase()
    try:
        db.start()
        time.sleep(0.5)
        db.set_yaw_zero()
        _log("Continuous yaw test: printing current yaw and ~200 ms ago. Press Ctrl+C to stop.")

        # --- Drive behavior tests ---
        _log("Test A: accelerate to +0.5 m/s for 3s, then set target to 0 (should instant COAST).")
        db.set_target_speed(0.5)
        time.sleep(3.0)
        _log("Setting target speed to 0.0 (expect immediate coast).")
        db.set_target_speed(0.0)
        time.sleep(2.0)

        _log("Test B: accelerate to +0.4 m/s for 3s, then request -0.1 m/s (should BRAKE to stop first).")
        db.set_target_speed(0.4)
        time.sleep(3.0)
        _log("Requesting -0.1 m/s (full BRAKE to stop first).")
        db.set_target_speed(-0.1)

        # --- Continuous print loop ---
        while True:
            now = _now()
            curr_wrapped = db.get_current_yaw()   # relative to zero
            curr_cont = db.get_continuous_yaw()   # relative to zero
            past_wrapped = db.get_yaw_at(now - 0.2)  # relative to zero

            print(
                f"now: yaw_wrapped={curr_wrapped:+7.2f}°, "
                f"yaw_cont={curr_cont:+8.2f}° | "
                f"~200ms ago: yaw_wrapped={past_wrapped:+7.2f}°   "
                f"| mode={db._mode} v={db.get_speed():+.2f} m/s tgt={db._target_speed_mps:+.2f}"
            )

            time.sleep(0.05)

    except KeyboardInterrupt:
        db.set_target_speed(0.0)
        print("\nInterrupted by user")
    finally:
        db.shutdown()
