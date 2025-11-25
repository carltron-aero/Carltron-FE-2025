#!/usr/bin/env python3
"""
hw_pwm

Low-level hardware-PWM control for Raspberry Pi 5 via sysfs (/sys/class/pwm).

Channels (on pwmchip0):
  0: Steering servo (50 Hz, RC pulses 0.5–2.5 ms)
  1: Front LED (0..1 duty) — high-rate dimming
  2: Drive reverse  (0..1 duty) — H-bridge leg
  3: Drive forward  (0..1 duty) — H-bridge leg

This module exposes:
  - setup_pwm_servo(), set_steering_angle(), cleanup_pwm_servo()
  - setup_pwm_led(),   set_led(level)
  - setup_pwm_drive(), set_drive(throttle), brake(level), coast()

"""

import os
import time

# === HW PWM CONFIGURATION ===
PWM_CHIP = 0  # pwmchip0

# Channel mapping:
# 0: Steering servo (50 Hz RC pulse)
# 1: LED (0..1)
# 2: Drive reverse  (0..1)
# 3: Drive forward  (0..1)

# Frequencies
SERVO_FREQ_HZ = 50
LED_FREQ_HZ   = 1000
DRIVE_FREQ_HZ = 20000

# Periods in nanoseconds (sysfs expects ns)
SERVO_PERIOD_NS = (1_000_000 // SERVO_FREQ_HZ) * 1000
LED_PERIOD_NS   = (1_000_000 // LED_FREQ_HZ)   * 1000
DRIVE_PERIOD_NS = (1_000_000 // DRIVE_FREQ_HZ) * 1000

# Servo pulse range (in microseconds)
MIN_PULSE_US = 500
MAX_PULSE_US = 2500
MIN_PULSE_NS = MIN_PULSE_US * 1000
MAX_PULSE_NS = MAX_PULSE_US * 1000

# === Paths ===
BASE_PATH   = f"/sys/class/pwm/pwmchip{PWM_CHIP}"
PWM0_PATH   = f"{BASE_PATH}/pwm0"  # Servo
PWM1_PATH   = f"{BASE_PATH}/pwm1"  # LED
PWM2_PATH   = f"{BASE_PATH}/pwm2"  # Drive reverse
PWM3_PATH   = f"{BASE_PATH}/pwm3"  # Drive forward

def write(path, value):
    """Write a stringified value to a sysfs file."""
    with open(path, "w") as f:
        f.write(str(value))

def ensure_exported(pwm_path, channel_index):
    """
    Ensure that /sys/class/pwm/pwmchipX/pwmY exists by exporting Y if missing.
    Small sleep gives the kernel time to create the node.
    """
    if not os.path.exists(pwm_path):
        write(f"{BASE_PATH}/export", channel_index)
        time.sleep(0.05)

def setup_channel(pwm_path, period_ns, initial_duty_ns=0):
    """
    Disable → set period → set duty → enable.
    NOTE: Uses the last character of pwm_path as channel index (0..3).
    """
    ensure_exported(pwm_path, int(pwm_path[-1]))  # uses last char (0..3)
    try: write(f"{pwm_path}/enable", 0)
    except: pass
    write(f"{pwm_path}/period", period_ns)
    write(f"{pwm_path}/duty_cycle", initial_duty_ns)
    write(f"{pwm_path}/enable", 1)

def set_fraction(pwm_path, period_ns, frac):
    """
    Set duty as a fraction (0..1) of the given period in ns.
    Clamped to [0,1].
    """
    x = max(0.0, min(1.0, float(frac)))
    duty_ns = int(period_ns * x)
    write(f"{pwm_path}/duty_cycle", duty_ns)

# === Servo (channel 0) ===
def angle_to_duty_ns(angle):
    """
    Map servo angle (0..180°) to duty in ns within [0.5ms .. 2.5ms].
    Input is clamped to [0,180].
    """
    angle = max(0, min(180, angle))
    return int(MIN_PULSE_NS + (MAX_PULSE_NS - MIN_PULSE_NS) * angle // 180)

def setup_pwm_servo():
    """Export & enable servo PWM at 50 Hz, and center the pulse (~1.5ms)."""
    setup_channel(PWM0_PATH, SERVO_PERIOD_NS, 1_500_000)  # center

def cleanup_pwm_servo():
    """Disable the servo channel (does not unexport)."""
    write(f"{PWM0_PATH}/enable", 0)

def set_steering_angle(normalized):
    """
    Steering command: normalized in [-1, +1].
    Mapped to 45°..135° (±45° around 90° center) and converted to RC pulse width.
    """
    x = max(-1.0, min(1.0, normalized))
    angle = 45.0 * x + 90.0
    angle = min(max(45, angle), 135)
    duty_ns = angle_to_duty_ns(angle)
    write(f"{PWM0_PATH}/duty_cycle", duty_ns)

# === LED (channel 1) ===
def setup_pwm_led():
    """Export & enable LED PWM at 1 kHz, duty = 0%."""
    setup_channel(PWM1_PATH, LED_PERIOD_NS, 0)

def set_led(level):
    """Set LED brightness as fraction (0..1)."""
    set_fraction(PWM1_PATH, LED_PERIOD_NS, level)

# === Drive (channels 2 & 3) ===
def setup_pwm_drive():
    """Export & enable both H-bridge legs at 20 kHz, duty = 0%."""
    setup_channel(PWM2_PATH, DRIVE_PERIOD_NS, 0)
    setup_channel(PWM3_PATH, DRIVE_PERIOD_NS, 0)

def set_drive(throttle):
    """
    Bidirectional drive:
      throttle ∈ [-1, +1]
        > 0 → forward leg (pwm3) gets |throttle|, reverse leg off
        < 0 → reverse leg (pwm2) gets |throttle|, forward leg off
        = 0 → both legs off (coast)
    """
    #print("Drive Power: " + str(throttle))
    t = max(-1.0, min(1.0, float(throttle)))
    if t > 0:
        set_fraction(PWM2_PATH, DRIVE_PERIOD_NS, 0.0)  # reverse off
        set_fraction(PWM3_PATH, DRIVE_PERIOD_NS, t)    # forward on
    elif t < 0:
        set_fraction(PWM3_PATH, DRIVE_PERIOD_NS, 0.0)  # forward off
        set_fraction(PWM2_PATH, DRIVE_PERIOD_NS, -t)   # reverse on
    else:
        set_fraction(PWM2_PATH, DRIVE_PERIOD_NS, 0.0)
        set_fraction(PWM3_PATH, DRIVE_PERIOD_NS, 0.0)

def brake(level):
    """
    Active braking: drive both legs with the same duty (0..1),
    which shorts the motor through the H-bridge (depending on wiring).
    """
    lvl = max(0.0, min(1.0, float(level)))
    set_fraction(PWM2_PATH, DRIVE_PERIOD_NS, lvl)
    set_fraction(PWM3_PATH, DRIVE_PERIOD_NS, lvl)

def coast():
    """High-impedance/coast: both legs off."""
    set_fraction(PWM2_PATH, DRIVE_PERIOD_NS, 0.0)
    set_fraction(PWM3_PATH, DRIVE_PERIOD_NS, 0.0)

# === Logging helper for tests ===
_t0 = time.monotonic()
def log(msg):
    """Timestamped print helper for the demo below."""
    dt = time.monotonic() - _t0
    print(f"[{dt:6.3f}s] {msg}")

# === Demo / Tests ===
if __name__ == "__main__":
    """
    Basic manual test flow:
      - Initialize all channels
      - LED fade up/down
      - Drive ramps reverse→neutral→forward
      - Compare "full forward → brake" vs "full forward → coast"
      - Brake sweep
      - Smooth steering sweep back and forth
    Ctrl+C to exit; shutdown puts hardware into a safe state.
    """
    try:
        print(f"Starting: pwmchip{PWM_CHIP}")
        setup_pwm_servo()
        setup_pwm_led()
        setup_pwm_drive()

        # --- LED fade test ---
        log("LED fade: 0% → 100%")
        for i in range(0, 101):
            set_led(i/100.0)
            time.sleep(0.005)
        log("LED fade: 100% → 0%")
        for i in range(100, -1, -1):
            set_led(i/100.0)
            time.sleep(0.005)
        set_led(0.2)
        log("LED set to 20%")

        # --- Drive ramp test ---
        log("Drive ramp: reverse -1.0 → 0.0")
        for i in range(0, 101):
            set_drive(-i/100.0)
            if i % 20 == 0:
                log(f"Drive throttle = {-i/100.0:+.2f}")
            time.sleep(0.01)

        log("Drive ramp: 0.0 → +1.0 forward")
        for i in range(0, 101):
            set_drive(i/100.0)
            if i % 20 == 0:
                log(f"Drive throttle = {i/100.0:+.2f}")
            time.sleep(0.01)

        # --- Comparison test: Full forward → BRAKE vs → COAST ---
        # A) Forward → BRAKE(100%)
        log("Comparison A: FULL FORWARD (+1.0) for 1.5s")
        set_drive(1.0)
        time.sleep(1.5)
        log("Apply BRAKE 100% for 1.5s")
        brake(1.0)
        time.sleep(1.5)
        log("Coast for 0.5s (after braking)")
        coast()
        time.sleep(0.5)

        # Cool-down pause
        log("Pause 1.0s before Comparison B")
        time.sleep(1.0)

        # B) Forward → COAST
        log("Comparison B: FULL FORWARD (+1.0) for 1.5s")
        set_drive(1.0)
        time.sleep(1.5)
        log("COAST for 1.5s")
        coast()
        time.sleep(1.5)

        # Restore neutral
        set_drive(0.0)
        coast()
        log("Drive neutral")

        # --- Brake strength sweep (optional extra insight) ---
        log("Brake strength sweep: 0% → 100% → 0%")
        for i in range(0, 101, 5):
            brake(i/100.0)
            time.sleep(0.05)
        for i in range(100, -1, -5):
            brake(i/100.0)
            time.sleep(0.05)
        coast()
        log("Brake released; coasting")

        # --- Steering smooth sweep loop ---
        log("Steering sweep: -1.0 ↔ +1.0 (loop)")
        steps = 200
        dwell = 0.01
        while True:
            for i in range(steps + 1):
                x = -1.0 + 2.0 * i / steps
                set_steering_angle(x)
                if i % 50 == 0:
                    log(f"Steering normalized = {x:+.2f}")
                time.sleep(dwell)
            for i in range(steps + 1):
                x = 1.0 - 2.0 * i / steps
                set_steering_angle(x)
                if i % 50 == 0:
                    log(f"Steering normalized = {x:+.2f}")
                time.sleep(dwell)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        # Safe shutdown states
        log("Shutdown: center steering, LED off, motor coast")
        set_steering_angle(0.0)
        set_led(0.0)
        coast()
        time.sleep(0.4)
        cleanup_pwm_servo()
        for p in (PWM1_PATH, PWM2_PATH, PWM3_PATH):
            try: write(f"{p}/enable", 0)
            except: pass
        print("PWM disabled.")
