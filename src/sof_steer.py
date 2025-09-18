#!/usr/bin/env python3
"""
Servo steering test using lgpio with tx_servo.
Updated to include set_steering_angle(normalized) function.
"""

import time
import lgpio
import sys
import signal

# Configuration
GPIOCHIP = 4       # gpiochip index (e.g. /dev/gpiochip4)
SERVO_GPIO = 4     # GPIO pin for servo
FREQ = 50          # Servo PWM frequency in Hz
MIN_PW = 500       # Min pulse width (μs)
MAX_PW = 2500      # Max pulse width (μs)
PULSE_OFFSET = 0   # Pulse start offset (μs)

# Globals
handle = None

def angle_to_pulse(angle):
    """
    Map steering angle (50–130°) to pulse width.
    """
    # Clamp angle to safe range
    angle = min(max(50.0, angle), 130.0)
    # Map 50–130° to pulse width between MIN_PW and MAX_PW
    return int(MIN_PW + (angle - 0) * (MAX_PW - MIN_PW) / 180.0)

def set_steering_angle(normalized):
    """
    Set servo steering based on normalized input in [-1, 1].

    Maps to angle in [50, 130], then converts to pulse width.
    Uses tx_servo with infinite cycles.
    """
    global handle

    # Clamp input
    x = max(-1.0, min(1.0, normalized))
    angle = 40.0 * x + 90.0  # [-1,1] → [50,130]
    angle = min(max(50.0, angle), 130.0)
    print(f"Setting angle: {angle:.1f}°")

    pulse_width = angle_to_pulse(angle)
    print(f"Pulse width: {pulse_width} μs")

    # Apply using tx_servo (0 = infinite cycles)
    r = lgpio.tx_servo(handle, SERVO_GPIO, pulse_width,
                       servo_frequency=FREQ,
                       pulse_offset=PULSE_OFFSET,
                       pulse_cycles=0)
    if r < 0:
        print(f"tx_servo error: {lgpio.lguErrorText(r)} ({r})", file=sys.stderr)

def cleanup():
    """
    Stop pulses and release GPIO.
    """
    global handle
    if handle is not None:
        try:
            lgpio.tx_servo(handle, SERVO_GPIO, 0,
                           servo_frequency=FREQ,
                           pulse_offset=0,
                           pulse_cycles=1)  # send stop pulse
        except Exception:
            pass
        try:
            lgpio.gpio_free(handle, SERVO_GPIO)
        except Exception:
            pass
        try:
            lgpio.gpiochip_close(handle)
        except Exception:
            pass

def main():
    global handle

    # Open gpiochip
    handle = lgpio.gpiochip_open(GPIOCHIP)
    if handle < 0:
        print(f"Failed to open gpiochip {GPIOCHIP}: {lgpio.lguErrorText(handle)}", file=sys.stderr)
        sys.exit(1)

    # Signal handler
    def sigint_handler(sig, frame):
        print("\nInterrupted.")
        cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, sigint_handler)

    # Claim GPIO for output
    r = lgpio.gpio_claim_output(handle, 0, SERVO_GPIO, 0)
    if r < 0:
        print(f"Failed to claim GPIO {SERVO_GPIO}: {lgpio.lguErrorText(r)} ({r})", file=sys.stderr)
        cleanup()
        sys.exit(1)

    print("Testing set_steering_angle(normalized). Press Ctrl+C to stop.")
    try:
        while True:
            for x in [-1.0, -0.5, 0.0, 0.5, 1.0]:
                set_steering_angle(x)
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()
        print("Done.")

if __name__ == "__main__":
    main()
