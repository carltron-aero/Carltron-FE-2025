#!/usr/bin/env python3
import lgpio
import time
import signal
import sys

# === CONFIGURATION ===

GPIO_CHIP = 4      # as specified
PIN1 = 23          # BCM pin wired to DRV8871 IN1
PIN2 = 24          # BCM pin wired to DRV8871 IN2

# PWM frequency for lgpio.tx_pwm (Hz): max 10000 as requested
FREQ_HZ = 10000

# Global handle
h = None

def setup():
    """
    Open gpiochip, claim PIN1 and PIN2 as outputs, initialize LOW.
    """
    global h
    h = lgpio.gpiochip_open(GPIO_CHIP)
    lgpio.gpio_claim_output(h, PIN1)
    lgpio.gpio_claim_output(h, PIN2)
    # Initialize both LOW (coast if PWM disabled)
    lgpio.gpio_write(h, PIN1, 0)
    lgpio.gpio_write(h, PIN2, 0)
    print(f"GPIO chip {GPIO_CHIP} opened; pins {PIN1},{PIN2} claimed as outputs LOW")

def cleanup():
    """
    Stop PWM on both pins, set both LOW, close handle.
    """
    global h
    if h is not None:
        # Stop any PWM
        try:
            lgpio.tx_pwm(h, PIN1, FREQ_HZ, 0)
        except Exception:
            pass
        try:
            lgpio.tx_pwm(h, PIN2, FREQ_HZ, 0)
        except Exception:
            pass
        # Finally set static LOW (coast)
        lgpio.gpio_write(h, PIN1, 0)
        lgpio.gpio_write(h, PIN2, 0)
        lgpio.gpiochip_close(h)
        h = None
        print("Cleaned up: PWM stopped, pins LOW, gpiochip closed")

def set_speed(value):
    """
    Control motor speed and direction:
      - value ∈ [-1.0, 1.0]
      - value > 0: forward at 100*value % duty
      - value < 0: reverse at 100*abs(value)% duty
      - value == 0: brake (both inputs HIGH → slow-decay brake) :contentReference[oaicite:0]{index=0}
    """
    global h
    # First, stop any ongoing PWM on both pins (duty=0 disables PWM)
    try:
        lgpio.tx_pwm(h, PIN1, FREQ_HZ, 0)
    except Exception:
        pass
    try:
        lgpio.tx_pwm(h, PIN2, FREQ_HZ, 0)
    except Exception:
        pass

    # Brake
    if abs(value) < 1e-6:
        # Both HIGH for brake :contentReference[oaicite:1]{index=1}
        lgpio.gpio_write(h, PIN1, 1)
        lgpio.gpio_write(h, PIN2, 1)
        print("Brake applied (both HIGH)")
        return

    duty = min(100.0, max(0.0, abs(value) * 100.0))
    if value > 0:
        # Forward: PWM on PIN1, PIN2 LOW
        lgpio.gpio_write(h, PIN2, 0)
        # Start PWM on PIN1
        lgpio.tx_pwm(h, PIN1, FREQ_HZ, duty)
        print(f"Forward at {duty:.1f}% duty")
    else:
        # Reverse: PWM on PIN2, PIN1 LOW
        lgpio.gpio_write(h, PIN1, 0)
        lgpio.tx_pwm(h, PIN2, FREQ_HZ, duty)
        print(f"Reverse at {duty:.1f}% duty")

def set_coast():
    """
    Coast mode: both inputs LOW → low-power sleep / coast :contentReference[oaicite:2]{index=2}
    """
    global h
    # Disable PWM
    try:
        lgpio.tx_pwm(h, PIN1, FREQ_HZ, 0)
    except Exception:
        pass
    try:
        lgpio.tx_pwm(h, PIN2, FREQ_HZ, 0)
    except Exception:
        pass
    # Both LOW
    lgpio.gpio_write(h, PIN1, 0)
    lgpio.gpio_write(h, PIN2, 0)
    print("Coast applied (both LOW)")

def signal_handler(sig, frame):
    print("\nSignal received, cleaning up...")
    cleanup()
    sys.exit(0)

if __name__ == "__main__":
    # Trap signals for clean exit
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    setup()

    try:
        # Example usage:
        print("Running example sequence: forward 70%, brake, coast, reverse 50%, brake, coast.")
        set_speed(0.7)
        time.sleep(5)
        set_speed(-7.0)  # brake
        time.sleep(2)
        set_coast()
        time.sleep(2)
        set_speed(-0.9)
        time.sleep(5)
        set_speed(0.0)  # brake
        time.sleep(2)
        set_coast()
        print("Example sequence done. Entering idle (coast). Ctrl+C to exit.")
        # Keep coasting until interrupted
        while True:
            time.sleep(1)
    except Exception as e:
        print(f"Exception: {e}")
    finally:
        cleanup()
        print("Exited cleanly.")
