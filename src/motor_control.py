#!/usr/bin/env python3
"""
Module: motor_control.py

Provides MotorController class to manage GPIO-based motor control using lgpio.

Usage:
    from motor_control import MotorController
    mc = MotorController()
    mc.setup()
    mc.set_speed(0.5)  # forward 50%
    mc.set_coast()
    mc.cleanup()

Optionally register signal handlers:
    mc.register_signal_handlers()

"""
import lgpio
import signal
import sys

# Default configuration constants
DEFAULT_GPIO_CHIP = 4    # BCM gpiochip number
DEFAULT_PIN1 = 23        # BCM pin wired to DRV8871 IN1
DEFAULT_PIN2 = 24        # BCM pin wired to DRV8871 IN2
DEFAULT_FREQ_HZ = 10000   # PWM frequency (Hz)

class MotorController:
    def __init__(self, gpio_chip=DEFAULT_GPIO_CHIP, pin1=DEFAULT_PIN1, pin2=DEFAULT_PIN2, freq_hz=DEFAULT_FREQ_HZ):
        """
        Initialize MotorController with specified GPIO chip and pins.
        :param gpio_chip: GPIO chip number (BCM gpiochip index)
        :param pin1: BCM pin for IN1
        :param pin2: BCM pin for IN2
        :param freq_hz: PWM frequency in Hz
        """
        self.gpio_chip = gpio_chip
        self.pin1 = pin1
        self.pin2 = pin2
        self.freq_hz = freq_hz
        self.h = None

    def setup(self):
        """
        Open gpiochip, claim pins as outputs, initialize LOW.
        """
        if self.h is not None:
            # Already set up
            return
        try:
            self.h = lgpio.gpiochip_open(self.gpio_chip)
            lgpio.gpio_claim_output(self.h, self.pin1)
            lgpio.gpio_claim_output(self.h, self.pin2)
            # Initialize both LOW (coast if PWM disabled)
            lgpio.gpio_write(self.h, self.pin1, 0)
            lgpio.gpio_write(self.h, self.pin2, 0)
            print(f"Initialized GPIO chip {self.gpio_chip}; pins {self.pin1},{self.pin2} set as outputs LOW")
        except Exception as e:
            self.h = None
            raise RuntimeError(f"Failed to setup MotorController: {e}")

    def cleanup(self):
        """
        Stop PWM, set pins LOW, and close gpiochip handle.
        """
        if self.h is None:
            return
        try:
            # Stop any PWM by sending duty=0
            try:
                lgpio.tx_pwm(self.h, self.pin1, self.freq_hz, 0)
            except Exception:
                pass
            try:
                lgpio.tx_pwm(self.h, self.pin2, self.freq_hz, 0)
            except Exception:
                pass
            # Finally set static LOW (coast)
            lgpio.gpio_write(self.h, self.pin1, 0)
            lgpio.gpio_write(self.h, self.pin2, 0)
            lgpio.gpiochip_close(self.h)
            print(f"Cleaned up GPIO chip {self.gpio_chip}: PWM stopped, pins LOW, handle closed")
        except Exception as e:
            print(f"Warning: exception during cleanup: {e}")
        finally:
            self.h = None

    def set_speed(self, value):
        """
        Control motor speed and direction:
          - value ∈ [-1.0, 1.0]
          - value > 0: forward at 100*value % duty
          - value < 0: reverse at 100*abs(value)% duty
          - value == 0: brake (both inputs HIGH → slow-decay brake)
        """
        if self.h is None:
            raise RuntimeError("GPIO handle not initialized; call setup() before set_speed()")
        # Stop any ongoing PWM on both pins
        try:
            lgpio.tx_pwm(self.h, self.pin1, self.freq_hz, 0)
        except Exception:
            pass
        try:
            lgpio.tx_pwm(self.h, self.pin2, self.freq_hz, 0)
        except Exception:
            pass

        # Brake if near zero
        if abs(value) < 1e-6:
            # Both HIGH for brake (slow-decay)
            lgpio.gpio_write(self.h, self.pin1, 1)
            lgpio.gpio_write(self.h, self.pin2, 1)
            print("Brake applied (both HIGH)")
            return

        # Compute duty percentage
        duty = min(100.0, max(0.0, abs(value) * 100.0))
        if value > 0:
            # Forward: PWM on pin1, pin2 LOW
            lgpio.gpio_write(self.h, self.pin2, 0)
            lgpio.tx_pwm(self.h, self.pin1, self.freq_hz, duty)
            print(f"Forward at {duty:.1f}% duty on pin {self.pin1}")
        else:
            # Reverse: PWM on pin2, pin1 LOW
            lgpio.gpio_write(self.h, self.pin1, 0)
            lgpio.tx_pwm(self.h, self.pin2, self.freq_hz, duty)
            print(f"Reverse at {duty:.1f}% duty on pin {self.pin2}")

    def set_coast(self):
        """
        Coast mode: both inputs LOW → low-power sleep / coast
        """
        if self.h is None:
            raise RuntimeError("GPIO handle not initialized; call setup() before set_coast()")
        # Disable PWM on both pins
        try:
            lgpio.tx_pwm(self.h, self.pin1, self.freq_hz, 0)
        except Exception:
            pass
        try:
            lgpio.tx_pwm(self.h, self.pin2, self.freq_hz, 0)
        except Exception:
            pass
        # Both LOW
        lgpio.gpio_write(self.h, self.pin1, 0)
        lgpio.gpio_write(self.h, self.pin2, 0)
        print("Coast applied (both LOW)")

    def register_signal_handlers(self):
        """
        Register SIGINT and SIGTERM handlers to cleanup on exit.
        """
        def _handler(sig, frame):
            print(f"Signal {sig} received, cleaning up and exiting...")
            try:
                self.cleanup()
            except Exception as e:
                print(f"Exception during cleanup: {e}")
            sys.exit(0)

        signal.signal(signal.SIGINT, _handler)
        signal.signal(signal.SIGTERM, _handler)
        print("Signal handlers registered for SIGINT and SIGTERM")


if __name__ == "__main__":
    # Example usage when running this module directly
    mc = MotorController()
    mc.register_signal_handlers()
    try:
        mc.setup()
        print("Running example sequence: forward 70%, brake, coast, reverse 90%, brake, coast.")
        mc.set_speed(0.7)
        time.sleep(5)
        mc.set_speed(0.0)  # brake
        time.sleep(2)
        mc.set_coast()
        time.sleep(2)
        mc.set_speed(-0.9)
        time.sleep(5)
        mc.set_speed(0.0)  # brake
        time.sleep(2)
        mc.set_coast()
        print("Example sequence done. Entering idle (coast). Ctrl+C to exit.")
        while True:
            time.sleep(1)
    except Exception as e:
        print(f"Exception in main: {e}")
    finally:
        mc.cleanup()
        print("Exited cleanly.")
