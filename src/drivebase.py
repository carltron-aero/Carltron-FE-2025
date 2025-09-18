#!/usr/bin/env python3
"""
Module: drive_base.py

Provides DriveBase class that integrates IO control (switch and LED), motor control, steering, and IMU yaw tracking.
Also includes main() to test capabilities: IO, steering, motor, IMU.

Usage:
    from drive_base import DriveBase
    db = DriveBase()
    db.register_signal_handlers()
    db.setup_all()
    db.reset_yaw_reference()
    try:
        while True:
            yaw = db.get_continuous_yaw()
            print(f"Continuous Yaw: {yaw:.2f}°")
            # Example: drive and steer
            # db.set_speed(0.5)
            # db.set_steering(0.2)
            # IO example: if db.get_button_state(): db.set_led(True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        db.cleanup_all()
"""
import os
import time
import signal
import sys
import io_control
from motor_control import MotorController
from imu_control import BNO08XController

class DriveBase:
    def __init__(self,
                 # IO control: uses io_control module
                 # MotorController parameters
                 motor_gpio_chip=None,
                 motor_pin1=None,
                 motor_pin2=None,
                 motor_freq_hz=None,
                 # IMU parameters
                 i2c_bus=None,
                 scl=None,
                 sda=None,
                 i2c_frequency=400000,
                 i2c_address=0x4A):
        """
        Initialize DriveBase combining IO control, motor controller, and IMU controller.
        :param motor_gpio_chip: GPIO chip index for MotorController (default)
        :param motor_pin1: MotorController pin1 (default)
        :param motor_pin2: MotorController pin2 (default)
        :param motor_freq_hz: MotorController PWM frequency (default)
        :param i2c_bus: existing I2C bus instance
        :param scl: I2C SCL pin if creating new bus
        :param sda: I2C SDA pin if creating new bus
        :param i2c_frequency: I2C bus frequency
        :param i2c_address: IMU I2C address
        """
        # IO control uses io_control module
        # MotorController
        motor_kwargs = {}
        if motor_gpio_chip is not None:
            motor_kwargs['gpio_chip'] = motor_gpio_chip
        if motor_pin1 is not None:
            motor_kwargs['pin1'] = motor_pin1
        if motor_pin2 is not None:
            motor_kwargs['pin2'] = motor_pin2
        if motor_freq_hz is not None:
            motor_kwargs['freq_hz'] = motor_freq_hz
        self.motor = MotorController(**motor_kwargs)
        # IMU controller
        self.imu = BNO08XController(i2c_bus=i2c_bus, scl=scl, sda=sda,
                                   frequency=i2c_frequency, address=i2c_address)

    # --- IO methods via io_control module ---
    def setup_io(self):
        """Initialize IO: switch and LED."""
        io_control.setup()

    def get_button_state(self):
        """Return switch state: True if released/high, False if pressed/low."""
        io_control.setup()
        return io_control.start_button_state()

    def set_led(self, state):
        """Set LED on/off."""
        io_control.setup()
        io_control.set_led(state)

    def cleanup_io(self):
        """Cleanup IO resources."""
        io_control.cleanup()

    # --- Steering methods: assuming steering module still used if needed ---
    def setup_servo(self):
        """Initialize steering PWM via steering module."""
        try:
            import steering
            steering.setup_pwm()
            #steering.set_steering_angle(0)
        except ImportError:
            print("steering module not found; steering unavailable")

    def set_steering(self, normalized):
        """Set steering via normalized value [-1,1]."""
        try:
            import steering
            steering.set_steering_angle(normalized)
        except ImportError:
            raise RuntimeError("steering module not available")

    def cleanup_servo(self):
        """Cleanup steering PWM."""
        try:
            import steering
            steering.cleanup_pwm()
        except ImportError:
            pass

    # --- Motor methods ---
    def setup_motor(self):
        """Initialize motor controller."""
        self.motor.setup()

    def set_speed(self, value):
        """Set motor speed [-1,1]."""
        self.motor.setup()
        self.motor.set_speed(-value)

    def set_coast(self):
        """Set motor to coast."""
        self.motor.setup()
        self.motor.set_coast()

    # --- IMU methods ---
    def setup_imu(self):
        """Initialize IMU controller."""
        self.imu.setup()

    def get_yaw(self):
        """Return absolute yaw [0,360)."""
        self.imu.setup()
        return self.imu.get_yaw()

    def get_continuous_yaw(self):
        """Return continuous yaw since reference reset."""
        self.imu.setup()
        return self.imu.get_continuous_yaw()

    def reset_yaw_reference(self):
        """Reset continuous yaw reference to current heading."""
        self.imu.setup()
        self.imu.reset_continuous_yaw()

    def run_calibration(self):
        """Run IMU calibration routine."""
        self.imu.setup()
        self.imu.run_calibration()

    # --- Combined setup/cleanup ---
    def setup_all(self):
        """Setup IO, steering, motor, and IMU."""
        # IO
        try:
            self.setup_io()
        except Exception as e:
            print(f"Warning: IO setup failed: {e}")
        # Steering
        self.setup_servo()
        # Motor
        self.setup_motor()
        # IMU
        self.setup_imu()

    def cleanup_all(self):
        """Cleanup IO, steering, motor, and IMU."""
        try:
            self.cleanup_io()
        except Exception:
            pass
        try:
            self.cleanup_servo()
        except Exception:
            pass
        try:
            self.motor.cleanup()
        except Exception:
            pass
        try:
            self.imu.cleanup()
        except Exception:
            pass

    def register_signal_handlers(self):
        """Register SIGINT/SIGTERM to cleanup all and exit."""
        def _handler(sig, frame):
            print(f"Signal {sig} received, cleaning up and exiting...")
            try:
                self.cleanup_all()
            except Exception as e:
                print(f"Exception during cleanup: {e}")
            sys.exit(0)
        signal.signal(signal.SIGINT, _handler)
        signal.signal(signal.SIGTERM, _handler)
        print("Signal handlers registered for SIGINT and SIGTERM")


def main():
    """Test all capabilities: IO, steering, motor, IMU."""
    db = DriveBase()
    db.register_signal_handlers()
    try:
        print("Setting up all subsystems...")
        db.setup_all()
        # IO test: blink LED and read button
        
        print("Testing IO: blink LED and read button state...")
        for _ in range(3):
            db.set_led(True)
            time.sleep(0.5)
            db.set_led(False)
            time.sleep(0.5)
        print("Now monitoring button; LED mirrors button state for 10 seconds...")
        start = time.time()
        while time.time() - start < 10:
            btn = db.get_button_state()
            db.set_led(btn)
            print(f"Button is {'released (HIGH)' if btn else 'pressed (LOW)'}; LED {'ON' if btn else 'OFF'}")
            time.sleep(0.5)
        # Steering test
        print("Testing steering: center, left, right, center")
        try:
            db.set_steering(0); time.sleep(1)
            db.set_steering(1); time.sleep(1)
            db.set_steering(-1); time.sleep(1)
            db.set_steering(0); time.sleep(1)
        except RuntimeError as e:
            print(f"Steering test skipped: {e}")
        # Motor test
        print("Testing motor: forward, coast, reverse, coast")
        
        db.set_speed(0.6); time.sleep(2)
        db.set_coast(); time.sleep(1)
        db.set_speed(-0.6); time.sleep(2)
        db.set_coast(); time.sleep(1)
        # IMU test
        print("Testing IMU continuous yaw. Turn the device; printing for 10 seconds...")
        db.reset_yaw_reference()
        start = time.time()
        while time.time() - start < 10:
            yaw_abs = db.get_yaw()
            yaw_cont = db.get_continuous_yaw()
            print(f"Yaw abs: {yaw_abs:.2f}°, cont: {yaw_cont:.2f}°")
            time.sleep(0.5)
        print("Example run complete. Entering idle loop: printing continuous yaw and button state")
        while True:
            yaw_cont = db.get_continuous_yaw()
            btn = db.get_button_state()
            print(f"Continuous Yaw: {yaw_cont:.2f}°, Button: {'HIGH' if btn else 'LOW'}")
            time.sleep(0.5)
    except Exception as e:
        print(f"Exception in main: {e}")
    finally:
        print("Cleaning up all...")
        db.cleanup_all()
        print("Exited cleanly.")

if __name__ == "__main__":
    main()
