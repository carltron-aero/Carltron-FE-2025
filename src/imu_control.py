#!/usr/bin/env python3
"""
Module: imu_control.py

Provides BNO08XController class to manage Adafruit BNO08X IMU sensor over I2C,
including initialization, optional calibration, and yaw (heading) retrieval.
Also provides continuous yaw tracking from initial position.

Usage:
    from imu_control import BNO08XController
    imu = BNO08XController()
    imu.setup()
    # Optionally calibrate:
    imu.run_calibration()
    yaw_abs = imu.get_yaw()  # absolute heading [0,360)
    yaw_cont = imu.get_continuous_yaw()  # continuous yaw from start
    imu.cleanup()

Example as script:
    python imu_control.py
"""
import time
import math
import board
import busio
import signal
import sys
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

# Default I2C parameters
DEFAULT_I2C_FREQUENCY = 400000
DEFAULT_I2C_ADDRESS = 0x4A

class BNO08XController:
    def __init__(self, i2c_bus=None, scl=None, sda=None, frequency=DEFAULT_I2C_FREQUENCY, address=DEFAULT_I2C_ADDRESS):
        """
        Initialize controller parameters.
        :param i2c_bus: existing busio.I2C instance, or None to create new from board.SCL/SDA
        :param scl: board.SCL pin (used if i2c_bus is None)
        :param sda: board.SDA pin (used if i2c_bus is None)
        :param frequency: I2C frequency in Hz
        :param address: I2C address of BNO08X (default 0x4A)
        """
        self.i2c_bus = i2c_bus
        self.scl = scl
        self.sda = sda
        self.frequency = frequency
        self.address = address
        self.bno = None
        # For continuous yaw tracking
        self._last_yaw = None
        self._yaw_offset = 0.0

    def setup(self):
        """
        Initialize I2C bus and BNO08X sensor, enable required features.
        Must be called before get_yaw or run_calibration or get_continuous_yaw.
        """
        if self.bno is not None:
            return
        # Initialize I2C if not provided
        if self.i2c_bus is None:
            if self.scl is None or self.sda is None:
                # Default to board pins
                self.scl = board.SCL
                self.sda = board.SDA
            try:
                self.i2c_bus = busio.I2C(self.scl, self.sda, frequency=self.frequency)
            except Exception as e:
                raise RuntimeError(f"Failed to initialize I2C bus: {e}")
            # Wait for I2C lock briefly
            t_start = time.monotonic()
            while not self.i2c_bus.try_lock():
                if time.monotonic() - t_start > 5:
                    raise RuntimeError("Timeout waiting for I2C lock")
                time.sleep(0.1)
            self.i2c_bus.unlock()
        # Initialize BNO08X
        try:
            self.bno = BNO08X_I2C(self.i2c_bus, address=self.address)
            # Enable features: accelerometer, gyro, magnetometer, rotation vector
            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            print(f"BNO08X initialized at I2C address 0x{self.address:02X}")
            # Reset continuous yaw tracking
            self._last_yaw = None
            self._yaw_offset = 0.0
        except Exception as e:
            self.bno = None
            raise RuntimeError(f"Failed to initialize BNO08X sensor: {e}")

    def quaternion_to_yaw_deg(self, w, x, y, z):
        """
        Convert quaternion (w, x, y, z) to yaw angle in degrees [0, 360).
        """
        t0 = +2.0 * (w * z + x * y)
        t1 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t0, t1)
        yaw_deg = math.degrees(yaw)
        if yaw_deg < 0:
            yaw_deg += 360.0
        return yaw_deg

    def get_quaternion(self):
        """
        Read quaternion from sensor. Returns (w, x, y, z).
        Must have called setup().
        """
        if self.bno is None:
            raise RuntimeError("Sensor not initialized; call setup() first")
        # bno.quaternion returns (i, j, k, real) => (x, y, z, w)
        quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
        return (quat_real, quat_i, quat_j, quat_k)

    def get_yaw(self):
        """
        Return current yaw (heading) in degrees [0, 360).
        Absolute heading from sensor reference.
        Must have called setup().
        """
        w, x, y, z = self.get_quaternion()
        return self.quaternion_to_yaw_deg(w, x, y, z)

    def get_continuous_yaw(self):
        """
        Return continuous yaw angle in degrees, starting at 0 on first call after setup(),
        and accumulating changes across wrap-around. Positive for increasing yaw, negative for reverse.
        Must have called setup().
        """
        current_yaw = self.get_yaw()
        if self._last_yaw is None:
            # First reading: initialize reference
            self._last_yaw = current_yaw
            self._yaw_offset = 0.0
            return 0.0
        # Compute delta, handling wrap-around at 0/360
        delta = current_yaw - self._last_yaw
        # Adjust for wrap-around: if delta > 180, assume crossed from 359->0: subtract 360
        if delta > 180.0:
            delta -= 360.0
        # If delta < -180, assume crossed from 0->359: add 360
        elif delta < -180.0:
            delta += 360.0
        # Accumulate
        self._yaw_offset += delta
        self._last_yaw = current_yaw
        return self._yaw_offset

    def reset_continuous_yaw(self):
        """
        Reset continuous yaw reference: next get_continuous_yaw() will treat current heading as zero.
        """
        if self.bno is None:
            raise RuntimeError("Sensor not initialized; call setup() first")
        current_yaw = self.get_yaw()
        self._last_yaw = current_yaw
        self._yaw_offset = 0.0
        print("Continuous yaw reference reset to current heading {:.2f}°".format(current_yaw))

    def run_calibration(self, poll_interval=1.0):
        """
        Guide user through calibration until statuses reach 3.
        Loops, printing status every poll_interval seconds.
        """
        if self.bno is None:
            raise RuntimeError("Sensor not initialized; call setup() first")
        print("Starting calibration routine. Please move the sensor as follows:")
        print(" - Rotate around each axis slowly and perform 'figure-8' motions for magnetometer calibration.")
        print(" - Continue until all calibration statuses are 3.")
        print("Calibration status tuple is (sys, gyro, accel, mag), each 0..3.")
        try:
            while True:
                status = self.bno.calibration_status  # tuple: (sys, gyro, accel, mag)
                sys_stat, gyro_stat, accel_stat, mag_stat = status
                print(
                    f"Calibration status -> System: {sys_stat}/3, Gyro: {gyro_stat}/3, "
                    f"Accel: {accel_stat}/3, Mag: {mag_stat}/3"
                )
                if sys_stat == 3 and gyro_stat == 3 and accel_stat == 3 and mag_stat == 3:
                    print("Calibration complete!")
                    break
                time.sleep(poll_interval)
        except KeyboardInterrupt:
            print("\nCalibration interrupted by user. Proceeding with current calibration state.")

    def cleanup(self):
        """
        Placeholder for cleanup if needed. Currently does nothing.
        """
        # BNO08X_I2C has no explicit close; if using I2C bus persistently, consider deinit
        # For busio.I2C, no deinit method; rely on Python exit.
        pass

    def register_signal_handlers(self):
        """
        Register SIGINT and SIGTERM to perform cleanup and exit.
        """
        def _handler(sig, frame):
            print(f"Signal {sig} received, exiting...")
            try:
                self.cleanup()
            except Exception as e:
                print(f"Exception during cleanup: {e}")
            sys.exit(0)
        signal.signal(signal.SIGINT, _handler)
        signal.signal(signal.SIGTERM, _handler)
        print("Signal handlers registered for SIGINT and SIGTERM")


def main():
    # Example usage when running as script
    imu = BNO08XController()
    imu.register_signal_handlers()
    try:
        imu.setup()
        # Optionally calibrate
        # imu.run_calibration()
        print("\nEntering main loop. Press Ctrl-C to exit.")
        # Demonstrate continuous yaw
        imu.reset_continuous_yaw()
        while True:
            yaw_abs = imu.get_yaw()
            yaw_cont = imu.get_continuous_yaw()
            print(f"Yaw absolute: {yaw_abs:.2f}°, continuous: {yaw_cont:.2f}°")
            time.sleep(0.1)
    except Exception as e:
        print(f"Exception in main: {e}")
    finally:
        imu.cleanup()
        print("Exited cleanly.")

if __name__ == "__main__":
    main()
