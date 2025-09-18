#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense

import time
import math
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

def quaternion_to_yaw_deg(w, x, y, z):
    """
    Convert quaternion (w, x, y, z) to yaw angle in degrees [0, 360).
    Quaternion components correspond to real part w and vector part (x, y, z).
    """
    # yaw (around Z axis) formula
    t0 = +2.0 * (w * z + x * y)
    t1 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t0, t1)
    yaw_deg = math.degrees(yaw)
    if yaw_deg < 0:
        yaw_deg += 360.0
    return yaw_deg
    """

def run_calibration(bno):
    
    #Guide the user through calibration. Loops until sys, gyro, accel, mag all reach status 3.
    #bno: initialized BNO08X_I2C instance with features enabled.
    
    print("Starting calibration routine. Please move the sensor as follows:")
    print(" - Rotate around each axis slowly and perform 'figure-8' motions for magnetometer calibration.")
    print(" - Continue until all calibration statuses are 3.")
    print("Calibration status tuple is (sys, gyro, accel, mag), each 0..3.")
    try:
        while True:
            status = bno.calibration_status  # tuple: (sys, gyro, accel, mag)
            sys_stat, gyro_stat, accel_stat, mag_stat = status
            print(
                f"Calibration status -> System: {sys_stat}/3, Gyro: {gyro_stat}/3, "
                f"Accel: {accel_stat}/3, Mag: {mag_stat}/3"
            )
            if sys_stat == 3 and gyro_stat == 3 and accel_stat == 3 and mag_stat == 3:
                print("Calibration complete!")
                break
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nCalibration interrupted by user. Proceeding with current calibration state.")
        """

def main():
    # Initialize I2C. Frequency 400kHz is often fine; adjust if needed.
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    # Create BNO08X instance. Default I2C address is 0x4A; adjust if your module uses a different address.
    bno = BNO08X_I2C(i2c, address=0x4A)

    # Enable features needed for calibration and yaw measurement
    bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)
    bno.enable_feature(BNO_REPORT_MAGNETOMETER)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

    # Run calibration before reading yaw
    #run_calibration(bno)

    print("\nEntering main loop. Press Ctrl-C to exit.")
    try:
        while True:
            # Read quaternion: returns (i, j, k, real) corresponding to (x, y, z, w)
            quat_i, quat_j, quat_k, quat_real = bno.quaternion
            yaw = quaternion_to_yaw_deg(quat_real, quat_i, quat_j, quat_k)
            print(f"Yaw (heading): {yaw:.2f}°")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExited by user.")

if __name__ == "__main__":
    main()
