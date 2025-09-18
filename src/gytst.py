#!/usr/bin/env python3
import time
import signal
import sys
from imu_control import BNO08XController

def main():
    imu = BNO08XController()
    # Register signal handlers so Ctrl-C or termination cleans up nicely
    imu.register_signal_handlers()
    try:
        imu.setup()
        # Initialize continuous yaw reference to current heading
        imu.reset_continuous_yaw()
        print("Printing continuous yaw. Ctrl-C to exit.")
        while True:
            yaw_cont = imu.get_continuous_yaw()
            print(f"Continuous Yaw: {yaw_cont:.2f}°")
            time.sleep(0.1)
    except Exception as e:
        print(f"Exception: {e}")
    finally:
        imu.cleanup()
        print("Exited cleanly.")

if __name__ == "__main__":
    main()
