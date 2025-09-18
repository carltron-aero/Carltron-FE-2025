#!/usr/bin/env python3
from rpi_hardware_pwm import HardwarePWM
from time import sleep

# ===== CONFIGURE THESE BASED ON YOUR SETUP =====
PWM_CHIP = 0        # the pwmchip index where hardware PWM appears (e.g. pwmchip2)
PWM_CHANNEL = 0     # channel number on that chip (e.g. 0 for the first channel)
FREQ = 1000         # PWM frequency in Hz for LED (e.g. 1kHz); choose a value your hardware supports


def main():
    # Instantiate hardware PWM
    try:
        pwm = HardwarePWM(pwm_channel=PWM_CHANNEL, hz=FREQ, chip=PWM_CHIP)
    except Exception as e:
        print(f"Error creating HardwarePWM: {e}")
        return

    try:
        print(f"Starting LED fade on pwmchip{PWM_CHIP} channel {PWM_CHANNEL} at {FREQ}Hz")
        # Fade loop: ramp duty 0→100% and back
        steps = 100
        delay = 0.02  # seconds per step; adjust for speed
        while True:
            # Fade in
            for i in range(steps + 1):
                duty = i * 100.0 / steps  # 0.0 to 100.0
                print(duty)
                pwm.start(duty) if i == 0 else pwm.change_duty_cycle(duty)
                sleep(delay)
            # Fade out
            for i in range(steps, -1, -1):
                duty = i * 100.0 / steps
                print(duty)
                pwm.change_duty_cycle(duty)
                sleep(delay)
    except KeyboardInterrupt:
        print("Interrupted, stopping PWM")
    except Exception as e:
        print(f"Runtime error: {e}")
    finally:
        try:
            pwm.stop()
        except:
            pass
        print("PWM stopped; exiting")

if __name__ == "__main__":
    main()
