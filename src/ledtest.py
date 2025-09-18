import time
import os
import lgpio

PIN = 23  # BCM pin number
PIN2 = 24

# 1. Open the GPIO chip
h = lgpio.gpiochip_open(4)

# 2. Claim the pin as output
lgpio.gpio_claim_output(h, PIN)
lgpio.gpio_claim_output(h, PIN2)
lgpio.gpio_write(h, PIN, 0)
lgpio.gpio_write(h, PIN2, 0)
time.sleep(2)

"""
PWM_CHIP = 0
PWM_CHANNEL = 2
BASE_PATH = f"/sys/class/pwm/pwmchip{PWM_CHIP}"
CHANNEL_PATH = f"{BASE_PATH}/pwm{PWM_CHANNEL}"

def write(path, value):
    with open(path, 'w') as f:
        f.write(str(value))

def setup_pwm():
    if not os.path.exists(CHANNEL_PATH):
        write(f"{BASE_PATH}/export", PWM_CHANNEL)
        time.sleep(0.1)
    write(f"{CHANNEL_PATH}/period", 20000000)
    write(f"{CHANNEL_PATH}/duty_cycle", 0)
    write(f"{CHANNEL_PATH}/enable", 1)

def fade_led():
    steps = 100
    while True:
        for i in range(steps + 1):
            duty = int(20000000 * i / steps)
            write(f"{CHANNEL_PATH}/duty_cycle", duty)
            time.sleep(0.02)
            print(duty)
        for i in range(steps, -1, -1):
            duty = int(20000000 * i / steps)
            write(f"{CHANNEL_PATH}/duty_cycle", duty)
            time.sleep(0.02)
            print(duty)

        

def cleanup():
    write(f"{CHANNEL_PATH}/enable", 0)

try:
    setup_pwm()
    fade_led()
except KeyboardInterrupt:
    print("Stopping")
    cleanup()
"""