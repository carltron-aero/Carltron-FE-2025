import os
import time

# === CONFIGURATION ===
PWM_CHIP = 0           # pwmchip0
PWM_CHANNEL = 1        # pwm0
FREQ_HZ = 50           # 50 Hz standard for RC servos
PERIOD_US = 1_000_000 // FREQ_HZ  # 20_000 µs
PERIOD_NS = PERIOD_US * 1000      # convert to nanoseconds

# Servo pulse range (in microseconds)
MIN_PULSE_US = 500
MAX_PULSE_US = 2500
MIN_PULSE_NS = MIN_PULSE_US * 1000
MAX_PULSE_NS = MAX_PULSE_US * 1000

# Sweep settings
STEP_DEG = 5
DELAY_SEC = 0.03

# === Paths ===
BASE_PATH = f"/sys/class/pwm/pwmchip{PWM_CHIP}"
PWM_PATH = f"{BASE_PATH}/pwm{PWM_CHANNEL}"

def write(path, value):
    with open(path, "w") as f:
        f.write(str(value))

def angle_to_duty_ns(angle):
    angle = max(0, min(180, angle))
    return MIN_PULSE_NS + (MAX_PULSE_NS - MIN_PULSE_NS) * angle // 180

def setup_pwm():
    if not os.path.exists(PWM_PATH):
        write(f"{BASE_PATH}/export", PWM_CHANNEL)
        time.sleep(0.1)
    write(f"{PWM_PATH}/period", PERIOD_NS)
    write(f"{PWM_PATH}/duty_cycle", MIN_PULSE_NS)
    write(f"{PWM_PATH}/enable", 1)

def cleanup_pwm():
    write(f"{PWM_PATH}/enable", 0)



def sweep_servo():
    test = 0
    while True:
        """
        for angle in range(0, 181, STEP_DEG):
            duty_ns = angle_to_duty_ns(angle)
            write(f"{PWM_PATH}/duty_cycle", duty_ns)
            time.sleep(DELAY_SEC)
        for angle in range(180, -1, -STEP_DEG):
            duty_ns = angle_to_duty_ns(angle)
            write(f"{PWM_PATH}/duty_cycle", duty_ns)
            time.sleep(DELAY_SEC)
        """

        test = 3

        if test == 0:
            duty_ns = angle_to_duty_ns(50)
            write(f"{PWM_PATH}/duty_cycle", duty_ns)
            test = 1
        elif test == 1:
            duty_ns = angle_to_duty_ns(130)
            write(f"{PWM_PATH}/duty_cycle", duty_ns)
            test = 0

        elif test == 3:
            duty_ns = angle_to_duty_ns(90)
            write(f"{PWM_PATH}/duty_cycle", duty_ns)
            test = 0
        print(duty_ns)

        time.sleep(1.2)

if __name__ == "__main__":
    try:
        print(f"Starting servo sweep on pwmchip{PWM_CHIP} pwm{PWM_CHANNEL}")
        setup_pwm()
        sweep_servo()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        cleanup_pwm()
        print("PWM disabled.")
