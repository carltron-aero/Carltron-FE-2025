import os
import time

# === HW PWM CONFIGURATION FOR SERVO ===
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

# === HW PWM CODE FOR SERVO ===
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

def set_steering_angle(angle):
    setup_pwm()
    angle = min(max(50,angle), 130)
    duty_ns = angle_to_duty_ns(angle)
    write(f"{PWM_PATH}/duty_cycle", duty_ns)

"""
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
"""