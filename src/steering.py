import os
import time

# === HW PWM CONFIGURATION FOR SERVO ===
PWM_CHIP = 0           # pwmchip0
PWM_CHANNEL = 2        # pwm0
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
        print("written")

def angle_to_duty_ns(angle):
    angle = max(0, min(180, angle))
    return int(MIN_PULSE_NS + (MAX_PULSE_NS - MIN_PULSE_NS) * angle // 180)

def setup_pwm():
    if not os.path.exists(PWM_PATH):
        write(f"{BASE_PATH}/export", PWM_CHANNEL)
        time.sleep(0.1)
    write(f"{PWM_PATH}/period", PERIOD_NS)
    write(f"{PWM_PATH}/duty_cycle", 1500000)
    write(f"{PWM_PATH}/enable", 1)

def cleanup_pwm():
    write(f"{PWM_PATH}/enable", 0)

def set_steering_angle(normalized):
    
    #normalized: float in [-1, 1]
    #Maps to steering angle in [50, 130], then applies PWM.
    
    setup_pwm()
    # Clamp input to [-1, 1]
    x = max(-1.0, min(1.0, normalized))
    # Map [-1,1] → [50,130]: angle = 40*x + 90
    angle = 40.0 * x + 90.0
    # Just in case, clamp to [50,130]
    angle = min(max(50, angle), 130)
    print(angle)
    duty_ns = angle_to_duty_ns(angle)
    print(duty_ns)
    write(f"{PWM_PATH}/duty_cycle", duty_ns)

"""

def set_steering_angle(angle):
    setup_pwm()
    angle = min(max(50,angle), 130)
    duty_ns = angle_to_duty_ns(angle)
    print(duty_ns)
    write(f"{PWM_PATH}/duty_cycle", duty_ns)

"""

if __name__ == "__main__":
    try:
        print(f"Starting servo sweep on pwmchip{PWM_CHIP} pwm{PWM_CHANNEL}")
        setup_pwm()
        while True:
            for x in [-1.0, -0.5, 0.0, 0.5, 1.0]:
                set_steering_angle(x)
                time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        cleanup_pwm()
        print("PWM disabled.")
