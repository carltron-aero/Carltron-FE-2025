import steering
import time

steering.set_steering_angle(0.5)
time.sleep(1)
steering.set_steering_angle(-0.5)
time.sleep(1)
steering.set_steering_angle(0)
time.sleep(1)
steering.cleanup_pwm()