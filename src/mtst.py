from motor_control import MotorController
import time

mc = MotorController()
mc.setup()
mc.set_speed(0.8)

time.sleep(2)

mc.set_speed(-0.4)

time.sleep(3)
mc.cleanup()