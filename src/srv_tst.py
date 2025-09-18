from gpiozero import AngularServo
from time import sleep

# Replace 17 with your GPIO pin
# Use pulse widths 0.0005–0.0025 seconds (500–2500 µs) and 50 Hz frame (default 20 ms)
servo = AngularServo(
    12,
    min_angle=0,
    max_angle=180,
    min_pulse_width=0.0005,
    max_pulse_width=0.0025,
    frame_width=0.02
)

test = 0

try:
    while True:
        """
        # Sweep from 0° to 180°
        for angle in range(0, 181, 2):
            servo.angle = angle
            sleep(0.05)
        # Sweep back from 180° to 0°
        for angle in range(180, -1, -2):
            servo.angle = angle
            sleep(0.05)
        # Pause before repeating

        sleep(3)

        """

        if test == 0:

            servo.angle = 90
            sleep(0.1)
            test = 0
        elif test == 1:
            servo.angle = 90
            sleep(2)
            test = 0
            

except KeyboardInterrupt:
    pass
finally:
    servo.close()
