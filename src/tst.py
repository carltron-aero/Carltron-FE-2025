import lgpio, time

GPIO_PIN = 12
CHIP = 4
FREQ = 50
MIN_PULSE_US = 500
MAX_PULSE_US = 2500
PERIOD_US = 1_000_000 // FREQ  # 20000 µs

def pulse_to_duty(pulse_us):
    return (pulse_us / PERIOD_US) * 100.0

MIN_DUTY = pulse_to_duty(MIN_PULSE_US)  # ~2.5%
MAX_DUTY = pulse_to_duty(MAX_PULSE_US)  # ~12.5%

h = lgpio.gpiochip_open(CHIP)
lgpio.gpio_claim_output(h, GPIO_PIN)
try:
    # Optional: test mid-point first
    mid = (MIN_DUTY + MAX_DUTY) / 2
    lgpio.tx_pwm(h, GPIO_PIN, FREQ, mid)
    time.sleep(2)

    # Sweep indefinitely
    steps = 50
    test = 0

    while True:
        """
        for i in range(steps + 1):
            duty = MIN_DUTY + (MAX_DUTY - MIN_DUTY) * (i / steps)
            lgpio.tx_pwm(h, GPIO_PIN, FREQ, duty)
            time.sleep(0.02)
        for i in range(steps, -1, -1):
            duty = MIN_DUTY + (MAX_DUTY - MIN_DUTY) * (i / steps)
            lgpio.tx_pwm(h, GPIO_PIN, FREQ, duty)
            time.sleep(0.02)
            """


        if test == 0:

            duty = MIN_DUTY + (MAX_DUTY - MIN_DUTY) * (10 / steps)
            lgpio.tx_pwm(h, GPIO_PIN, FREQ, duty)
            test = 1
        elif test == 1:
            duty = MIN_DUTY + (MAX_DUTY - MIN_DUTY) * (30 / steps)
            lgpio.tx_pwm(h, GPIO_PIN, FREQ, duty)
            test = 0


        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    lgpio.tx_pwm(h, GPIO_PIN, 0, 0)
    lgpio.gpiochip_close(h)
