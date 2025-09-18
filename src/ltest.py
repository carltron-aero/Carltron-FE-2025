import time
import lgpio

PIN = 23  # BCM pin number

# 1. Open the GPIO chip
h = lgpio.gpiochip_open(4)

# 2. Claim the pin as output
lgpio.gpio_claim_output(h, PIN)

try:
    # 3. Drive pin high
    lgpio.gpio_write(h, PIN, 1)
    print(f"GPIO {PIN} set HIGH")
    # Keep it high (or insert other logic here)
    time.sleep(10)  # e.g., hold for 10 seconds
finally:
    # 4. Optionally drive low and close
    lgpio.gpio_write(h, PIN, 0)
    lgpio.gpiochip_close(h)
    print(f"GPIO {PIN} released and set LOW")
