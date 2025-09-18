#!/usr/bin/env python3
"""
Module: switch_led.py

Provides:
 - setup(): internal, claims GPIO chip, configures switch input (BCM 16) with internal pull-up, and LED output (BCM 26).
 - start_button_state(): return True if switch reads HIGH (released), False if LOW (pressed).
 - set_led(state): turn LED on/off on BCM 26.
 - cleanup(): release resources.

When run as script, self-tests by blinking LED and then in a loop: mirror switch state to LED and print state.
"""
import lgpio
import time
import signal
import sys

# Configuration
GPIO_CHIP = 4    # use /dev/gpiochip4
SWITCH_PIN = 16  # BCM pin for switch input
LED_PIN = 26     # BCM pin for LED output

# Internal handle
_h = None

def setup():
    """
    Open gpiochip and claim SWITCH_PIN as input with internal pull-up, LED_PIN as output (initially off).
    """
    global _h
    if _h is not None:
        return
    try:
        _h = lgpio.gpiochip_open(GPIO_CHIP)
    except Exception as e:
        _h = None
        raise RuntimeError(f"Failed to open GPIO chip {GPIO_CHIP}: {e}")
    # Claim switch pin as input with pull-up
    try:
        # Determine pull-up flag: lgpio may expose LGPIO_PULL_UP; otherwise use bit 5 = 1<<5
        try:
            pull_up_flag = lgpio.LGPIO_PULL_UP
        except AttributeError:
            pull_up_flag = 1 << 5
        lgpio.gpio_claim_input(_h, SWITCH_PIN, lFlags=pull_up_flag)
    except Exception as e:
        cleanup()
        raise RuntimeError(f"Failed to claim switch pin {SWITCH_PIN} as input with pull-up: {e}")
    # Claim LED pin as output, initialize LOW
    try:
        lgpio.gpio_claim_output(_h, LED_PIN)
        lgpio.gpio_write(_h, LED_PIN, 0)
    except Exception as e:
        cleanup()
        raise RuntimeError(f"Failed to claim LED pin {LED_PIN} as output: {e}")
    print(f"GPIO setup done: switch on gpiochip {GPIO_CHIP} pin {SWITCH_PIN} (input w/ pull-up), LED on pin {LED_PIN} (output)")

def start_button_state():
    """
    Return switch state: True if HIGH (released), False if LOW (pressed to GND).
    Requires setup() called first.
    """
    if _h is None:
        raise RuntimeError("GPIO not initialized; call setup() first")
    try:
        val = lgpio.gpio_read(_h, SWITCH_PIN)
    except Exception as e:
        raise RuntimeError(f"Failed to read switch pin {SWITCH_PIN}: {e}")
    return bool(val)

def set_led(state):
    """
    Turn LED on/off.
    :param state: True to turn on (write 1), False to turn off (write 0).
    Requires setup() called first.
    """
    if _h is None:
        raise RuntimeError("GPIO not initialized; call setup() first")
    v = 1 if state else 0
    try:
        lgpio.gpio_write(_h, LED_PIN, v)
    except Exception as e:
        raise RuntimeError(f"Failed to write LED pin {LED_PIN}: {e}")

def cleanup():
    """
    Turn LED off and close gpiochip handle.
    """
    global _h
    if _h is None:
        return
    try:
        # Turn LED off
        try:
            lgpio.gpio_write(_h, LED_PIN, 0)
        except Exception:
            pass
        lgpio.gpiochip_close(_h)
    except Exception as e:
        print(f"Warning during cleanup: {e}")
    finally:
        _h = None
    print("GPIO cleaned up")

def _signal_handler(sig, frame):
    print("\nSignal received, cleaning up and exiting...")
    cleanup()
    sys.exit(0)

def main():
    """
    Self-test when run as script:
     - Blink LED 3 times.
     - Then in a loop: read switch, print its state, and mirror it to LED (LED on when switch released/high, off when pressed/low).
    """
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)
    try:
        setup()
        print("Self-test: blinking LED 3 times")
        for _ in range(3):
            set_led(True)
            time.sleep(0.5)
            set_led(False)
            time.sleep(0.5)
        print("Now monitoring switch; LED will mirror switch (ON when released/high, OFF when pressed/low). Ctrl-C to exit.")
        last = None
        while True:
            state = start_button_state()
            if state != last:
                print(f"Switch is {'HIGH (released)' if state else 'LOW (pressed)'}; setting LED {'ON' if state else 'OFF'}")
                last = state
            # Mirror LED to switch state
            set_led(state)
            time.sleep(0.1)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cleanup()
        print("Exited cleanly.")

if __name__ == "__main__":
    main()
