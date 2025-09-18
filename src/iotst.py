#!/usr/bin/env python3
"""
Simple switch monitor on gpiochip 4, BCM pin 16, using internal pull-up via lgpio.
Continuously prints switch state whenever it changes.
"""
import lgpio
import time
import signal
import sys

# Configuration
GPIO_CHIP = 4    # /dev/gpiochip4
SWITCH_PIN = 16  # BCM pin for switch input

_h = None

def setup():
    """
    Open gpiochip and claim SWITCH_PIN as input with internal pull-up.
    """
    global _h
    if _h is not None:
        return
    try:
        _h = lgpio.gpiochip_open(GPIO_CHIP)
    except Exception as e:
        _h = None
        raise RuntimeError(f"Failed to open GPIO chip {GPIO_CHIP}: {e}")
    # Determine pull-up flag: lgpio may expose LGPIO_PULL_UP; otherwise use bit 5 (value 1<<5)
    try:
        pull_up_flag = lgpio.LGPIO_PULL_UP
    except AttributeError:
        pull_up_flag = 1 << 5
    try:
        # Claim input with pull-up bias
        lgpio.gpio_claim_input(_h, SWITCH_PIN, lFlags=pull_up_flag)
    except Exception as e:
        cleanup()
        raise RuntimeError(f"Failed to claim switch pin {SWITCH_PIN} as input with pull-up: {e}")
    print(f"GPIO setup done: switch on gpiochip {GPIO_CHIP}, pin {SWITCH_PIN} as input with internal pull-up")

def read_switch():
    """
    Read and return the switch state: True if HIGH (released), False if LOW (pressed to GND).
    """
    if _h is None:
        raise RuntimeError("GPIO not initialized; call setup() first")
    try:
        val = lgpio.gpio_read(_h, SWITCH_PIN)
    except Exception as e:
        raise RuntimeError(f"Failed to read switch pin {SWITCH_PIN}: {e}")
    return bool(val)

def cleanup():
    """
    Close gpiochip handle.
    """
    global _h
    if _h is None:
        return
    try:
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
    # Register signal handlers for clean exit
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)
    try:
        setup()
        print("Monitoring switch state (with internal pull-up). Press Ctrl-C to exit.")
        last = None
        while True:
            state = read_switch()
            if state != last:
                # With pull-up: HIGH means open (released), LOW means pressed (to GND)
                if state:
                    print("Switch is HIGH (released)")
                else:
                    print("Switch is LOW (pressed)")
                last = state
            time.sleep(0.1)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cleanup()
        print("Exited cleanly.")

if __name__ == "__main__":
    main()
