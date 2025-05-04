# File: examples/freq_dutycycle.py
import time
import machine
from machine import u2if, Pin, PWM, FreqCounter, Signal

# --- Configuration ---
PWM_PIN_ID = u2if.GP16  # Choose a PWM-capable pin
FREQ_HZ = 45  # Hz
DUTY_PERCENT = 50.0  # % duty cycle
# ---------------------


def blink_led(signal: Signal, on_time: float = 0.01) -> None:
    signal.on()
    time.sleep(on_time)
    signal.off()


v0, v1, v2 = machine.firmware_version()
fwver = f"{v0}.{v1}.{v2}"
print(f"u2if Firmware version: {fwver}")
print("u2if Frequency Counter Test")

# Setup PWM output
print(f"Setting up PWM on pin {PWM_PIN_ID} at {FREQ_HZ} Hz, {DUTY_PERCENT}% duty cycle")
pwm_pin = Pin(PWM_PIN_ID)
pwm = PWM(pwm_pin)
pwm.freq(FREQ_HZ)
# Convert duty cycle percentage to 16-bit value (0-65535)
duty_u16 = int((DUTY_PERCENT / 100.0) * 65535)
pwm.duty_u16(duty_u16)
print("PWM setup complete.")
time.sleep(0.5)  # Allow PWM to stabilize

# # # Setup Frequency Counter
# # print(f"Initializing Frequency Counter on pin {PWM_PIN_ID}")
FREQ_COUNTER_PIN = u2if.GP15
LED_PIN = u2if.GP25
print(f"Initializing Frequency Counter on pin {FREQ_COUNTER_PIN}")
try:
    led = Signal(Pin(LED_PIN, Pin.OUT), invert=False)
    fc = FreqCounter(FREQ_COUNTER_PIN)
    print("Frequency Counter initialized.")

    start_time = time.perf_counter()
    print("\nStarting measurements (Ctrl+C to stop):")
    while True:
        end_time = time.perf_counter()
        elapsed_time = end_time - start_time
        try:
            freq, duty = fc.measure()
            blink_led(led)
            print(
                f"Measured: Freq = {freq:.2f} Hz, Duty = {duty:.2f} %, Elapsed time = {int(elapsed_time):,}s"
            )
            time.sleep(1.0)  # Measure every second
        except RuntimeError as e:
            print(f"Measurement error: {e}")
            break
        except KeyboardInterrupt:
            print("\nStopping measurements.")
            break

except RuntimeError as e:
    print(f"Error initializing FreqCounter: {e}")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
finally:
    # Clean up
    print("Deinitializing PWM and FreqCounter...")
    # Explicitly deinit FreqCounter if it was created
    if 'fc' in locals() and fc._initialized:
        fc.deinit()
    # Deinit PWM (happens in __del__ too, but explicit is good)
    if 'pwm' in locals():
        pwm.deinit()
    print("Cleanup complete.")
