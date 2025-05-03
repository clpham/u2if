# File: source/machine/freqcounter.py
from .u2if import Device
from . import u2if_const as report_const
from .pin import Pin  # Import Pin if needed for type hinting or validation
import time

# Cache system clock frequency to avoid repeated requests
_sys_clk_hz = None
_sys_clk_last_checked = 0


class FreqCounter:
    """
    Measures frequency and duty cycle of a digital signal using RP2040 PIO.
    """

    def __init__(self, pin, serial_number_str=None):
        """
        Initialize frequency counter on the specified pin.

        Args:
            pin (int | Pin): The GPIO pin number (e.g., u2if.GP15) or a Pin object.
            serial_number_str (str, optional): Serial number of the target u2if device. Defaults to None.
        """
        if isinstance(pin, Pin):
            self.pin_id = pin.id
        elif isinstance(pin, int):
            self.pin_id = pin
        else:
            raise ValueError("pin must be an integer pin ID or a Pin object")

        self._device = Device(serial_number_str=serial_number_str)
        self._initialized = False
        self._init()
        self._initialized = True  # Assume success if no exception

        # Get system clock frequency (cached)
        global _sys_clk_hz, _sys_clk_last_checked
        current_time = time.monotonic()
        # Re-check clock every 60 seconds or if never checked
        if _sys_clk_hz is None or (current_time - _sys_clk_last_checked > 60):
            # Add a command to get sys clock if it doesn't exist
            # For now, let's assume 125MHz as a common default for Pico
            # Or retrieve it if SYS_GET_VN returns it, or add SYS_GET_CLK
            # _sys_clk_hz = self._device.get_sys_clock_hz() # Placeholder
            _sys_clk_hz = 125_000_000  # Assume 125 MHz for now
            _sys_clk_last_checked = current_time
        self._sys_clk = _sys_clk_hz

    def _init(self):
        """Sends the initialization command to the firmware."""
        res = self._device.send_report(
            bytes([report_const.FREQ_COUNTER_INIT, self.pin_id])
        )
        if res[1] != report_const.OK:
            err_code = res[3] if len(res) > 3 else 0
            if err_code == 0x01:
                raise RuntimeError(
                    f"FreqCounter init failed on pin {self.pin_id}: No PIO resources available."
                )
            elif err_code == 0x02:
                raise RuntimeError(
                    f"FreqCounter init failed on pin {self.pin_id}: Pin already in use."
                )
            else:
                raise RuntimeError(
                    f"FreqCounter init failed on pin {self.pin_id} with unknown error code {err_code}."
                )
        print(f"FreqCounter initialized on pin {self.pin_id}")

        # clpham:
        self._sys_clk = int.from_bytes(res[3:7], byteorder='little')

    def deinit(self):
        """Deinitialize the frequency counter and release resources."""
        if not self._initialized:
            return
        try:
            res = self._device.send_report(
                bytes([report_const.FREQ_COUNTER_DEINIT, self.pin_id])
            )
            if res[1] != report_const.OK:
                # Log warning or ignore? Deinit failure might not be critical.
                print(f"Warning: FreqCounter deinit failed on pin {self.pin_id}.")
            else:
                print(f"FreqCounter deinitialized on pin {self.pin_id}")
        except Exception as e:
            print(
                f"Warning: Exception during FreqCounter deinit on pin {self.pin_id}: {e}"
            )
        finally:
            self._initialized = False

    def __del__(self):
        """Ensure deinitialization when the object is garbage collected."""
        self.deinit()

    def measure(self):
        """
        Performs a measurement and returns the frequency and duty cycle.

        Returns:
            tuple[float, float]: A tuple containing:
                - frequency (float): Frequency in Hertz.
                - duty_cycle (float): Duty cycle in percent (0.0 to 100.0).
             Returns (0.0, 0.0) if the measurement fails or the period is zero.
        """
        if not self._initialized:
            raise RuntimeError("FreqCounter is not initialized.")
        if self._sys_clk is None or self._sys_clk == 0:
            # This should ideally not happen if init succeeded and clock was fetched
            print("Warning: System clock frequency unknown or zero, cannot calculate.")
            return (0.0, 0.0)

        res = self._device.send_report(
            bytes([report_const.FREQ_COUNTER_GET_MEASUREMENT, self.pin_id])
        )

        if res[1] != report_const.OK:
            raise RuntimeError(f"FreqCounter measurement failed on pin {self.pin_id}.")

        if len(res) < 11:  # 1 (ID) + 1 (Status) + 1 (Pin) + 4 (High) + 4 (Low)
            raise RuntimeError(
                f"FreqCounter received incomplete measurement data on pin {self.pin_id}."
            )

        high_cycles = int.from_bytes(res[3:7], byteorder='little')
        low_cycles = int.from_bytes(res[7:11], byteorder='little')

        total_cycles = high_cycles + low_cycles

        if total_cycles == 0:
            # Avoid division by zero - indicates no full cycle measured or error
            frequency_hz = 0.0
            duty_cycle_percent = 0.0  # Or perhaps NaN?
        else:
            # Calculate frequency in Hz
            frequency_hz = self._sys_clk / total_cycles

            # Calculate duty cycle in percent
            duty_cycle_percent = (high_cycles * 100.0) / total_cycles

        return frequency_hz, duty_cycle_percent
