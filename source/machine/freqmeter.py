from .u2if import Device
from . import u2if_const as report_const
from .pin import Pin  # Import Pin if type hinting or validation is desired


class FreqMeter:
    """
    Measures frequency on a GPIO pin using the RP2040's PWM input and DMA edge timing.

    Note: Each measurement instance consumes a PWM slice and a DMA channel on the RP2040.
          Resource limits apply (typically 8 PWM slices, 12 DMA channels).
    """

    def __init__(self, pin_id, serial_number_str=None):
        """
        Initialize frequency measurement on the specified pin.

        Args:
            pin_id (int): The GPIO pin number (e.g., u2if.GP7).
            serial_number_str (str, optional): Specific device serial number. Defaults to None.
        """
        if not isinstance(pin_id, int):
            raise ValueError("pin_id must be an integer GPIO number")
        self.pin_id = pin_id
        self._device = Device(serial_number_str=serial_number_str)
        self._initialized = False
        self._init()

    def _init(self):
        """Sends the initialization command to the firmware."""
        try:
            res = self._device.send_report(
                bytes([report_const.FREQ_METER_INIT, self.pin_id])
            )
            if res[1] == report_const.OK:
                self._initialized = True
            elif res[1] == report_const.NOK:
                error_code = res[3] if len(res) > 3 else 0
                if error_code == 0x01:
                    raise RuntimeError(
                        f"FreqMeter Error: Pin {self.pin_id} or its PWM slice is busy."
                    )
                elif error_code == 0x02:
                    raise RuntimeError(
                        f"FreqMeter Error: No free DMA channel available."
                    )
                elif error_code == 0x03:
                    raise RuntimeError(
                        f"FreqMeter Error: Maximum concurrent measurements reached."
                    )
                elif error_code == 0x04:
                    raise RuntimeError(
                        f"FreqMeter Error: Pin {self.pin_id} must map to PWM Channel B."
                    )
                else:
                    raise RuntimeError(
                        f"FreqMeter failed to initialize pin {self.pin_id} (Unknown firmware error code: {error_code})."
                    )
            else:
                raise RuntimeError(
                    f"FreqMeter failed to initialize pin {self.pin_id} (Unexpected firmware response)."
                )
        except Exception as e:
            self._initialized = False
            # print(f"FreqMeter Initialization failed: {e}") # Optional logging
            raise  # Re-raise the exception

    def deinit(self):
        """Deinitialize frequency measurement and release resources on the firmware."""
        if not self._initialized:
            return
        try:
            # Send deinit command even if subsequent operations fail
            res = self._device.send_report(
                bytes([report_const.FREQ_METER_DEINIT, self.pin_id])
            )
            if res[1] != report_const.OK:
                # Log error but proceed with cleanup
                print(
                    f"Warning: FreqMeter firmware deinit for pin {self.pin_id} failed, but attempting cleanup."
                )
        except Exception as e:
            print(
                f"Warning: Exception during FreqMeter firmware deinit for pin {self.pin_id}: {e}"
            )
        finally:
            self._initialized = (
                False  # Mark as uninitialized regardless of firmware response
            )

    def __del__(self):
        """Ensure resources are released when the object is garbage collected."""
        self.deinit()

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Context manager exit."""
        self.deinit()

    def start(self):
        """
        Start or restart the edge capture process on the firmware.
        Clears previous captures and waits for new edges.
        """
        if not self._initialized:
            raise RuntimeError(
                "FreqMeter is not initialized. Call init() or create a new instance."
            )
        res = self._device.send_report(
            bytes([report_const.FREQ_METER_START, self.pin_id])
        )
        if res[1] != report_const.OK:
            error_code = res[3] if len(res) > 3 else 0
            if error_code == 0x01:
                raise RuntimeError(
                    f"FreqMeter start error: Pin {self.pin_id} not initialized."
                )
            else:
                raise RuntimeError(
                    f"FreqMeter failed to start measurement on pin {self.pin_id}."
                )

    def read_period_ticks(self):
        """
        Reads the last calculated average period between edges in raw timer ticks (microseconds).

        Returns:
            int: Average period in microseconds.

        Raises:
            RuntimeError: If not initialized or if the firmware reports a measurement error.
        """
        if not self._initialized:
            raise RuntimeError("FreqMeter is not initialized.")

        res = self._device.send_report(
            bytes([report_const.FREQ_METER_GET_PERIOD_TICKS, self.pin_id])
        )

        if res[1] == report_const.OK:
            # Response: | CMD | OK | GPIO | Ticks[0] | Ticks[1] | Ticks[2] | Ticks[3] | ...
            if len(res) >= 7:
                period_ticks = int.from_bytes(res[3:7], byteorder='little')
                return period_ticks
            else:
                raise RuntimeError(
                    f"FreqMeter received incomplete data for pin {self.pin_id}."
                )
        elif res[1] == report_const.NOK:
            error_code = res[3] if len(res) > 3 else 0
            if error_code == 0x01:
                raise RuntimeError(
                    f"FreqMeter read error: Pin {self.pin_id} not initialized."
                )
            elif error_code == 0x02:
                raise RuntimeError(
                    f"FreqMeter read error: Measurement error or timeout on pin {self.pin_id} (e.g., no edges detected)."
                )
            else:
                raise RuntimeError(
                    f"FreqMeter failed to read period for pin {self.pin_id} (Unknown firmware error code: {error_code})."
                )
        else:
            raise RuntimeError(
                f"FreqMeter failed to read period for pin {self.pin_id} (Unexpected firmware response)."
            )

    def frequency(self):
        """
        Reads the average period and calculates the frequency in Hertz.

        Returns:
            float: Measured frequency in Hz, or 0.0 if the period is zero.

        Raises:
            RuntimeError: If reading the period from the firmware fails.
        """
        period_ticks = self.read_period_ticks()
        if period_ticks > 0:
            # RP2040 timer runs at 1 MHz -> 1 tick = 1 microsecond
            frequency_hz = 1_000_000.0 / period_ticks
            return frequency_hz
        else:
            return 0.0
