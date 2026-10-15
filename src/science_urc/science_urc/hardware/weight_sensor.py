
"""Hardware interface for the LUSI Science Module weight sensor."""

# HX711 pins use BCM GPIO numbering.
DT_PIN = 27
SCK_PIN = 17


class WeightSensor:
    """Reads the load cell through an HX711 amplifier."""

    def __init__(self, hx=None, gpio=None):
        # Keep track of whether this class created the hardware connection.
        self.owns_hardware = hx is None

        if hx is None:
            # Only import Pi libraries when using the real sensor.
            # This lets us run tests in Docker without GPIO hardware.
            import RPi.GPIO as GPIO
            from hx711 import HX711

            self.gpio = GPIO
            self.hx = HX711(DT_PIN, SCK_PIN)
        else:
            # Tests can pass in a fake HX711 object.
            self.hx = hx
            self.gpio = gpio

        self.closed = False

        # Measurements are not considered grams until calibration is verified.
        self.calibrated = False

        self.reference_unit = None

    def tare(self):
        """Set the current load cell reading to zero."""
        if self.closed:
            raise RuntimeError("Weight sensor is closed")

        # The container should be empty when this is called.
        self.hx.tare()

    def set_calibration(self, reference_unit):
        """Configure the HX711 scale using a verified reference unit."""
        if self.closed:
            raise RuntimeError("Weight sensor is closed")

        if isinstance(reference_unit, bool) or not isinstance(
            reference_unit, (int, float)
        ):
            raise ValueError("Reference unit must be a number")

        import math

        if not math.isfinite(reference_unit) or reference_unit == 0:
            raise ValueError("Reference unit must be finite and nonzero")

        self.calibrated = False

        # The reference unit must come from a known-mass calibration.
        self.hx.set_reference_unit(reference_unit)
        self.reference_unit = float(reference_unit)
        self.calibrated = True
        
    def read_weight(self, samples=5):
        """Read the average load cell value using the given sample count."""
        if self.closed:
            raise RuntimeError("Weight sensor is closed")

        if not isinstance(samples, int) or isinstance(samples, bool):
            raise ValueError("Sample count must be an integer")

        if samples < 1:
            raise ValueError("Sample count must be at least 1")

        # Average multiple readings to reduce noise.
        weight = self.hx.get_weight(samples)

        # Match the original script's power cycle between readings.
        self.hx.power_down()
        self.hx.power_up()

        return float(weight)

    def close(self):
        """Clean up the GPIO pins used by the weight sensor."""
        if self.closed:
            return

        self.closed = True

        # Only release the HX711 pins, not other GPIO channels.
        if self.owns_hardware and self.gpio is not None:
            self.gpio.cleanup((DT_PIN, SCK_PIN))