"""
LUSI Science Module motor controller.

Reusable Raspberry Pi motor-control logic for standalone
operation and future ROS 2 integration.
"""

# GPIO pin assignments (BCM numbering)
AUGER_PIN = 12
PLATFORM_PIN = 13
CHAMBER_LID_PIN = 18
SOIL_DROP_PIN = 19

# REV Spark MAX PWM pulse widths (microseconds)
SPARK_NEUTRAL_US = 1500
SPARK_MAX_FWD_US = 2000
SPARK_MAX_REV_US = 1000

# Chamber lid servo pulse widths (microseconds)
CHAMBER_LID_MIN_US = 500
CHAMBER_LID_MAX_US = 2500

# Soil dropper servo pulse widths (microseconds)
SOIL_DROP_MIN_US = 500
SOIL_DROP_MAX_US = 2500

def speed_to_pulse_width(speed_pct, direction="forward"):
    """
    Convert motor speed (0–100%) to a Spark MAX PWM pulse width.

    Forward: 1500–2000 microseconds
    Reverse: 1500–1000 microseconds
    """
    if not 0 <= speed_pct <= 100:
        raise ValueError("Speed must be between 0 and 100")

    if direction == "forward":
        return SPARK_NEUTRAL_US + (speed_pct / 100.0) * (
            SPARK_MAX_FWD_US - SPARK_NEUTRAL_US
        )

    elif direction == "reverse":
        return SPARK_NEUTRAL_US - (speed_pct / 100.0) * (
            SPARK_NEUTRAL_US - SPARK_MAX_REV_US
        )

    raise ValueError("Direction must be 'forward' or 'reverse'")


def angle_to_pulse_width(angle, min_us, max_us):
    """
    Convert servo angle (0–180 degrees) to PWM pulse width.

    Preserves the original CLI's angle-clamping behavior.
    """
    angle = max(0, min(180, angle))

    return min_us + (angle / 180.0) * (max_us - min_us)

class MotorController:
    """Manage the Science Module's Raspberry Pi motor hardware."""

    def __init__(self, pi=None):
        """
        Connect to pigpio and initialize the motors.

        An existing pigpio connection can be supplied for testing.
        """
        self._owns_connection = pi is None

        if pi is None:
            import pigpio

            pi = pigpio.pi()

            if not pi.connected:
                pi.stop()
                raise RuntimeError(
                    "Cannot connect to pigpio daemon. "
                    "Start it with: sudo pigpiod"
                )

        self.pi = pi

        # Start the Spark MAX controllers at neutral.
        self.pi.set_servo_pulsewidth(AUGER_PIN, SPARK_NEUTRAL_US)
        self.pi.set_servo_pulsewidth(PLATFORM_PIN, SPARK_NEUTRAL_US)

        # Disable both servo signals at startup.
        self.pi.wave_clear()
        self.pi.write(CHAMBER_LID_PIN, 0)
        self.pi.set_servo_pulsewidth(SOIL_DROP_PIN, 0)   

    def set_auger_speed(self, speed_pct):
        """Set auger speed from 0 to 100 percent."""
        pulse_width = speed_to_pulse_width(speed_pct, "forward")
        self.pi.set_servo_pulsewidth(AUGER_PIN, int(pulse_width))

    def set_platform_speed(self, speed_pct, direction="up"):
        """Set platform speed and direction."""
        if direction not in ("up", "down"):
            raise ValueError("Platform direction must be 'up' or 'down'")

        spark_direction = "forward" if direction == "up" else "reverse"
        pulse_width = speed_to_pulse_width(speed_pct, spark_direction)

        self.pi.set_servo_pulsewidth(PLATFORM_PIN, int(pulse_width))

    def set_soil_drop_angle(self, angle):
        """Set soil dropper servo angle."""
        if not 0 <= angle <= 180:
            raise ValueError("Servo angle must be between 0 and 180")

        pulse_width = angle_to_pulse_width(
            angle, SOIL_DROP_MIN_US, SOIL_DROP_MAX_US
        )

        self.pi.set_servo_pulsewidth(SOIL_DROP_PIN, int(pulse_width))
    
    def set_chamber_lid_angle(self, angle):
        """Set chamber-lid servo angle from 0 to 180 degrees."""
        if not 0 <= angle <= 180:
            raise ValueError("Servo angle must be between 0 and 180")

        # Match the original CLI's behavior:
        # zero degrees disables the servo signal.
        if angle == 0:
            self.pi.set_servo_pulsewidth(CHAMBER_LID_PIN, 0)
            return

        pulse_width = angle_to_pulse_width(
            angle, CHAMBER_LID_MIN_US, CHAMBER_LID_MAX_US
        )

        self.pi.set_servo_pulsewidth(
            CHAMBER_LID_PIN, int(pulse_width)
        )    
    def stop_all(self):
        """Stop both motors and disable the servo signals."""
        # Neutral PWM stops the Spark MAX motors.
        self.pi.set_servo_pulsewidth(AUGER_PIN, SPARK_NEUTRAL_US)
        self.pi.set_servo_pulsewidth(PLATFORM_PIN, SPARK_NEUTRAL_US)

        # Disable chamber lid and soil dropper signals.
        self.pi.wave_tx_stop()
        self.pi.set_servo_pulsewidth(CHAMBER_LID_PIN, 0)
        self.pi.write(CHAMBER_LID_PIN, 0)
        self.pi.set_servo_pulsewidth(SOIL_DROP_PIN, 0)

    def close(self):
        """Stop motors and release an owned pigpio connection."""
        try:
            self.stop_all()
        finally:
            if self._owns_connection:
                self.pi.stop()    