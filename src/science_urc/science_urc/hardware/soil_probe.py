
import time


class SoilProbe:
    def __init__(self, port="/dev/serial0", baudrate=9600,
                 direction_pin=4, serial_connection=None, gpio=None):
        self.direction_pin = direction_pin
        self.gpio = gpio
        self.serial = serial_connection
        self.closed = False

        if self.gpio is None:
            import RPi.GPIO as GPIO
            self.gpio = GPIO

        self.gpio.setmode(self.gpio.BCM)
        self.gpio.setup(self.direction_pin, self.gpio.OUT)
        self.gpio.output(self.direction_pin, self.gpio.LOW)

        if self.serial is None:
            import serial
            self.serial = serial.Serial(port, baudrate, timeout=0.4)

    def read_register(self, query):
        if self.closed:
            raise RuntimeError("Soil probe is closed")

        self.serial.reset_input_buffer()
        self.gpio.output(self.direction_pin, self.gpio.HIGH)

        try:
            time.sleep(0.001)
            self.serial.write(query)
            self.serial.flush()
            time.sleep(0.002)
        finally:
            self.gpio.output(self.direction_pin, self.gpio.LOW)

        time.sleep(0.001)
        response = self.serial.read(7)

        if (len(response) != 7 or response[0] != 0x01
                or response[1] != 0x03 or response[2] != 0x02):
            return None

        return (response[3] << 8) | response[4]

    def read_measurements(self):
        queries = {
            "moisture": bytes.fromhex("01 03 00 12 00 01 24 0F"),
            "temperature": bytes.fromhex("01 03 00 13 00 01 75 CF"),
            "ph": bytes.fromhex("01 03 00 06 00 01 64 0B"),
            "conductivity": bytes.fromhex("01 03 00 15 00 01 95 CE"),
            "nitrogen": bytes.fromhex("01 03 00 1E 00 01 E4 0C"),
            "phosphorus": bytes.fromhex("01 03 00 1F 00 01 B5 CC"),
            "potassium": bytes.fromhex("01 03 00 20 00 01 85 C0"),
        }

        readings = {}

        for name, query in queries.items():
            raw = self.read_register(query)

            if raw is None:
                readings[name] = None
            elif name == "temperature":
                if raw >= 0x8000:
                    raw -= 0x10000
                readings[name] = raw / 10.0
            elif name == "moisture":
                readings[name] = raw / 10.0
            elif name == "ph":
                readings[name] = raw / 100.0
            else:
                readings[name] = float(raw)

            time.sleep(0.15)

        return readings

    def close(self):
        if not self.closed:
            self.closed = True
            try:
                self.serial.close()
            finally:
                self.gpio.output(self.direction_pin, self.gpio.LOW)
                self.gpio.cleanup(self.direction_pin)
