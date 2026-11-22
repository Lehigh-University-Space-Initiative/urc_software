
import time


def modbus_crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, crc >> 8])


class SoilProbe:
    def __init__(self, port="/dev/serial0", baudrate=9600,
                 direction_pin=4, serial_connection=None, gpio=None):
        self.baudrate = baudrate
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
            # Don't use serial.flush() here: on the Pi it can return a few
            # ms late, and the probe answers within ~1 ms, so the start of
            # its reply is lost. Release the bus as soon as the query
            # (10 bits per byte) has left the UART.
            start = time.perf_counter()
            self.serial.write(query)
            done = start + len(query) * 10 / self.baudrate + 0.0003
            while time.perf_counter() < done:
                pass
        finally:
            self.gpio.output(self.direction_pin, self.gpio.LOW)

        response = self.serial.read(7)

        if (len(response) != 7 or response[0] != 0x01
                or response[1] != 0x03 or response[2] != 0x02
                or modbus_crc16(response[:5]) != response[5:]):
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
