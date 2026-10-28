import time
import struct
import serial
import RPi.GPIO as GPIO

# --- Hardware Configuration ---
SERIAL_PORT = '/dev/serial0'  # Built-in RPi UART (GPIO 14 TX, GPIO 15 RX)
BAUDRATE = 9600
DE_RE_PIN = 4                 # GPIO pin tied to MAX485 DE and RE pins

# Sensor register addresses (device address 0x01, function 0x03)
REGISTERS = {
    "moisture":     0x0012,
    "temperature":  0x0013,
    "ph":           0x0006,
    "conductivity": 0x0015,
    "nitrogen":     0x001E,
    "phosphorus":   0x001F,
    "potassium":    0x0020,
}


def setup_gpio():
    """Initializes the GPIO pin for RS485 Transmit/Receive direction control."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(DE_RE_PIN, GPIO.OUT)
    GPIO.output(DE_RE_PIN, GPIO.LOW)  # Default to Receive Mode


# --- Modbus helpers ---

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def build_query(reg, addr=0x01):
    frame = bytes([addr, 0x03, reg >> 8, reg & 0xFF, 0x00, 0x01])
    return frame + struct.pack('<H', crc16(frame))  # CRC is low byte first


def read_register(ser, reg, retries=3):
    """Sends one query, waits for the frame to fully leave, then reads and validates the reply."""
    frame = build_query(reg)
    tx_time = len(frame) * 10 / BAUDRATE  # ~8.3 ms at 9600 baud

    for _ in range(retries):
        ser.reset_input_buffer()

        GPIO.output(DE_RE_PIN, GPIO.HIGH)
        time.sleep(0.002)
        t_start = time.monotonic()
        ser.write(frame)
        ser.flush()
        # Wait only for the transmit time still left (plus a small margin),
        # so we release the line promptly and don't miss the start of the reply
        remaining = tx_time + 0.001 - (time.monotonic() - t_start)
        if remaining > 0:
            time.sleep(remaining)
        GPIO.output(DE_RE_PIN, GPIO.LOW)

        resp = ser.read(7)
        if (len(resp) == 7
                and resp[0] == 0x01 and resp[1] == 0x03 and resp[2] == 0x02
                and crc16(resp[:5]) == struct.unpack('<H', resp[5:7])[0]):
            return (resp[3] << 8) | resp[4]

        time.sleep(0.1)  # let the bus settle before retrying
    return None


# --- Individual Parameter Reading Functions ---

def read_moisture(ser):
    raw = read_register(ser, REGISTERS["moisture"])
    return (raw / 10.0) if raw is not None else None


def read_temperature(ser):
    raw = read_register(ser, REGISTERS["temperature"])
    if raw is not None:
        signed_raw = struct.unpack('>h', raw.to_bytes(2, 'big'))[0]
        return signed_raw / 10.0
    return None


def read_ph(ser):
    raw = read_register(ser, REGISTERS["ph"])
    return (raw / 100.0) if raw is not None else None


def read_conductivity(ser):
    return read_register(ser, REGISTERS["conductivity"])


def read_nitrogen(ser):
    return read_register(ser, REGISTERS["nitrogen"])


def read_phosphorus(ser):
    return read_register(ser, REGISTERS["phosphorus"])


def read_potassium(ser):
    return read_register(ser, REGISTERS["potassium"])


# --- Main Loop ---

def main():
    setup_gpio()

    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.4)
        print(f"Connected to {SERIAL_PORT} at {BAUDRATE} baud.\n")
    except Exception as e:
        print(f"Failed to open serial port {SERIAL_PORT}: {e}")
        GPIO.cleanup()
        return

    try:
        while True:
            print("--- Soil 7-in-1 Sensor Reader ---")

            m = read_moisture(ser)
            print(f"Soil Moisture:    {m:.1f} %" if m is not None else "Error reading Moisture")
            time.sleep(0.15)

            t = read_temperature(ser)
            print(f"Temperature:      {t:.1f} °C" if t is not None else "Error reading Temperature")
            time.sleep(0.15)

            ph = read_ph(ser)
            print(f"Soil pH:          {ph:.2f}" if ph is not None else "Error reading pH")
            time.sleep(0.15)

            ec = read_conductivity(ser)
            print(f"Conductivity:     {ec} us/cm" if ec is not None else "Error reading Conductivity")
            time.sleep(0.15)

            n = read_nitrogen(ser)
            print(f"Nitrogen (N):     {n} mg/kg" if n is not None else "Error reading Nitrogen")
            time.sleep(0.15)

            p = read_phosphorus(ser)
            print(f"Phosphorus (P):   {p} mg/kg" if p is not None else "Error reading Phosphorus")
            time.sleep(0.15)

            k = read_potassium(ser)
            print(f"Potassium (K):    {k} mg/kg" if k is not None else "Error reading Potassium")

            print("---------------------------------------\n")
            time.sleep(2.0)

    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        ser.close()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
