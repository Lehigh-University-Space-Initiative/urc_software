import time
import struct
import serial
import RPi.GPIO as GPIO

# --- Hardware Configuration ---
SERIAL_PORT = '/dev/serial0' # Built-in RPi UART (GPIO 14 TX, GPIO 15 RX)
BAUDRATE = 9600
DE_RE_PIN = 4                # GPIO pin tied to MAX485 DE and RE pins

# Sensor Query Frames (Address 0x01, Function 0x03, Register Address, Length 1, CRC16)
QUERIES = {
    "moisture":     bytes([0x01, 0x03, 0x00, 0x12, 0x00, 0x01, 0x24, 0x0F]),
    "temperature":  bytes([0x01, 0x03, 0x00, 0x13, 0x00, 0x01, 0x75, 0xCF]),
    "ph":           bytes([0x01, 0x03, 0x00, 0x06, 0x00, 0x01, 0x64, 0x0B]),
    "conductivity": bytes([0x01, 0x03, 0x00, 0x15, 0x00, 0x01, 0x95, 0xCE]),
    "nitrogen":     bytes([0x01, 0x03, 0x00, 0x1E, 0x00, 0x01, 0xE4, 0x0C]),
    "phosphorus":   bytes([0x01, 0x03, 0x00, 0x1F, 0x00, 0x01, 0xB5, 0xCC]),
    "potassium":    bytes([0x01, 0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xC0]),
}

def setup_gpio():
    """Initializes the GPIO pin for RS485 Transmit/Receive direction control."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(DE_RE_PIN, GPIO.OUT)
    GPIO.output(DE_RE_PIN, GPIO.LOW) # Default to Receive Mode

def send_query_and_get_raw(ser, query_bytes, expected_len=7):
    """Handles RS485 direction switching, clears buffers, sends query, and reads response."""
    ser.reset_input_buffer()

    # 1. Switch MAX485 to Transmit Mode
    GPIO.output(DE_RE_PIN, GPIO.HIGH)
    time.sleep(0.001)

    # 2. Transmit frame
    ser.write(query_bytes)
    ser.flush()
    time.sleep(0.002) # Let the last byte fully leave the hardware buffer

    # 3. Switch MAX485 back to Receive Mode
    GPIO.output(DE_RE_PIN, GPIO.LOW)
    time.sleep(0.001)

    # 4. Read response
    response = ser.read(expected_len)

    if len(response) == expected_len and response[0] == 0x01 and response[1] == 0x03:
        raw_val = (response[3] << 8) | response[4]
        return raw_val
    return None

# --- Individual Parameter Reading Functions ---

def read_moisture(ser):
    raw = send_query_and_get_raw(ser, QUERIES["moisture"])
    return (raw / 10.0) if raw is not None else None

def read_temperature(ser):
    raw = send_query_and_get_raw(ser, QUERIES["temperature"])
    if raw is not None:
        signed_raw = struct.unpack('>h', raw.to_bytes(2, 'big'))[0]
        return signed_raw / 10.0
    return None

def read_ph(ser):
    raw = send_query_and_get_raw(ser, QUERIES["ph"])
    return (raw / 100.0) if raw is not None else None

def read_conductivity(ser):
    return send_query_and_get_raw(ser, QUERIES["conductivity"])

def read_nitrogen(ser):
    return send_query_and_get_raw(ser, QUERIES["nitrogen"])

def read_phosphorus(ser):
    return send_query_and_get_raw(ser, QUERIES["phosphorus"])

def read_potassium(ser):
    return send_query_and_get_raw(ser, QUERIES["potassium"])

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