#####################################################################################################################
# Serial Port Lister / GNSS Probe
#   - Lists available serial ports with descriptions
#   - Optional: probes for NMEA (GGA/RMC) at common baud rates
#####################################################################################################################

import argparse
from typing import Iterable, Optional, Tuple


def list_ports():
    try:
        from serial.tools import list_ports  # type: ignore
    except Exception:
        print("pyserial not installed. Install with: pip install pyserial")
        return []
    return list(list_ports.comports())


def probe_port(port: str, bauds: Iterable[int], seconds: float = 1.5) -> Optional[Tuple[int, str]]:
    try:
        import time
        import serial  # type: ignore
    except Exception:
        return None
    for b in bauds:
        try:
            ser = serial.Serial(port, b, timeout=0.2)
        except Exception:
            continue
        with ser:
            t0 = time.perf_counter()
            while (time.perf_counter() - t0) < seconds:
                try:
                    line = ser.readline().decode(errors="ignore").strip()
                    if not line or not line.startswith("$"):
                        continue
                    if "GGA" in line or "RMC" in line:
                        return (b, line)
                except Exception:
                    break
    return None


def main():
    parser = argparse.ArgumentParser(description="List serial ports and optionally probe for GNSS/NMEA")
    parser.add_argument("--probe", action="store_true", help="Probe ports for NMEA ($..GGA/$..RMC)")
    parser.add_argument("--seconds", type=float, default=1.5, help="Probe duration per baud")
    parser.add_argument("--baud", type=int, nargs="*", default=[9600, 38400, 115200], help="Baud rates to try")
    args = parser.parse_args()

    ports = list_ports()
    if not ports:
        print("No serial ports found.")
        return

    print("Detected serial ports:\n")
    for p in ports:
        print(f"- {p.device}\n  desc: {p.description}\n  hwid: {p.hwid}")
        if args.probe:
            res = probe_port(p.device, args.baud, args.seconds)
            if res:
                b, sample = res
                print(f"  NMEA: yes @ {b} baud | sample: {sample[:80]}")
            else:
                print("  NMEA: no")
        print("")


if __name__ == "__main__":
    main()

