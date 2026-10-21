#!/usr/bin/env python3
# This shebang line tells Linux and macOS to use Python 3 when the script is
# run directly from the terminal (e.g., ./Camera_System.py). On Windows,
# this line is ignored - you run scripts with "python Camera_System.py" instead.


#####################################################################################################################
# LUSI Science Module - Camera System
#
# This script provides a unified interface for the ELP USB camera and related utilities.
# It runs on a Raspberry Pi or Windows computer and supports three modes:
#
# Modes:
#   1. Live Preview (default)
#     - Opens the camera with optional GNSS overlay, scale bar, photo/video capture
#     - Usage: python Camera_System.py [camera options]
#
#   2. Camera Scan (scan)
#     - Scans for connected camera devices by index and reports which are accessible
#     - Usage: python Camera_System.py scan
#
#   3. Serial Port Listing (ports)
#     - Lists available serial ports with descriptions, optionally probes for GNSS/NMEA
#     - Usage: python Camera_System.py ports [--probe]
#####################################################################################################################




#####################################################################################################################
# Importing Program Libraries
#   - argparse:
#       - Parses command-line arguments so users can configure camera settings, choose
#         subcommands (scan, ports, live preview), and set options like resolution or codec
#   - json:
#       - Reads and writes JSON files
#       - Used to save metadata "sidecar" files alongside captured photos (camera settings,
#         GNSS coordinates, timestamps, etc.)
#   - os:
#       - Provides file and directory utilities (creating folders, building file paths)
#       - Detects the operating system so we can use DirectShow on Windows
#   - time:
#       - Measures elapsed time for FPS (frames per second) calculations
#       - Adds small delays during serial port probing so we don't overwhelm the device
#   - datetime:
#       - Generates ISO-8601 timestamps in UTC for naming captured files
#       - Ensures every photo/video has a unique, sortable filename
#   - cv2 (OpenCV):
#       - The main computer vision library
#       - Opens the camera, reads frames, displays the live preview window
#       - Draws text overlays (resolution, FPS, GNSS coordinates) on the video feed
#       - Saves still images as PNG and records video as AVI
#   - math:
#       - Provides trigonometric functions (tangent, radians)
#       - Used to calculate meters-per-pixel from the camera's field of view and altitude
#   - threading:
#       - Runs background tasks in separate threads so they don't block the camera feed
#       - The GNSS reader and Windows Location reader each run in their own thread
#   - typing:
#       - Provides type hints (Optional, Tuple, Iterable) that document what kind of
#         values a function expects and returns, making the code easier to understand
#   - serial (pyserial):
#       - Communicates with serial devices (USB-to-serial adapters, GNSS receivers)
#       - Reads NMEA sentences from GPS/GNSS hardware connected via COM/ttyUSB ports
#       - If pyserial is not installed, any feature that uses serial will print a
#         helpful error message instead of crashing the whole script
#   - serial.tools.list_ports:
#       - Discovers all serial ports on the system with their descriptions and hardware IDs
#       - Helps users find which COM port their GNSS receiver is connected to
#####################################################################################################################

import argparse
import json
import os
import time
from datetime import datetime, timezone
import cv2
import math
import threading
from typing import Iterable, Optional, Tuple

# pyserial is optional - if it is not installed, serial features will print an
# error message instead of crashing the script
try:
    import serial  # type: ignore
    from serial.tools import list_ports as serialListPortsModule  # type: ignore
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False




#####################################################################################################################
# Camera Scanning
#   - Scans for connected camera devices across possible index values
#   - Each index corresponds to a unique camera recognized by the operating system
#   - Verifies connection by attempting to open the camera and read a frame
#   - Prints a list showing which indices correspond to valid, accessible cameras
#   - Typically:
#       - Index 0: Built-in laptop webcam (if present)
#       - Index 1+: External USB cameras (ex. ELP USB camera)
#
#   Parameters:
#       totalIndices (int) - how many camera indices to try, starting from 0
#
#   How it works:
#       For each index 0 through totalIndices-1, it tries to open the camera with
#       OpenCV and read one frame. If the frame comes back successfully, that index
#       has a working camera. The camera handle is released immediately after testing
#       so it doesn't stay locked.
#####################################################################################################################

def listCameras(totalIndices: int = 5):
    """Scan camera indices and report which are accessible."""
    print("[INFO] Scanning for available cameras...")
    for cameraIndex in range(totalIndices):
        cameraCapture = cv2.VideoCapture(cameraIndex)  # Attempt to open the camera at this index
        success, capturedFrame = cameraCapture.read()  # Try reading one frame to confirm access

        if success:
            print(f"[INFO] Camera detected at index {cameraIndex}")
        else:
            print(f"[INFO] No camera detected at index {cameraIndex}")

        cameraCapture.release()  # Release device handle to prevent resource lock

    print("[INFO] Scan complete.")




#####################################################################################################################
# Serial Port Listing
#   - Discovers all serial ports currently available on the system
#   - Each serial port represents a physical or virtual connection point for devices
#     like GNSS receivers, Arduino boards, or USB-to-serial adapters
#   - Returns a list of port info objects containing the device path, human-readable
#     description, and hardware ID
#
#   Returns:
#       list - a list of serial port info objects (empty if pyserial is not installed)
#####################################################################################################################

def listPorts() -> list:
    """Return a list of available serial ports on the system."""
    if not SERIAL_AVAILABLE:
        print("[ERROR] pyserial not installed. Install with: pip install pyserial")
        return []
    return list(serialListPortsModule.comports())




#####################################################################################################################
# Serial Port GNSS Probing
#   - Tests a single serial port for NMEA sentences at various baud rates
#   - NMEA is the standard GPS/GNSS data format - sentences start with "$" and contain
#     fix data like latitude, longitude, altitude, and satellite count
#   - GGA sentences contain position and altitude data
#   - RMC sentences contain position and speed data
#   - The function tries each baud rate for a configurable number of seconds
#
#   Parameters:
#       port (str)     - the serial port device path (e.g., "COM3" or "/dev/ttyUSB0")
#       bauds (tuple)  - baud rates to try (e.g., (9600, 38400, 115200))
#       seconds (float) - how long to listen at each baud rate before giving up
#
#   Returns:
#       tuple or None - (baudRate, sampleSentence) if NMEA found, None otherwise
#####################################################################################################################

def probePort(port: str, bauds: tuple, seconds: float = 1.5) -> Optional[Tuple[int, str]]:
    """Probe a serial port for NMEA sentences at various baud rates."""
    if not SERIAL_AVAILABLE:
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




#####################################################################################################################
# Serial Port Listing Command
#   - Combines port listing and optional GNSS probing into one user-facing command
#   - Prints each port's device path, description, and hardware ID
#   - When --probe is used, also tests each port for NMEA data at common baud rates
#
#   Parameters:
#       args (Namespace) - parsed CLI arguments containing .probe, .baud, and .seconds
#####################################################################################################################

def runPortsCommand(args):
    """List serial ports and optionally probe for GNSS/NMEA."""
    ports = listPorts()
    if not ports:
        print("[INFO] No serial ports found.")
        return

    print("[INFO] Detected serial ports:\n")
    for p in ports:
        deviceLine = f"- {p.device}"
        descLine = f"  desc: {p.description}"
        hwidLine = f"  hwid: {p.hwid}"
        print(deviceLine)
        print(descLine)
        print(hwidLine)
        if args.probe:
            result = probePort(p.device, args.baud, args.seconds)
            if result:
                baud, sample = result
                nmeaLine = f"  NMEA: yes @ {baud} baud | sample: {sample[:80]}"
                print(nmeaLine)
            else:
                print("  NMEA: no")
        print("")




#####################################################################################################################
# Camera Initialization
#   - Opens a camera by its index number using OpenCV
#   - On Windows, tries the DirectShow backend first for better compatibility with USB
#     cameras - DirectShow is a Windows media framework that gives more reliable access
#     to camera settings like resolution and exposure
#   - If DirectShow fails, falls back to the default OpenCV backend
#   - On Linux/macOS, always uses the default backend (V4L2 on Linux, AVFoundation on Mac)
#
#   Parameters:
#       index (int)     - camera index number (0 = first camera, 1 = second, etc.)
#       useDshow (bool) - whether to try DirectShow backend on Windows
#
#   Returns:
#       cv2.VideoCapture - an OpenCV camera capture object (check .isOpened() before use)
#####################################################################################################################

def openCamera(index: int, useDshow: bool) -> cv2.VideoCapture:
    """Open a camera by index, with optional DirectShow backend on Windows."""
    backend = cv2.CAP_DSHOW if (os.name == "nt" and useDshow) else 0
    cameraCapture = cv2.VideoCapture(index, backend)

    # If DirectShow failed on Windows, try the default backend as a fallback
    if not cameraCapture.isOpened() and backend == cv2.CAP_DSHOW:
        cameraCapture = cv2.VideoCapture(index)

    return cameraCapture




#####################################################################################################################
# Filesystem Utilities
#   - Creates directories if they don't already exist
#   - Used to create the "photos" and "videos" folders inside the captures directory
#   - os.makedirs with exist_ok=True means it won't crash if the folder is already there
#
#   Parameters:
#       path (str) - the directory path to create
#####################################################################################################################

def ensureDir(path: str):
    """Create a directory (and any parent directories) if it doesn't already exist."""
    os.makedirs(path, exist_ok=True)




#####################################################################################################################
# Overlay Utilities
#   - Draws text on a camera frame using OpenCV
#   - Used for on-screen status info: resolution, FPS, GNSS coordinates, hotkey hints
#   - The text is drawn in yellow (BGR: 0, 255, 255) with anti-aliasing for readability
#   - The y parameter controls vertical position (higher values = further down the frame)
#
#   Parameters:
#       frame (numpy.ndarray) - the camera frame to draw on (modified in place)
#       text (str)            - the text string to display
#       y (int)               - vertical pixel position for the text (default: 20)
#####################################################################################################################

def putOverlay(frame, text: str, y: int = 20):
    """Draw yellow text on a camera frame at the given vertical position."""
    cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)




#####################################################################################################################
# NMEA Coordinate Conversion
#   - NMEA (National Marine Electronics Association) is the standard data format used by
#     GPS/GNSS receivers. Coordinates come in a special format that needs converting.
#   - NMEA sends latitude as "ddmm.mmmm" (degrees and minutes) and longitude as
#     "dddmm.mmmm" (3-digit degrees and minutes) along with a hemisphere letter (N/S/E/W)
#   - This function converts that format into standard decimal degrees that mapping
#     software and metadata files can use (e.g., 40.607842° N, -75.378252° W)
#   - Southern and Western hemispheres produce negative values
#
#   Parameters:
#       value (str) - the NMEA coordinate string (e.g., "4036.4705" for 40°36.4705')
#       hemi (str)  - hemisphere letter: "N", "S", "E", or "W"
#
#   Returns:
#       float or None - decimal degrees (negative for S/W), or None if parsing fails
#####################################################################################################################

def nmeaDeg(value: str, hemi: str) -> Optional[float]:
    """Convert NMEA ddmm.mmmm / dddmm.mmmm + hemisphere to signed decimal degrees."""
    try:
        if not value or not hemi:
            return None
        if "." not in value:
            return None
        idx = value.find(".")
        degLen = 2
        if len(value[:idx]) >= 5:  # Longitude path (3-digit degrees like 07538.1234)
            degLen = 3
        deg = float(value[:degLen])
        minutes = float(value[degLen:])
        dec = deg + minutes / 60.0
        if hemi in ("S", "W"):
            dec = -dec
        return dec
    except Exception:
        return None




#####################################################################################################################
# GGA Sentence Parsing
#   - GGA is one of the most important NMEA sentence types from GPS/GNSS receivers
#   - It contains: latitude, longitude, fix quality, satellite count, HDOP, and altitude
#   - Fix quality tells you how accurate the position is:
#       0 = no fix, 1 = GPS fix, 2 = DGPS fix, 4 = RTK fixed, 5 = RTK float
#   - HDOP (Horizontal Dilution of Precision) indicates position accuracy - lower is better
#   - Altitude is height above mean sea level in meters
#
#   Parameters:
#       fields (list) - the comma-separated fields of a GGA sentence (already split)
#
#   Returns:
#       dict - parsed values (lat, lon, fixQuality, satellites, hdop, alt)
#             values that couldn't be parsed are set to None
#####################################################################################################################

def parseGga(fields: list) -> dict:
    """Parse a $GxGGA sentence into a dict of lat, lon, fixQuality, satellites, hdop, alt."""
    data = {}
    try:
        data["lat"] = nmeaDeg(fields[2], fields[3]) if len(fields) > 4 else None
        data["lon"] = nmeaDeg(fields[4], fields[5]) if len(fields) > 6 else None
        data["fixQuality"] = int(fields[6]) if len(fields) > 6 and fields[6].isdigit() else None
        data["satellites"] = int(fields[7]) if len(fields) > 7 and fields[7].isdigit() else None
        data["hdop"] = float(fields[8]) if len(fields) > 8 and fields[8] != "" else None
        data["alt"] = float(fields[9]) if len(fields) > 9 and fields[9] != "" else None
    except Exception:
        pass
    return data




#####################################################################################################################
# RMC Sentence Parsing
#   - RMC (Recommended Minimum) is another common NMEA sentence type
#   - It contains: latitude, longitude, and speed over ground
#   - Speed comes in knots from the GNSS receiver and is converted to m/s and km/h
#
#   Parameters:
#       fields (list) - the comma-separated fields of an RMC sentence (already split)
#
#   Returns:
#       dict - parsed values (lat, lon, speedMs, speedKmh)
#             values that couldn't be parsed are set to None
#####################################################################################################################

def parseRmc(fields: list) -> dict:
    """Parse a $GxRMC sentence into a dict of lat, lon, speed."""
    data = {}
    try:
        data["lat"] = nmeaDeg(fields[3], fields[4]) if len(fields) > 5 else None
        data["lon"] = nmeaDeg(fields[5], fields[6]) if len(fields) > 7 else None
        if len(fields) > 7 and fields[7] != "":
            sogKnots = float(fields[7])
            data["speedMs"] = sogKnots * 0.514444  # Knots to meters per second
            data["speedKmh"] = sogKnots * 1.852  # Knots to kilometers per hour
    except Exception:
        pass
    return data




#####################################################################################################################
# GNSS Reader Class
#   - Reads NMEA sentences from a serial GPS/GNSS receiver in a background thread
#   - A "background thread" means this code runs at the same time as the camera feed
#     without slowing it down - the camera loop and GNSS reading happen simultaneously
#   - Continuously parses GGA and RMC sentences and stores the latest fix data
#   - The main camera loop can read self.latest at any time to get current coordinates
#   - Errors are stored in self.error so the camera overlay can show what went wrong
#
#   How it works:
#       1. The start() method launches a daemon thread that runs the _run() method
#       2. _run() opens the serial port and reads lines in an infinite loop
#       3. Each NMEA sentence is parsed and merged into the self.latest dictionary
#       4. The stop() method sets a flag that tells the thread to exit cleanly
#####################################################################################################################

class GNSSReader:
    """Background thread that reads NMEA sentences from a serial GNSS device."""

    def __init__(self, port: str, baud: int = 9600):
        self.port = port
        self.baud = baud
        self.thread = None
        self._stop = threading.Event()
        self.latest = {
            "lat": None,
            "lon": None,
            "alt": None,
            "hdop": None,
            "fixQuality": None,
            "satellites": None,
            "speedKmh": None,
        }
        self.error = None

    def start(self):
        """Start the background NMEA reading thread."""
        if self.thread and self.thread.is_alive():
            return
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        """Signal the background thread to stop and wait for it to finish."""
        self._stop.set()
        if self.thread:
            self.thread.join(timeout=0.5)

    def _run(self):
        """Main loop for the background thread - reads and parses NMEA sentences."""
        try:
            if not SERIAL_AVAILABLE:
                self.error = "pyserial not available"
                return
            ser = serial.Serial(self.port, self.baud, timeout=1)
            with ser:
                while not self._stop.is_set():
                    try:
                        line = ser.readline().decode(errors="ignore").strip()
                        if not line or not line.startswith("$"):
                            continue
                        # Strip the checksum (everything after "*") before parsing
                        if "*" in line:
                            line = line.split("*")[0]
                        parts = line.split(",")
                        # Extract the sentence type (GGA, RMC, etc.) from the talker ID
                        talker = parts[0][3:6] if len(parts[0]) >= 6 else parts[0][3:]
                        if talker == "GGA":
                            data = parseGga(parts)
                            self._merge(data)
                        elif talker == "RMC":
                            data = parseRmc(parts)
                            self._merge(data)
                    except Exception:
                        continue
        except Exception as e:
            self.error = str(e)

    def _merge(self, data: dict):
        """Update self.latest with any non-None values from a parsed sentence."""
        for k, v in data.items():
            if v is not None:
                self.latest[k] = v




#####################################################################################################################
# GNSS Auto-detection
#   - Automatically finds the correct serial port and baud rate for a connected GNSS
#     receiver by scanning all available ports and testing common baud rates
#   - Prioritizes ports whose descriptions match known GNSS hardware names (u-blox,
#     Quectel, SimCom, etc.) so it checks the most likely ports first
#   - Tests each port/baud combination for a configurable number of seconds
#   - Returns as soon as it finds the first port that sends valid NMEA sentences
#
#   Parameters:
#       bauds (tuple)    - baud rates to try at each port (default: 9600, 38400, 115200)
#       seconds (float)  - how long to listen at each baud rate before moving on
#
#   Returns:
#       tuple or None - (devicePath, baudRate) if NMEA found, None if no GNSS detected
#####################################################################################################################

def autodetectGnssPort(bauds: tuple = (9600, 38400, 115200), seconds: float = 1.5) -> Optional[Tuple[str, int]]:
    """Scan serial ports for NMEA talkers and return (device, baud) or None."""
    if not SERIAL_AVAILABLE:
        return None

    # Keywords that commonly appear in GNSS device descriptions
    PREFERRED_KEYWORDS = (
        "u-blox", "ublox", "GNSS", "GPS", "Quectel", "SimCom",
        "USB-SERIAL", "USB Serial", "Prolific", "CH340", "Silicon Labs",
    )
    ports = list(serialListPortsModule.comports())

    def score(p):
        """Score ports so likely GNSS devices are tested first (lower = higher priority)."""
        desc = (p.description or "").lower()
        return 0 if not desc else -sum(1 for k in PREFERRED_KEYWORDS if k.lower() in desc)

    portsSorted = sorted(ports, key=score)

    for p in portsSorted:
        dev = p.device
        for b in bauds:
            try:
                ser = serial.Serial(dev, b, timeout=0.2)
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
                            return (dev, b)
                    except Exception:
                        break
    return None




#####################################################################################################################
# Windows Location API Reader Class
#   - Provides GPS coordinates on Windows computers that have a built-in location sensor
#     or a USB GPS receiver that registers through the Windows Location API
#   - This is an alternative to serial GNSS - useful when no COM port is available
#   - Uses the "winrt" Python package to access the Windows.Devices.Geolocation API
#   - Runs in a background thread so it doesn't block the camera feed
#   - Location access must be enabled in Windows Settings --> Privacy --> Location
#
#   How it works:
#       1. start() launches a daemon thread that polls the Windows Location API
#       2. Each poll uses an async call to the Geolocator to get the current position
#       3. Latitude, longitude, altitude, and speed are stored in self.latest
#       4. The camera loop reads self.latest to show coordinates on the overlay
#       5. stop() signals the thread to exit cleanly
#####################################################################################################################

class WindowsLocationReader:
    """Background thread that polls the Windows Location API for coordinates."""

    def __init__(self, intervalSec: float = 1.0):
        self.interval = intervalSec
        self.thread = None
        self._stop = threading.Event()
        self.latest = {
            "lat": None,
            "lon": None,
            "alt": None,
            "speedKmh": None,
        }
        self.error = None

    def start(self):
        """Start the background location polling thread."""
        if self.thread and self.thread.is_alive():
            return
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        """Signal the background thread to stop and wait for it to finish."""
        self._stop.set()
        if self.thread:
            self.thread.join(timeout=0.5)

    def _run(self):
        """Main loop - polls Windows Location API and stores results."""
        try:
            import asyncio
            from winrt.windows.devices.geolocation import Geolocator, PositionAccuracy  # type: ignore
        except Exception:
            self.error = "winrt not available"
            return

        locator = Geolocator()
        try:
            locator.desired_accuracy = PositionAccuracy.HIGH
        except Exception:
            pass

        async def fetchOnce():
            """Make one async request to the Windows Geolocator."""
            try:
                pos = await locator.get_geoposition_async()
                coord = pos.coordinate
                p = coord.point.position
                lat = float(p.latitude)
                lon = float(p.longitude)
                alt = None
                try:
                    alt = float(p.altitude)
                except Exception:
                    alt = None
                speedKmh = None
                try:
                    if coord.speed is not None:
                        speedKmh = float(coord.speed) * 3.6  # m/s to km/h
                except Exception:
                    pass
                return {"lat": lat, "lon": lon, "alt": alt, "speedKmh": speedKmh}
            except Exception as e:
                return {"error": str(e)}

        while not self._stop.is_set():
            try:
                result = asyncio.run(fetchOnce())
                if isinstance(result, dict):
                    if "error" in result and result["error"]:
                        self.error = result["error"]
                    else:
                        for k, v in result.items():
                            if k != "error" and v is not None:
                                self.latest[k] = v
            except Exception as e:
                self.error = str(e)
            try:
                time.sleep(self.interval)
            except Exception:
                break




#####################################################################################################################
# Scale Bar - Meters Per Pixel Estimation
#   - Calculates how many real-world meters each pixel in the camera image represents
#   - This is essential for adding a meaningful scale bar to aerial or elevated images
#   - The calculation uses the camera's horizontal field of view (HFOV) and altitude
#     above the ground to determine how wide the image is in meters
#   - If the user provides a direct MPP (meters-per-pixel) override, that value is used
#     instead of calculating from altitude and HFOV
#
#   Parameters:
#       widthPx (int)           - frame width in pixels
#       altitudeM (float)       - altitude above ground in meters (from GNSS or manual)
#       hfovDeg (float)         - camera horizontal field of view in degrees
#       overrideMpp (float)     - direct meters-per-pixel value (bypasses calculation)
#
#   Returns:
#       float or None - meters per pixel, or None if not enough data to calculate
#
#   How it works:
#       The ground width visible in the image is:
#           widthMeters = 2 × altitude × tan(HFOV / 2)
#       Then meters per pixel is simply:
#           mpp = widthMeters / widthPixels
#####################################################################################################################

def estimateMpp(widthPx: int, altitudeM: Optional[float], hfovDeg: Optional[float], overrideMpp: Optional[float]) -> Optional[float]:
    """Calculate meters-per-pixel from altitude and horizontal FOV, or use override."""
    if overrideMpp and overrideMpp > 0:
        return overrideMpp
    if altitudeM is None or hfovDeg is None or widthPx <= 0:
        return None
    try:
        widthM = 2.0 * altitudeM * math.tan(math.radians(hfovDeg) / 2.0)
        if widthM <= 0:
            return None
        return widthM / float(widthPx)
    except Exception:
        return None




#####################################################################################################################
# Scale Bar - Drawing on Frame
#   - Draws a visual scale bar in the bottom-left corner of the camera frame
#   - The bar length is chosen to represent a "nice" round distance (e.g., 1 m, 5 m,
#     10 m, 100 m) rather than an awkward number
#   - Includes tick marks at both ends and a label showing the distance
#   - A dark background rectangle is drawn behind the bar for readability
#
#   Parameters:
#       frame (numpy.ndarray) - the camera frame to draw on (modified in place)
#       mpp (float)           - meters per pixel (from estimateMpp)
#
#   How it works:
#       1. Calculate a target bar length of ~1/5 the frame width
#       2. Convert that to meters using the MPP value
#       3. Round up to the nearest "nice" step (0.1, 0.2, 0.5, 1, 2, 5, 10, ...)
#       4. Convert back to pixels and draw the bar, tick marks, and label
#####################################################################################################################

def drawScaleBar(frame, mpp: float):
    """Draw a scale bar with label in the bottom-left corner of a camera frame."""
    if not mpp or mpp <= 0:
        return
    h, w = frame.shape[:2]
    targetPx = max(60, w // 5)
    targetM = targetPx * mpp

    # "Nice" round distances to choose from
    STEPS = [
        0.1, 0.2, 0.5,
        1, 2, 5,
        10, 20, 50,
        100, 200, 500,
        1000, 2000, 5000,
    ]
    bestM = STEPS[-1]
    for s in STEPS:
        if s >= targetM:
            bestM = s
            break

    barPx = int(round(bestM / mpp))
    margin = 16
    y = h - margin
    x0 = margin
    x1 = x0 + barPx

    # Draw dark background rectangle for readability
    pad = 6
    boxX0 = x0 - pad
    boxY0 = y - 22 - pad
    boxX1 = x1 + pad
    boxY1 = y + pad
    cv2.rectangle(frame, (boxX0, boxY0), (boxX1, boxY1), (0, 0, 0), thickness=-1)
    cv2.rectangle(frame, (boxX0, boxY0), (boxX1, boxY1), (255, 255, 255), thickness=1)

    # Draw the bar and tick marks
    cv2.line(frame, (x0, y), (x1, y), (255, 255, 255), 2)
    cv2.line(frame, (x0, y - 8), (x0, y + 8), (255, 255, 255), 2)
    cv2.line(frame, (x1, y - 8), (x1, y + 8), (255, 255, 255), 2)

    # Draw the distance label above the bar
    label = f"{bestM:g} m"
    cv2.putText(frame, label, (x0, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)




#####################################################################################################################
# Scale Bar - MPP Label Formatting
#   - Formats the meters-per-pixel value into a human-readable string
#   - Automatically chooses the best unit (mm/px, cm/px, or m/px) based on the value
#   - Small values (< 0.01 m/px) are shown in millimeters for clarity
#   - Medium values (< 1.0 m/px) are shown in centimeters
#   - Large values are shown in meters
#
#   Parameters:
#       mpp (float) - meters per pixel
#
#   Returns:
#       str or None - formatted label string, or None if mpp is invalid
#####################################################################################################################

def formatMppLabel(mpp: Optional[float]) -> Optional[str]:
    """Format a meters-per-pixel value as a human-readable scale label."""
    if mpp is None or mpp <= 0:
        return None
    if mpp < 0.01:
        label = f"Scale: {mpp * 1000:.1f} mm/px"
        return label
    if mpp < 1.0:
        label = f"Scale: {mpp * 100:.2f} cm/px"
        return label
    label = f"Scale: {mpp:.3f} m/px"
    return label




#####################################################################################################################
# Timestamp and Metadata Utilities
#   - nowUtcIso() generates a timestamp string in ISO-8601 format using UTC time
#     (e.g., "2025-02-22T18-30-45.123456Z"). Colons are replaced with hyphens so the
#     string is safe to use in filenames on all operating systems.
#   - sidecarMetadata() saves a JSON file alongside a captured photo containing all the
#     camera settings, GNSS coordinates, and scale information at the moment of capture.
#     The JSON file has the same name as the photo but with a .json extension.
#####################################################################################################################

def nowUtcIso() -> str:
    """Return the current UTC time as an ISO-8601 string safe for filenames."""
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%S.%fZ")


def sidecarMetadata(pngPath: str, metadata: dict):
    """Save a JSON metadata sidecar file next to a captured image."""
    metadataPath = os.path.splitext(pngPath)[0] + ".json"
    with open(metadataPath, "w", encoding="utf-8") as file:
        json.dump(metadata, file, indent=2)




#####################################################################################################################
# Live Preview - Main Camera Loop
#   - Opens the camera and displays a live preview window using OpenCV
#   - Overlays on the video feed: resolution, FPS, GNSS coordinates, scale bar
#   - Hotkeys during preview:
#       c --> capture a still photo (saved as PNG with JSON sidecar metadata)
#       r --> start/stop video recording (saved as AVI)
#       q --> quit the preview and close the camera
#   - The FPS display uses an exponential moving average (EMA) for smooth readings
#   - GNSS coordinates are read from a background thread (GNSSReader or WindowsLocationReader)
#
#   Parameters:
#       args (Namespace) - parsed CLI arguments with all camera and GNSS settings
#
#   How it works:
#       1. Create output directories for photos and videos
#       2. Open the camera and apply resolution/FPS/exposure settings
#       3. Optionally start a GNSS reader (serial or Windows Location API)
#       4. Enter the main loop: read frame --> draw overlays --> check hotkeys --> repeat
#       5. On exit, release the camera, close windows, and stop the GNSS reader
#####################################################################################################################

def runLivePreview(args):
    """Launch the live camera preview with overlays and capture/record support."""

    # ============================================================
    # SETUP - Create output directories
    # ============================================================
    ensureDir(args.saveDir)
    photosDir = os.path.join(args.saveDir, "photos")
    videosDir = os.path.join(args.saveDir, "videos")
    ensureDir(photosDir)
    ensureDir(videosDir)

    # ============================================================
    # SETUP - Open camera and apply settings
    # ============================================================
    cameraCapture = openCamera(args.index, useDshow=not args.noDshow)
    if not cameraCapture.isOpened():
        print("[ERROR] Camera not detected. Try a different index or check the USB connection.")
        return

    cameraCapture.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize frame buffer lag
    if args.width > 0:
        cameraCapture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height > 0:
        cameraCapture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if args.fps > 0:
        cameraCapture.set(cv2.CAP_PROP_FPS, args.fps)
    if args.exposure is not None:
        cameraCapture.set(cv2.CAP_PROP_EXPOSURE, args.exposure)
    if args.gain is not None:
        cameraCapture.set(cv2.CAP_PROP_GAIN, args.gain)

    actualWidth = int(cameraCapture.get(cv2.CAP_PROP_FRAME_WIDTH))
    actualHeight = int(cameraCapture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    nominalFps = cameraCapture.get(cv2.CAP_PROP_FPS) or args.fps or 0

    # ============================================================
    # SETUP - Recording state
    # ============================================================
    fourcc = cv2.VideoWriter_fourcc(*args.codec)
    isRecording = False
    videoWriter = None
    pendingRecordPath = args.record if args.record else None

    # ============================================================
    # SETUP - GNSS reader (serial or Windows Location API)
    # ============================================================
    gnss = None
    if args.gnssPort:
        port = args.gnssPort
        baud = args.gnssBaud
        if isinstance(port, str) and port.lower() == "auto":
            # Try to automatically find the GNSS device
            bauds = (baud,) if (isinstance(baud, (int, float)) and baud and baud > 0) else (9600, 38400, 115200)
            detected = autodetectGnssPort(bauds=bauds, seconds=2.0)
            if detected:
                port, baud = detected
                print(f"[INFO] GNSS auto-detected: {port} @ {baud} baud")
            else:
                print("[WARN] GNSS auto-detect failed: no NMEA found on available ports")
                port = None
        if port:
            gnss = GNSSReader(port, int(baud))
            gnss.start()
    elif args.winLocation:
        gnss = WindowsLocationReader(intervalSec=1.0)
        gnss.start()

    windowName = "ELP USB Camera"
    cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)

    # FPS tracking with exponential moving average
    fpsEma = 0.0
    alpha = 0.2  # Smoothing factor (0.0 = very smooth, 1.0 = no smoothing)
    lastTime = time.perf_counter()

    # Print startup info
    print("[INFO] Hotkeys: q quit | c capture | r record")
    if args.gnssPort:
        if isinstance(gnss, GNSSReader):
            gnssInfoLine = f"[INFO] GNSS: reading NMEA from {gnss.port} @ {gnss.baud} baud"
            print(gnssInfoLine)
        else:
            gnssInfoLine = f"[WARN] GNSS: requested '{args.gnssPort}' but not available"
            print(gnssInfoLine)
    elif args.winLocation:
        if isinstance(gnss, WindowsLocationReader):
            print("[INFO] GNSS: using Windows Location API (enable Location in Settings)")
        else:
            print("[WARN] GNSS: Windows Location requested but not available")
    if args.scale:
        print("[INFO] Scale bar: enabled (requires --mpp or --hfov + altitude)")

    # ============================================================
    # MAIN LOOP - Read frames, draw overlays, handle hotkeys
    # ============================================================
    while True:
        success, frame = cameraCapture.read()
        if not success or frame is None:
            print("[ERROR] Failed to grab frame from camera")
            break

        # Update actual dimensions if the camera changed them
        if frame.shape[1] != actualWidth or frame.shape[0] != actualHeight:
            actualWidth = frame.shape[1]
            actualHeight = frame.shape[0]

        # ---- FPS calculation (exponential moving average) ----
        currentTime = time.perf_counter()
        deltaTime = currentTime - lastTime
        lastTime = currentTime
        instantFps = (1.0 / deltaTime) if deltaTime > 0 else 0.0
        fpsEma = instantFps if fpsEma == 0 else (alpha * instantFps + (1 - alpha) * fpsEma)

        # ---- Status overlays ----
        resolutionLine = f"{actualWidth}x{actualHeight} | FPS: {fpsEma:4.1f}"
        putOverlay(frame, resolutionLine)
        putOverlay(frame, "[c] Capture  [r] Record  [q] Quit", y=40)

        # ---- GNSS overlay ----
        gnssLat = None
        gnssLon = None
        gnssAlt = None
        hdop = None
        if gnss:
            if getattr(gnss, "error", None):
                gnssErrorLine = f"[ERROR] GNSS: {gnss.error}"
                putOverlay(frame, gnssErrorLine, y=60)
            else:
                gnssLat = gnss.latest.get("lat")
                gnssLon = gnss.latest.get("lon")
                gnssAlt = gnss.latest.get("alt")
                hdop = gnss.latest.get("hdop")
                if gnssLat is not None and gnssLon is not None:
                    latStr = f"{gnssLat:.6f}"
                    lonStr = f"{gnssLon:.6f}"
                    altStr = f" alt {gnssAlt:.1f} m" if gnssAlt is not None else ""
                    hdopStr = f" HDOP {hdop:.1f} m" if isinstance(hdop, (int, float)) else ""
                    gnssLine = f"GNSS: {latStr}, {lonStr}{altStr}{hdopStr}"
                    putOverlay(frame, gnssLine, y=60)
                else:
                    putOverlay(frame, "GNSS: searching...", y=60)
        else:
            if args.alt is not None:
                altLine = f"Alt: {args.alt:.1f} m (manual)"
            else:
                altLine = "GNSS: disabled (use --gnssPort)"
            putOverlay(frame, altLine, y=60)

        # ---- Scale bar overlay ----
        altMForScale = args.alt if args.alt is not None else (gnssAlt if gnssAlt is not None else None)
        mpp = estimateMpp(actualWidth, altMForScale, args.hfov, args.mpp)
        label = formatMppLabel(mpp)
        if label:
            putOverlay(frame, label, y=80)
            if args.scale:
                drawScaleBar(frame, mpp)
        else:
            putOverlay(frame, "Scale: set --mpp or --hfov + altitude", y=80)

        # ---- Auto-start recording if --record was passed ----
        if pendingRecordPath and videoWriter is None:
            outputPath = pendingRecordPath
            if not os.path.isabs(outputPath) and not outputPath.startswith(args.saveDir):
                if os.path.dirname(outputPath):
                    outputPath = os.path.join(args.saveDir, outputPath)
                else:
                    outputPath = os.path.join(videosDir, outputPath)
            ensureDir(os.path.dirname(outputPath) or ".")
            videoFps = nominalFps if nominalFps and nominalFps > 0 else max(1.0, fpsEma)
            videoWriter = cv2.VideoWriter(outputPath, fourcc, videoFps, (actualWidth, actualHeight))
            isRecording = videoWriter.isOpened()
            if isRecording:
                print(f"[INFO] Recording started --> {outputPath}")
            else:
                print(f"[ERROR] Failed to start recording --> {outputPath}")
            pendingRecordPath = None

        # Write current frame to video if recording
        if isRecording and videoWriter is not None and videoWriter.isOpened():
            videoWriter.write(frame)

        # ---- Show the frame and handle hotkeys ----
        cv2.imshow(windowName, frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        elif key == ord("c"):
            # ============================================================
            # CAPTURE PHOTO - save PNG + JSON sidecar
            # ============================================================
            timestamp = nowUtcIso()
            baseName = f"IMG_{timestamp}_{actualWidth}x{actualHeight}"
            imagePath = os.path.join(photosDir, baseName + ".png")
            cv2.imwrite(imagePath, frame)
            metadata = {
                "timestampUtc": timestamp,
                "cameraIndex": args.index,
                "frameWidth": actualWidth,
                "frameHeight": actualHeight,
                "requestedFps": args.fps,
                "measuredFps": round(fpsEma, 2),
                "exposure": cameraCapture.get(cv2.CAP_PROP_EXPOSURE),
                "gain": cameraCapture.get(cv2.CAP_PROP_GAIN),
                "codec": args.codec,
                "software": "Camera_System.py",
            }
            if gnss and not getattr(gnss, "error", None):
                if gnss.latest.get("lat") is not None and gnss.latest.get("lon") is not None:
                    metadata["gnss"] = {
                        "lat": gnss.latest.get("lat"),
                        "lon": gnss.latest.get("lon"),
                        "altMslM": gnss.latest.get("alt"),
                    }
                    if gnss.latest.get("hdop") is not None:
                        metadata["gnss"]["hdop"] = gnss.latest.get("hdop")
            if args.scale:
                altM = args.alt if args.alt is not None else (gnss.latest.get("alt") if gnss else None)
                mpp2 = estimateMpp(actualWidth, altM, args.hfov, args.mpp)
                if mpp2:
                    metadata["scale"] = {
                        "metersPerPixel": mpp2,
                        "altitudeM": altM,
                        "hfovDeg": args.hfov,
                    }
            sidecarMetadata(imagePath, metadata)
            print(f"[INFO] Saved still --> {imagePath}")

        elif key == ord("r"):
            # ============================================================
            # TOGGLE VIDEO RECORDING
            # ============================================================
            if isRecording and videoWriter is not None:
                videoWriter.release()
                isRecording = False
                print("[INFO] Recording stopped")
            else:
                timestamp = nowUtcIso()
                videoName = f"VID_{timestamp}_{actualWidth}x{actualHeight}.avi"
                pendingRecordPath = os.path.join(videosDir, videoName)

        # Check if the user closed the preview window with the X button
        if cv2.getWindowProperty(windowName, cv2.WND_PROP_VISIBLE) < 1:
            break

    # ============================================================
    # CLEANUP - Release camera, close windows, stop GNSS
    # ============================================================
    if videoWriter is not None and videoWriter.isOpened():
        videoWriter.release()
    cameraCapture.release()
    cv2.destroyAllWindows()
    print("[INFO] Camera released and windows closed successfully.")
    if gnss:
        gnss.stop()




#####################################################################################################################
# CLI Argument Parsing
#   - Builds the command-line argument parser with three subcommands:
#       (default) live preview, scan, ports
#   - argparse is a Python standard library that automatically generates help text
#     and validates user input from the command line
#   - The "scan" and "ports" subcommands have their own dedicated arguments
#   - Camera and GNSS arguments are on the main parser so they work without a subcommand
#
# Usage:
#   python Camera_System.py                              --> live preview (default)
#   python Camera_System.py --index 0 --width 640        --> live preview with options
#   python Camera_System.py scan                         --> scan for cameras
#   python Camera_System.py ports                        --> list serial ports
#   python Camera_System.py ports --probe                --> list ports and probe for GNSS
#####################################################################################################################

def buildParser():
    """Build the argument parser with subcommands for scan, ports, and live preview."""
    parser = argparse.ArgumentParser(
        description="LUSI Camera System - live preview, camera scan, and serial port listing",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    subparsers = parser.add_subparsers(dest="command")

    # Subcommand: scan
    scanParser = subparsers.add_parser("scan", help="Scan for connected cameras by index")
    scanParser.add_argument("--indices", type=int, default=5, help="Number of camera indices to scan (0 through N-1)")

    # Subcommand: ports
    portsParser = subparsers.add_parser("ports", help="List serial ports and optionally probe for GNSS/NMEA")
    portsParser.add_argument("--probe", action="store_true", help="Probe ports for NMEA ($..GGA/$..RMC)")
    portsParser.add_argument("--seconds", type=float, default=1.5, help="Probe duration per baud")
    portsParser.add_argument("--baud", type=int, nargs="*", default=[9600, 38400, 115200], help="Baud rates to try")

    # Default (live preview) arguments - added to the main parser
    parser.add_argument("--index", type=int, default=1, help="Camera index to open")
    parser.add_argument("--width", type=int, default=1280, help="Requested frame width")
    parser.add_argument("--height", type=int, default=720, help="Requested frame height")
    parser.add_argument("--fps", type=float, default=30.0, help="Requested FPS; 0 to skip setting FPS")
    parser.add_argument("--codec", type=str, default="MJPG", help="FourCC video codec for recording")
    parser.add_argument("--saveDir", type=str, default="Camera_Captures", help="Directory for saved images and videos")
    parser.add_argument("--record", type=str, default=None, help="Start recording to this file path")
    parser.add_argument("--noDshow", action="store_true", help="Do not force DirectShow backend on Windows")
    parser.add_argument("--exposure", type=float, default=None, help="Set exposure value if supported")
    parser.add_argument("--gain", type=float, default=None, help="Set gain value if supported")
    parser.add_argument("--gnssPort", type=str, default=None, help="Serial port for GNSS/NMEA (e.g., COM3, /dev/ttyUSB0 or 'auto')")
    parser.add_argument("--gnssBaud", type=int, default=9600, help="Baud rate for GNSS serial port")
    parser.add_argument("--winLocation", action="store_true", help="Use Windows Location API for coordinates (requires 'winrt')")
    parser.add_argument("--hfov", type=float, default=None, help="Camera horizontal FOV in degrees for scale bar")
    parser.add_argument("--alt", type=float, default=None, help="Fixed altitude above ground in meters (overrides GNSS altitude)")
    parser.add_argument("--mpp", type=float, default=None, help="Override meters-per-pixel directly (bypass FOV/alt calc)")
    parser.add_argument("--scale", action="store_true", help="Show scale bar overlay if MPP can be determined")

    return parser




#####################################################################################################################
# Program Entry Point
#   - Parses CLI arguments and dispatches to the appropriate mode
#   - No subcommand --> live preview (the default behavior)
#   - "scan" --> camera scan (checks which camera indices are accessible)
#   - "ports" --> serial port listing (with optional GNSS probing)
#####################################################################################################################

parser = buildParser()
args = parser.parse_args()

if args.command == "scan":
    listCameras(args.indices)
elif args.command == "ports":
    runPortsCommand(args)
else:
    runLivePreview(args)
