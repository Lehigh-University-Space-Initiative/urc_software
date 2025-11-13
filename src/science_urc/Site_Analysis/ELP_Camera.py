#####################################################################################################################
# Importing Program Libraries
#   Purpose
#   - Centralized imports for the entire module.
#   - Each import below is intentionally grouped and documented.
#
#   Contents
#   - argparse        : CLI parsing for configuration
#   - json            : JSON sidecar metadata for captures
#   - os              : Filesystem utilities
#   - time            : Timing and FPS measurement
#   - datetime        : ISO-8601 timestamps in UTC
#   - cv2             : OpenCV for capture, overlays, and recording
#   - math            : Geometry for scale estimates
#   - threading       : Background readers (GNSS, Windows Location)
#   - typing          : Optional typing hints
#####################################################################################################################

import argparse
import json
import os
import time
from datetime import datetime, timezone
import cv2
import math
import threading
from typing import Optional, Tuple










#####################################################################################################################
# Argument Parsing
#   Purpose
#   - Provide a consistent CLI to configure camera, overlays, GNSS, and scale.
#
#   Usage Examples
#   - python ELP_Camera.py --index 1 --width 1280 --height 720 --gnssPort auto --hfov 70 --scale
#   - python ELP_Camera.py --winLocation --hfov 70 --alt 2.0 --scale
#
# ------- OPTIONS -------
#SUBSUBSECTION
#   Index/Resolution/FPS      : --index, --width, --height, --fps
#   Recording                 : --codec, --saveDir, --record
#   Camera Controls           : --noDshow, --exposure, --gain
#   GNSS                      : --gnssPort, --gnssBaud, --winLocation
#   Scale                     : --hfov, --alt, --mpp, --scale
#####################################################################################################################

def parseArgs():
    # ------- PARSER_SETUP -------
    parser = argparse.ArgumentParser(
        description="ELP camera live preview with capture/record overlays",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # ------- CAMERA_PARAMS -------
    parser.add_argument("--index", type=int, default=1, help="Camera index to open")
    parser.add_argument("--width", type=int, default=1280, help="Requested frame width")
    parser.add_argument("--height", type=int, default=720, help="Requested frame height")
    parser.add_argument("--fps", type=float, default=30.0, help="Requested FPS; 0 to skip setting FPS")
    parser.add_argument("--codec", type=str, default="MJPG", help="FourCC video codec for recording")
    parser.add_argument("--saveDir", type=str, default="captures", help="Directory for saved images and videos")
    parser.add_argument("--record", type=str, default=None, help="Start recording to this file path")
    parser.add_argument("--noDshow", action="store_true", help="Do not force DirectShow backend on Windows")
    parser.add_argument("--exposure", type=float, default=None, help="Set exposure value if supported")
    parser.add_argument("--gain", type=float, default=None, help="Set gain value if supported")

    # ------- GNSS_AND_SCALE -------
    parser.add_argument("--gnssPort", type=str, default=None, help="Serial port for GNSS/NMEA (e.g., COM3, /dev/ttyUSB0 or 'auto')")
    parser.add_argument("--gnssBaud", type=int, default=9600, help="Baud rate for GNSS serial port")
    parser.add_argument("--winLocation", action="store_true", help="Use Windows Location API for coordinates (requires 'winrt')")
    parser.add_argument("--hfov", type=float, default=None, help="Camera horizontal FOV in degrees for scale bar")
    parser.add_argument("--alt", type=float, default=None, help="Fixed altitude above ground in meters (overrides GNSS altitude)")
    parser.add_argument("--mpp", type=float, default=None, help="Override meters-per-pixel directly (bypass FOV/alt calc)")
    parser.add_argument("--scale", action="store_true", help="Show scale bar overlay if MPP can be determined")

    return parser.parse_args()











#####################################################################################################################
# Camera Initialization
#   Purpose
#   - Robustly open the camera by index with optional DirectShow on Windows.
#
# ------- SUBSYSTEM -------
#SUBSUBSECTION
#   openCamera(index, useDshow)
#####################################################################################################################

def openCamera(index: int, useDshow: bool):
    backend = cv2.CAP_DSHOW if (os.name == "nt" and useDshow) else 0
    cameraCapture = cv2.VideoCapture(index, backend)

    if not cameraCapture.isOpened() and backend == cv2.CAP_DSHOW:
        cameraCapture = cv2.VideoCapture(index)

    return cameraCapture










#####################################################################################################################
# Filesystem Utilities
#   Purpose
#   - Ensure directories exist for photos and videos.
#
# ------- SUBSECTION: ensureDir -------
#SUBSUBSECTION
#####################################################################################################################

def ensureDir(path: str):
    os.makedirs(path, exist_ok=True)










#####################################################################################################################
# Overlay Utilities
#   Purpose
#   - Consistent on-frame text rendering for status and guidance.
#
# ------- SUBSECTION: putOverlay -------
#SUBSUBSECTION
#####################################################################################################################

def putOverlay(frame, text, y=20):
    cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)










#####################################################################################################################
# GNSS Utilities
#   Purpose
#   - Lightweight NMEA parsing (GGA/RMC) with no external parser.
#   - GNSSReader thread pulls latest fix for overlay/metadata.
#   - Auto-detection helper scans COM ports for NMEA talkers.
#   - Optional Windows Location reader for non-COM GPS.
#
# ------- Subsections -------
#SUBSUBSECTION
#   _nmea_deg, _parse_gga, _parse_rmc
#   GNSSReader (serial NMEA)
#   autodetect_gnss_port()
#   WindowsLocationReader (winrt)
#####################################################################################################################

def _nmea_deg(value: str, hemi: str) -> Optional[float]:
    # ------- NMEA_DEGREES -------
    # Convert NMEA ddmm.mmmm / dddmm.mmmm + hemisphere to signed decimal degrees.
    try:
        if not value or not hemi:
            return None
        if "." not in value:
            return None
        idx = value.find(".")
        deg_len = 2
        if len(value[:idx]) >= 5:  # longitude path (3-digit degrees)
            deg_len = 3
        deg = float(value[:deg_len])
        minutes = float(value[deg_len:])
        dec = deg + minutes / 60.0
        if hemi in ("S", "W"):
            dec = -dec
        return dec
    except Exception:
        return None


def _parse_gga(fields: list) -> dict:
    # ------- PARSE_GGA -------
    # $GxGGA,time,lat,N,lon,E,quality,numSV,HDOP,alt,M,sep,M,diffAge,diffStation*cs
    data = {}
    try:
        data["lat"] = _nmea_deg(fields[2], fields[3]) if len(fields) > 4 else None
        data["lon"] = _nmea_deg(fields[4], fields[5]) if len(fields) > 6 else None
        data["fix_quality"] = int(fields[6]) if len(fields) > 6 and fields[6].isdigit() else None
        data["satellites"] = int(fields[7]) if len(fields) > 7 and fields[7].isdigit() else None
        data["hdop"] = float(fields[8]) if len(fields) > 8 and fields[8] != "" else None
        data["alt"] = float(fields[9]) if len(fields) > 9 and fields[9] != "" else None
    except Exception:
        pass
    return data


def _parse_rmc(fields: list) -> dict:
    # ------- PARSE_RMC -------
    # $GxRMC,time,status,lat,N,lon,E,sog,cog,date,...
    data = {}
    try:
        data["lat"] = _nmea_deg(fields[3], fields[4]) if len(fields) > 5 else None
        data["lon"] = _nmea_deg(fields[5], fields[6]) if len(fields) > 7 else None
        if len(fields) > 7 and fields[7] != "":
            sog_kn = float(fields[7])
            data["speed_ms"] = sog_kn * 0.514444
            data["speed_kmh"] = sog_kn * 1.852
    except Exception:
        pass
    return data

class GNSSReader:
    # ------- GNSS_SERIAL_READER -------
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
            "fix_quality": None,
            "satellites": None,
            "speed_kmh": None,
        }
        self.error: Optional[str] = None

    def start(self):
        if self.thread and self.thread.is_alive():
            return
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self._stop.set()
        if self.thread:
            self.thread.join(timeout=0.5)

    def _run(self):
        try:
            try:
                import serial  # type: ignore
            except Exception:
                self.error = "pyserial not available"
                return
            ser = serial.Serial(self.port, self.baud, timeout=1)
            with ser:
                while not self._stop.is_set():
                    try:
                        line = ser.readline().decode(errors="ignore").strip()
                        if not line or not line.startswith("$"):
                            continue
                        if "*" in line:
                            line = line.split("*")[0]
                        parts = line.split(",")
                        talker = parts[0][3:6] if len(parts[0]) >= 6 else parts[0][3:]
                        if talker == "GGA":
                            data = _parse_gga(parts)
                            self._merge(data)
                        elif talker == "RMC":
                            data = _parse_rmc(parts)
                            self._merge(data)
                    except Exception:
                        continue
        except Exception as e:
            self.error = str(e)

    def _merge(self, data: dict):
        for k, v in data.items():
            if v is not None:
                self.latest[k] = v











#####################################################################################################################
# GNSS Auto-detection
#   Purpose
#   - Iterate serial ports and test common baud rates for NMEA talkers.
#
# ------- SUBSECTION: autodetect_gnss_port -------
#SUBSUBSECTION
#####################################################################################################################

def autodetect_gnss_port(bauds=(9600, 38400, 115200), seconds: float = 1.5):
    try:
        import time as _t
        import serial  # type: ignore
        from serial.tools import list_ports  # type: ignore
    except Exception:
        return None

    preferred_keywords = (
        "u-blox", "ublox", "GNSS", "GPS", "Quectel", "SimCom",
        "USB-SERIAL", "USB Serial", "Prolific", "CH340", "Silicon Labs",
    )
    ports = list(list_ports.comports())

    def score(p):
        desc = (p.description or "").lower()
        return 0 if not desc else -sum(1 for k in preferred_keywords if k.lower() in desc)

    ports_sorted = sorted(ports, key=score)

    for p in ports_sorted:
        dev = p.device
        for b in bauds:
            try:
                ser = serial.Serial(dev, b, timeout=0.2)
            except Exception:
                continue
            with ser:
                t0 = _t.perf_counter()
                while (_t.perf_counter() - t0) < seconds:
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
# Windows Location API Reader
#   Purpose
#   - Provide coordinates on Windows when no COM/NMEA device is present.
#   - Uses winrt package (optional) and Geolocator.
#
# ------- SUBSECTION: WindowsLocationReader -------
#SUBSUBSECTION
#####################################################################################################################

class WindowsLocationReader:
    def __init__(self, interval_sec: float = 1.0):
        self.interval = interval_sec
        self.thread = None
        self._stop = threading.Event()
        self.latest = {
            "lat": None,
            "lon": None,
            "alt": None,
            "speed_kmh": None,
        }
        self.error: Optional[str] = None

    def start(self):
        if self.thread and self.thread.is_alive():
            return
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self._stop.set()
        if self.thread:
            self.thread.join(timeout=0.5)

    def _run(self):
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

        async def _fetch_once():
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
                speed_kmh = None
                try:
                    if coord.speed is not None:
                        speed_kmh = float(coord.speed) * 3.6
                except Exception:
                    pass
                return {"lat": lat, "lon": lon, "alt": alt, "speed_kmh": speed_kmh}
            except Exception as e:
                return {"error": str(e)}

        while not self._stop.is_set():
            try:
                result = asyncio.run(_fetch_once())
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
# Scale Bar Utilities
#   Purpose
#   - Compute meters-per-pixel (MPP) via altitude and HFOV.
#   - Draw a visually balanced scale bar and textual label.
#
# ------- SUBSECTIONS -------
#SUBSUBSECTION
#   estimate_mpp(width_px, altitude_m, hfov_deg, override_mpp)
#   draw_scale_bar(frame, mpp)
#   format_mpp_label(mpp)
#####################################################################################################################

def estimate_mpp(width_px: int, altitude_m: Optional[float], hfov_deg: Optional[float], override_mpp: Optional[float]) -> Optional[float]:
    if override_mpp and override_mpp > 0:
        return override_mpp
    if altitude_m is None or hfov_deg is None or width_px <= 0:
        return None
    try:
        width_m = 2.0 * altitude_m * math.tan(math.radians(hfov_deg) / 2.0)
        if width_m <= 0:
            return None
        return width_m / float(width_px)
    except Exception:
        return None


def draw_scale_bar(frame, mpp: float):
    if not mpp or mpp <= 0:
        return
    h, w = frame.shape[:2]
    target_px = max(60, w // 5)
    target_m = target_px * mpp
    steps = [
        0.1, 0.2, 0.5,
        1, 2, 5,
        10, 20, 50,
        100, 200, 500,
        1000, 2000, 5000,
    ]
    best_m = steps[-1]
    for s in steps:
        if s >= target_m:
            best_m = s
            break
    bar_px = int(round(best_m / mpp))
    margin = 16
    y = h - margin
    x0 = margin
    x1 = x0 + bar_px
    pad = 6
    box_x0 = x0 - pad
    box_y0 = y - 22 - pad
    box_x1 = x1 + pad
    box_y1 = y + pad
    cv2.rectangle(frame, (box_x0, box_y0), (box_x1, box_y1), (0, 0, 0), thickness=-1)
    cv2.rectangle(frame, (box_x0, box_y0), (box_x1, box_y1), (255, 255, 255), thickness=1)
    cv2.line(frame, (x0, y), (x1, y), (255, 255, 255), 2)
    cv2.line(frame, (x0, y - 8), (x0, y + 8), (255, 255, 255), 2)
    cv2.line(frame, (x1, y - 8), (x1, y + 8), (255, 255, 255), 2)
    label = f"{best_m:g} m"
    cv2.putText(frame, label, (x0, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


def format_mpp_label(mpp: Optional[float]) -> Optional[str]:
    if mpp is None or mpp <= 0:
        return None
    if mpp < 0.01:
        return f"Scale: {mpp*1000:.1f} mm/px"
    if mpp < 1.0:
        return f"Scale: {mpp*100:.2f} cm/px"
    return f"Scale: {mpp:.3f} m/px"











#####################################################################################################################
# Timestamp and Metadata Utilities
#   Purpose
#   - Timestamps for filenames; JSON sidecar for capture metadata.
#
# ------- SUBSECTIONS -------
#SUBSUBSECTION
#   nowUtcIso()
#   sidecarMetadata(pathPng, metadata)
#####################################################################################################################

def nowUtcIso():
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%S.%fZ")


def sidecarMetadata(pathPng: str, metadata: dict):
    metadataPath = os.path.splitext(pathPng)[0] + ".json"
    with open(metadataPath, "w", encoding="utf-8") as file:
        json.dump(metadata, file, indent=2)










#####################################################################################################################
# Main Execution Loop
#   Purpose
#   - Live preview, overlays (status, GNSS, scale), still capture, and recording.
#
# ------- SUBSECTIONS -------
#SUBSUBSECTION
#   Setup (dirs, camera, GNSS)
#   Loop (read, FPS, overlays, record/capture)
#   Cleanup
#####################################################################################################################

def main():
    # ------- SETUP: CLI & DIRS -------
    args = parseArgs()
    ensureDir(args.saveDir)
    photosDir = os.path.join(args.saveDir, "photos")
    videosDir = os.path.join(args.saveDir, "videos")
    ensureDir(photosDir)
    ensureDir(videosDir)

    # ------- SETUP: CAMERA -------
    cameraCapture = openCamera(args.index, useDshow=not args.noDshow)
    if not cameraCapture.isOpened():
        print("Camera not detected. Try a different index or check the USB connection.")
        return

    cameraCapture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
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

    # ------- SETUP: RECORDING -------
    fourcc = cv2.VideoWriter_fourcc(*args.codec)
    isRecording = False
    videoWriter = None
    pendingRecordPath = args.record if args.record else None

    # ------- SETUP: GNSS -------
    showGrid = False
    showHist = False
    gnss = None
    if args.gnssPort:
        port = args.gnssPort
        baud = args.gnssBaud
        if isinstance(port, str) and port.lower() == "auto":
            bauds = (baud,) if (isinstance(baud, (int, float)) and baud and baud > 0) else (9600, 38400, 115200)
            detected = autodetect_gnss_port(bauds=bauds, seconds=2.0)
            if detected:
                port, baud = detected
                print(f"GNSS auto-detected: {port} @ {baud} baud")
            else:
                print("GNSS auto-detect failed: no NMEA found on available ports")
                port = None
        if port:
            gnss = GNSSReader(port, int(baud))
            gnss.start()
    elif args.winLocation:
        gnss = WindowsLocationReader(interval_sec=1.0)
        gnss.start()

    windowName = "ELP USB Camera"
    cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)

    # ------- SETUP: TIMING -------
    fpsEma = 0.0
    alpha = 0.2
    lastTime = time.perf_counter()

    print("Hotkeys: q quit | c capture | r record")
    if args.gnssPort:
        if isinstance(gnss, GNSSReader):
            print(f"GNSS: reading NMEA from {gnss.port} @ {gnss.baud} baud")
        else:
            print(f"GNSS: requested '{args.gnssPort}' but not available")
    elif args.winLocation:
        if isinstance(gnss, WindowsLocationReader):
            print("GNSS: using Windows Location API (enable Location in Settings)")
        else:
            print("GNSS: Windows Location requested but not available")
    if args.scale:
        print("Scale bar: enabled (requires --mpp or --hfov + altitude)")

    # ------- MAIN LOOP -------
    while True:
        success, frame = cameraCapture.read()
        if not success or frame is None:
            print("Failed to grab frame from camera")
            break

        if frame.shape[1] != actualWidth or frame.shape[0] != actualHeight:
            actualWidth = frame.shape[1]
            actualHeight = frame.shape[0]

        # FPS EMA
        currentTime = time.perf_counter()
        deltaTime = currentTime - lastTime
        lastTime = currentTime
        instantFps = (1.0 / deltaTime) if deltaTime > 0 else 0.0
        fpsEma = instantFps if fpsEma == 0 else (alpha * instantFps + (1 - alpha) * fpsEma)

        # ------- OVERLAYS -------
        putOverlay(frame, f"{actualWidth}x{actualHeight} | FPS: {fpsEma:4.1f}")
        putOverlay(frame, "[c] Capture  [r] Record  [q] Quit", y=40)

        # GNSS overlay
        gnss_lat = gnss_lon = gnss_alt = hdop = None
        if gnss:
            if getattr(gnss, "error", None):
                putOverlay(frame, f"GNSS error: {gnss.error}", y=60)
            else:
                gnss_lat = gnss.latest.get("lat")
                gnss_lon = gnss.latest.get("lon")
                gnss_alt = gnss.latest.get("alt")
                hdop = gnss.latest.get("hdop")
                if gnss_lat is not None and gnss_lon is not None:
                    lat_s = f"{gnss_lat:.6f}"
                    lon_s = f"{gnss_lon:.6f}"
                    alt_s = f" alt {gnss_alt:.1f} m" if gnss_alt is not None else ""
                    hdop_s = f" HDOP {hdop:.1f} m" if isinstance(hdop, (int, float)) else ""
                    putOverlay(frame, f"GNSS: {lat_s}, {lon_s}{alt_s}{hdop_s}", y=60)
                else:
                    putOverlay(frame, "GNSS: searching...", y=60)
        else:
            msg = f"Alt: {args.alt:.1f} m (manual)" if args.alt is not None else "GNSS: disabled (use --gnssPort)"
            putOverlay(frame, msg, y=60)

        # Scale indicators (MPP text always if computable; bar when --scale)
        alt_m_for_scale = args.alt if args.alt is not None else (gnss_alt if gnss_alt is not None else None)
        mpp = estimate_mpp(actualWidth, alt_m_for_scale, args.hfov, args.mpp)
        label = format_mpp_label(mpp)
        if label:
            putOverlay(frame, label, y=80)
            if args.scale:
                draw_scale_bar(frame, mpp)
        else:
            putOverlay(frame, "Scale: set --mpp or --hfov + altitude", y=80)

        # ------- RECORDING -------
        if pendingRecordPath and videoWriter is None:
            outputPath = pendingRecordPath
            if not os.path.isabs(outputPath):
                if os.path.dirname(outputPath):
                    outputPath = os.path.join(args.saveDir, outputPath)
                else:
                    outputPath = os.path.join(videosDir, outputPath)
            ensureDir(os.path.dirname(outputPath) or ".")
            videoFps = nominalFps if nominalFps and nominalFps > 0 else max(1.0, fpsEma)
            videoWriter = cv2.VideoWriter(outputPath, fourcc, videoFps, (actualWidth, actualHeight))
            isRecording = videoWriter.isOpened()
            print(f"Recording started -> {outputPath}" if isRecording else "Failed to start recording")
            pendingRecordPath = None

        if isRecording and videoWriter is not None and videoWriter.isOpened():
            videoWriter.write(frame)

        # ------- INPUT -------
        cv2.imshow(windowName, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("c"):
            timestamp = nowUtcIso()
            baseName = f"IMG_{timestamp}_{actualWidth}x{actualHeight}"
            imagePath = os.path.join(photosDir, baseName + ".png")
            cv2.imwrite(imagePath, frame)
            metadata = {
                "timestamp_utc": timestamp,
                "camera_index": args.index,
                "frame_width": actualWidth,
                "frame_height": actualHeight,
                "requested_fps": args.fps,
                "measured_fps": round(fpsEma, 2),
                "exposure": cameraCapture.get(cv2.CAP_PROP_EXPOSURE),
                "gain": cameraCapture.get(cv2.CAP_PROP_GAIN),
                "codec": args.codec,
                "software": "ELP_Camera.py",
            }
            if gnss and not getattr(gnss, "error", None):
                if gnss.latest.get("lat") is not None and gnss.latest.get("lon") is not None:
                    metadata["gnss"] = {
                        "lat": gnss.latest.get("lat"),
                        "lon": gnss.latest.get("lon"),
                        "alt_msl_m": gnss.latest.get("alt"),
                    }
                    if gnss.latest.get("hdop") is not None:
                        metadata["gnss"]["hdop"] = gnss.latest.get("hdop")
            if args.scale:
                alt_m = args.alt if args.alt is not None else (gnss.latest.get("alt") if gnss else None)
                mpp2 = estimate_mpp(actualWidth, alt_m, args.hfov, args.mpp)
                if mpp2:
                    metadata["scale"] = {
                        "meters_per_pixel": mpp2,
                        "altitude_m": alt_m,
                        "hfov_deg": args.hfov,
                    }
            sidecarMetadata(imagePath, metadata)
            print(f"Saved still -> {imagePath}")
        elif key == ord("r"):
            if isRecording and videoWriter is not None:
                videoWriter.release()
                isRecording = False
                print("Recording stopped")
            else:
                timestamp = nowUtcIso()
                videoName = f"VID_{timestamp}_{actualWidth}x{actualHeight}.avi"
                pendingRecordPath = os.path.join(videosDir, videoName)

        if cv2.getWindowProperty(windowName, cv2.WND_PROP_VISIBLE) < 1:
            break

    # ------- CLEANUP -------
    if videoWriter is not None and videoWriter.isOpened():
        videoWriter.release()
    cameraCapture.release()
    cv2.destroyAllWindows()
    print("Camera released and windows closed successfully.")
    if gnss:
        gnss.stop()











#####################################################################################################################
# Program Entry Point
#   Purpose
#   - Execute main() only when the script is run directly.
#
# ------- SUBSECTION: __main__ -------
#SUBSUBSECTION
#####################################################################################################################

if __name__ == "__main__":
    main()