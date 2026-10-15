#!/usr/bin/env python3
# This shebang line tells Linux and macOS to use Python 3 when the script is
# run directly from the terminal (e.g., ./Spectrometer.py). On Windows,
# this line is ignored - you run scripts with "python Spectrometer.py" instead.


#####################################################################################################################
# LUSI Science Module - USB Spectrometer
#
# This script captures spectral data from a USB camera-based diffraction grating spectrometer and displays
# a real-time wavelength vs. intensity graph with peak detection, calibration, and data export capabilities.
#
# Based on PySpectrometer2 by Les Wright (https://github.com/leswright1977/PySpectrometer2), adapted for
# the LUSI Science Module project with integrated calibration, Savitzky-Golay filtering, peak labeling,
# and CSV/PNG data export.
#
# Hardware setup:
#   - A USB camera pointed at a diffraction grating spectroscope
#   - Plug the camera into a blue USB 3.0 port on the Pi 4 for best bandwidth
#     (black USB 2.0 ports also work but may limit frame rate)
#   - The camera captures the dispersed spectrum as a horizontal band of color
#   - Software reads pixel intensities along the center row and maps pixel positions to wavelengths
#   - Target absorbance wavelength: 440 nm (for carbonato-cobaltate (III) detection)
#
# How it works:
#   1. The camera captures a frame and crops a horizontal band from the center
#   2. The band is converted to grayscale and 3 rows of pixels are averaged for noise reduction
#   3. Pixel positions are mapped to wavelengths using polynomial calibration data
#   4. Intensity vs. wavelength is plotted in real time with labeled peaks
#   5. Calibration is done interactively by identifying known emission lines (e.g., from a fluorescent lamp)
#
# Key bindings:
#   h           --> Toggle peak hold (freezes the highest intensities seen)
#   m           --> Toggle measurement cursor (shows wavelength at mouse position)
#   p           --> Toggle pixel recording mode (select calibration points by clicking peaks)
#   c           --> Run calibration routine (enter known wavelengths for selected pixels in the terminal)
#   x           --> Clear selected calibration points
#   s           --> Save spectrum graph as PNG and intensity data as CSV
#   o / l       --> Increase / decrease Savitzky-Golay filter polynomial order
#   i / k       --> Increase / decrease minimum peak distance
#   u / j       --> Increase / decrease peak label threshold
#   q           --> Quit the program
#
# Command-line options:
#   --device N  --> USB camera device number (default: 0, find with v4l2-ctl --list-devices)
#   --fps N     --> Camera frame rate (default: 30)
#   --fullscreen --> Run the graph in fullscreen mode (800x480)
#   --waterfall --> Enable the waterfall display (shows spectral changes over time)
#####################################################################################################################




#####################################################################################################################
# Importing Program Libraries
#   - cv2 (OpenCV):
#       - Captures video frames from the USB camera
#       - Draws the spectrum graph, graticule, peak labels, and measurement cursors
#       - Handles keyboard input and mouse events
#   - numpy:
#       - Performs fast array math on pixel intensity data
#       - Used for polynomial fitting during calibration
#       - Correlation coefficient calculation for calibration accuracy (R-squared)
#   - time:
#       - Timestamps for saved files (e.g., "Spectrum-20260226--153000.csv")
#   - argparse:
#       - Parses command-line options (--device, --fps, --fullscreen, --waterfall)
#   - math:
#       - factorial() used by the Savitzky-Golay smoothing filter
#   - os:
#       - File path operations for calibration data storage
#####################################################################################################################

import cv2
import numpy as np
import time
import argparse
from math import factorial
import os
import signal
import glob




#####################################################################################################################
# Frame and Display Constants
#   - FRAME_WIDTH / FRAME_HEIGHT: Resolution requested from the USB camera (must be 800x600)
#   - GRAPH_HEIGHT: Height in pixels of the intensity graph
#   - BANNER_HEIGHT: Height of the status message banner at the top
#   - PREVIEW_HEIGHT: Height of the camera preview strip
#   - STACK_HEIGHT: Total height of the combined display (banner + preview + graph)
#####################################################################################################################

FRAME_WIDTH = 800
FRAME_HEIGHT = 600
GRAPH_HEIGHT = 320
BANNER_HEIGHT = 80
PREVIEW_HEIGHT = 80
STACK_HEIGHT = GRAPH_HEIGHT + BANNER_HEIGHT + PREVIEW_HEIGHT




#####################################################################################################################
# Default Signal Processing Settings
#   - DEFAULT_SAVPOLY: Starting polynomial order for the Savitzky-Golay smoothing filter (1-15)
#   - DEFAULT_MINDIST: Minimum pixel distance between detected peaks (1-100)
#   - DEFAULT_THRESH: Intensity threshold for peak labeling (1-100)
#####################################################################################################################

DEFAULT_SAVPOLY = 7
DEFAULT_MINDIST = 50
DEFAULT_THRESH = 20




#####################################################################################################################
# Calibration File Path
#   - Calibration data is stored in a text file so you only need to calibrate once
#   - Line 1: comma-separated pixel positions of known emission lines
#   - Line 2: comma-separated wavelengths (nm) matching those pixel positions
#   - The file is saved in the same directory as this script
#####################################################################################################################

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CAL_DATA_FILE = os.path.join(SCRIPT_DIR, "caldata.txt")




#####################################################################################################################
# Helper Function - Convert Wavelength (nm) to RGB Color
#
#   Parameters:
#       nm (float) - wavelength in nanometers
#
#   Returns:
#       tuple (R, G, B) - color values 0-255
#
#   How it works:
#       - Maps the visible spectrum (380-780 nm) to RGB values using piecewise linear interpolation
#       - Applies a gamma correction (0.8) for perceptual brightness
#       - Wavelengths outside the visible range are shown as gray (155, 155, 155)
#       - Based on the algorithm by Chris Webb (codedrome.com)
#####################################################################################################################

def wavelengthToRgb(nm):
    """Convert a wavelength in nm to an (R, G, B) tuple."""
    gamma = 0.8
    maxIntensity = 255
    factor = 0
    r = 0.0
    g = 0.0
    b = 0.0

    if 380 <= nm <= 439:
        r = -(nm - 440) / (440 - 380)
        g = 0.0
        b = 1.0
    elif 440 <= nm <= 489:
        r = 0.0
        g = (nm - 440) / (490 - 440)
        b = 1.0
    elif 490 <= nm <= 509:
        r = 0.0
        g = 1.0
        b = -(nm - 510) / (510 - 490)
    elif 510 <= nm <= 579:
        r = (nm - 510) / (580 - 510)
        g = 1.0
        b = 0.0
    elif 580 <= nm <= 644:
        r = 1.0
        g = -(nm - 645) / (645 - 580)
        b = 0.0
    elif 645 <= nm <= 780:
        r = 1.0
        g = 0.0
        b = 0.0

    if 380 <= nm <= 419:
        factor = 0.3 + 0.7 * (nm - 380) / (420 - 380)
    elif 420 <= nm <= 700:
        factor = 1.0
    elif 701 <= nm <= 780:
        factor = 0.3 + 0.7 * (780 - nm) / (780 - 700)

    rOut = int(maxIntensity * ((r * factor) ** gamma)) if r > 0 else 0
    gOut = int(maxIntensity * ((g * factor) ** gamma)) if g > 0 else 0
    bOut = int(maxIntensity * ((b * factor) ** gamma)) if b > 0 else 0

    # Display wavelengths outside visible range as gray
    if (rOut + gOut + bOut) == 0:
        rOut = 155
        gOut = 155
        bOut = 155

    return (rOut, gOut, bOut)




#####################################################################################################################
# Helper Function - Savitzky-Golay Smoothing Filter
#
#   Parameters:
#       y (array)       - the raw intensity data to smooth
#       windowSize (int) - number of points in the smoothing window (must be odd and positive)
#       order (int)      - polynomial order for the local fit (must be less than windowSize)
#       deriv (int)      - derivative order (0 = just smooth, no derivative)
#       rate (int)       - sample spacing (usually 1)
#
#   Returns:
#       array - smoothed intensity data, same length as input
#
#   How it works:
#       - Fits a polynomial of the given order to each sliding window of data points
#       - Replaces the center point with the fitted value
#       - Reduces high-frequency noise while preserving peak shapes and widths
#       - The larger the window, the more smoothing (but peaks may get flattened)
#       - This is the standard SciPy Savitzky-Golay implementation
#
#   Based on the SciPy Cookbook implementation:
#       Copyright (c) 2001-2002 Enthought, Inc. 2003-2022, SciPy Developers. BSD License.
#####################################################################################################################

def savitzkyGolay(y, windowSize, order, deriv=0, rate=1):
    """Apply a Savitzky-Golay smoothing filter to the intensity data."""
    windowSize = int(np.abs(np.int32(windowSize)))
    order = int(np.abs(np.int32(order)))

    if windowSize % 2 != 1 or windowSize < 1:
        raise TypeError("windowSize must be a positive odd number")
    if windowSize < order + 2:
        raise TypeError("windowSize is too small for the polynomial order")

    orderRange = range(order + 1)
    halfWindow = (windowSize - 1) // 2

    # Precompute the filter coefficients using a least-squares polynomial fit
    b = np.asmatrix([[k ** i for i in orderRange] for k in range(-halfWindow, halfWindow + 1)])
    m = np.linalg.pinv(b).A[deriv] * rate ** deriv * factorial(deriv)

    # Pad the signal at the edges by reflecting values to avoid boundary artifacts
    firstVals = y[0] - np.abs(y[1:halfWindow + 1][::-1] - y[0])
    lastVals = y[-1] + np.abs(y[-halfWindow - 1:-1][::-1] - y[-1])
    y = np.concatenate((firstVals, y, lastVals))

    return np.convolve(m[::-1], y, mode="valid")




#####################################################################################################################
# Helper Function - Detect Peaks in Intensity Data
#
#   Parameters:
#       y (array)          - intensity data to search for peaks
#       thres (float)      - threshold as fraction of (max - min) intensity; peaks below this are ignored
#       minDist (int)      - minimum distance in pixels between detected peaks
#       thresAbs (bool)    - if True, treat thres as an absolute value instead of a fraction
#
#   Returns:
#       array - indices (pixel positions) of detected peaks
#
#   How it works:
#       - Computes the first-order difference of the intensity data
#       - A peak occurs where the difference goes from positive to negative (local maximum)
#       - Plateaus (runs of zero difference) are handled by propagating neighboring values
#       - After finding all peaks, removes duplicates that are closer than minDist apart,
#         keeping the tallest peak in each neighborhood
#
#   Based on peakutils by Lucas Hermann Negri (MIT License).
#####################################################################################################################

def peakIndexes(y, thres=0.3, minDist=1, thresAbs=False):
    """Find peak indices in the intensity data."""
    if isinstance(y, np.ndarray) and np.issubdtype(y.dtype, np.unsignedinteger):
        raise ValueError("y must be signed")

    if not thresAbs:
        thres = thres * (np.max(y) - np.min(y)) + np.min(y)

    minDist = int(minDist)

    # Compute first order difference
    dy = np.diff(y)

    # Handle plateau pixels (where diff == 0) by propagating neighbor values
    zeros, = np.where(dy == 0)

    # Check if the signal is totally flat
    if len(zeros) == len(y) - 1:
        return np.array([])

    if len(zeros):
        zerosDiff = np.diff(zeros)
        zerosDiffNotOne, = np.add(np.where(zerosDiff != 1), 1)
        zeroPlateaus = np.split(zeros, zerosDiffNotOne)

        # Fix if leftmost value in dy is zero
        if zeroPlateaus[0][0] == 0:
            dy[zeroPlateaus[0]] = dy[zeroPlateaus[0][-1] + 1]
            zeroPlateaus.pop(0)

        # Fix if rightmost value of dy is zero
        if len(zeroPlateaus) and zeroPlateaus[-1][-1] == len(dy) - 1:
            dy[zeroPlateaus[-1]] = dy[zeroPlateaus[-1][0] - 1]
            zeroPlateaus.pop(-1)

        # For each chain of zero indexes, propagate values from the edges
        for plateau in zeroPlateaus:
            median = np.median(plateau)
            dy[plateau[plateau < median]] = dy[plateau[0] - 1]
            dy[plateau[plateau >= median]] = dy[plateau[-1] + 1]

    # Find peaks: points where the first-order difference changes from positive to negative
    peaks = np.where(
        (np.hstack([dy, 0.0]) < 0.0)
        & (np.hstack([0.0, dy]) > 0.0)
        & (np.greater(y, thres))
    )[0]

    # Enforce minimum distance between peaks - keep the tallest ones
    if peaks.size > 1 and minDist > 1:
        highest = peaks[np.argsort(y[peaks])][::-1]
        rem = np.ones(y.size, dtype=bool)
        rem[peaks] = False

        for peak in highest:
            if not rem[peak]:
                sl = slice(max(0, peak - minDist), peak + minDist + 1)
                rem[sl] = True
                rem[peak] = False

        peaks = np.arange(y.size)[~rem]

    return peaks




#####################################################################################################################
# Calibration Function - Read Calibration Data from File
#
#   Parameters:
#       width (int) - the frame width in pixels (determines how many wavelengths to compute)
#
#   Returns:
#       list - [wavelengthData, calMessage1, calMessage2, calMessage3]
#           wavelengthData: array of wavelengths (nm), one per pixel column
#           calMessage1-3: status strings for the display banner
#
#   How it works:
#       1. Reads pixel positions and wavelengths from caldata.txt
#       2. If 3 calibration points: fits a 2nd-order polynomial (reasonably accurate)
#       3. If 4+ calibration points: fits a 3rd-order polynomial (very accurate)
#       4. Uses the polynomial to compute the wavelength for every pixel position
#       5. Calculates R-squared to show how well the fit matches the input data
#       6. If no file is found, loads placeholder data (380-750 nm linear mapping)
#####################################################################################################################

def readCalibration(width):
    """Read calibration data from caldata.txt and compute wavelength-per-pixel mapping."""
    errors = 0
    message = 0
    pixels = []
    wavelengths = []

    try:
        print("[INFO] Loading calibration data from caldata.txt...")
        with open(CAL_DATA_FILE, "r") as f:
            lines = f.readlines()
            line0 = lines[0].strip()
            pixels = [int(i) for i in line0.split(",")]
            line1 = lines[1].strip()
            wavelengths = [float(i) for i in line1.split(",")]
    except Exception:
        errors = 1

    try:
        if len(pixels) != len(wavelengths):
            errors = 1
        if len(pixels) < 3:
            errors = 1
        if len(wavelengths) < 3:
            errors = 1
    except Exception:
        errors = 1

    if errors == 1:
        print("[WARN] Loading of calibration data failed (missing caldata.txt or corrupted data)")
        print("[INFO] Loading placeholder data... you MUST perform a calibration to use this software!")
        pixels = [0, 400, 800]
        wavelengths = [380, 560, 750]

    # Generate wavelength data from polynomial fit
    wavelengthData = []

    if len(pixels) == 3:
        print("[INFO] Calculating 2nd-order polynomial fit (3 calibration points)...")
        coefficients = np.poly1d(np.polyfit(pixels, wavelengths, 2))
        c1 = coefficients[2]
        c2 = coefficients[1]
        c3 = coefficients[0]
        print(f"[INFO] Coefficients: {c1}, {c2}, {c3}")
        for pixel in range(width):
            wl = (c1 * pixel ** 2) + (c2 * pixel) + c3
            wavelengthData.append(round(wl, 6))
        print("[INFO] Wavelength data generated")
        print("[WARN] Calibration with only 3 wavelengths will not be highly accurate")
        if errors == 1:
            message = 0
        else:
            message = 1

    if len(pixels) > 3:
        print("[INFO] Calculating 3rd-order polynomial fit (4+ calibration points)...")
        coefficients = np.poly1d(np.polyfit(pixels, wavelengths, 3))
        c1 = coefficients[3]
        c2 = coefficients[2]
        c3 = coefficients[1]
        c4 = coefficients[0]
        print(f"[INFO] Coefficients: {c1}, {c2}, {c3}, {c4}")
        for pixel in range(width):
            wl = (c1 * pixel ** 3) + (c2 * pixel ** 2) + (c3 * pixel) + c4
            wavelengthData.append(round(wl, 6))

        # Calculate R-squared to validate the calibration fit
        predicted = []
        for px in pixels:
            y = (c1 * px ** 3) + (c2 * px ** 2) + (c3 * px) + c4
            predicted.append(y)
        corrMatrix = np.corrcoef(wavelengths, predicted)
        corr = corrMatrix[0, 1]
        rSquared = corr ** 2
        print(f"[INFO] R-Squared = {rSquared}")
        message = 2

    # Build status messages for the display banner
    if message == 0:
        calMsg1 = "UNCALIBRATED!"
        calMsg2 = "Defaults loaded"
        calMsg3 = "Perform Calibration!"
    elif message == 1:
        calMsg1 = "Calibrated"
        calMsg2 = "Using 3 cal points"
        calMsg3 = "2nd Order Polyfit"
    else:
        calMsg1 = "Calibrated"
        calMsg2 = f"Using {len(pixels)} cal points"
        calMsg3 = "3rd Order Polyfit"

    return [wavelengthData, calMsg1, calMsg2, calMsg3]




#####################################################################################################################
# Calibration Function - Write Calibration Data to File
#
#   Parameters:
#       clickArray (list) - list of [pixelX, pixelY] pairs selected by the user during pixel recording
#
#   Returns:
#       bool - True if calibration was saved successfully, False otherwise
#
#   How it works:
#       1. Prompts the user in the terminal for the known wavelength at each selected pixel position
#       2. Validates that the user entered numbers (not text)
#       3. Writes pixel positions on line 1 and wavelengths on line 2 of caldata.txt
#####################################################################################################################

def writeCalibration(clickArray):
    """Prompt user for known wavelengths and save calibration data to caldata.txt."""
    pxData = []
    wlData = []
    print("\n[INFO] Enter known wavelengths for the observed pixel positions:")

    for point in clickArray:
        pixel = point[0]
        wavelength = input(f"  Enter wavelength (nm) for pixel {pixel}: ")
        pxData.append(pixel)
        wlData.append(wavelength)

    # Validate that only numbers were entered
    try:
        wlData = [float(x) for x in wlData]
    except ValueError:
        print("[ERROR] Only numbers and decimals are allowed! Calibration aborted.")
        return False

    # Write to caldata.txt
    pxString = ",".join(map(str, pxData))
    wlString = ",".join(map(str, wlData))
    with open(CAL_DATA_FILE, "w") as f:
        f.write(pxString + "\n")
        f.write(wlString + "\n")
    print("[INFO] Calibration data written to caldata.txt")
    return True




#####################################################################################################################
# Helper Function - Generate Graticule Lines for the Graph
#
#   Parameters:
#       wavelengthData (list) - array of wavelengths, one per pixel column
#
#   Returns:
#       list - [tensPositions, fiftiesPositions]
#           tensPositions: pixel positions where a line should be drawn every 10 nm
#           fiftiesPositions: [[pixel, wavelength], ...] where a labeled line should be drawn every 50 nm
#
#   How it works:
#       - Scans the wavelength data for positions that fall on whole 10 nm and 50 nm boundaries
#       - These positions are used to draw the vertical grid lines on the spectrum graph
#       - Only includes positions where the wavelength is within 1 nm of the target (avoids
#         crowding at the edges where wavelengths may bunch up)
#####################################################################################################################

def generateGraticule(wavelengthData):
    """Compute pixel positions for 10 nm and 50 nm graticule lines."""
    low = int(round(wavelengthData[0])) - 10
    high = int(round(wavelengthData[len(wavelengthData) - 1])) + 10

    tens = []
    for i in range(low, high):
        if i % 10 == 0:
            position = min(enumerate(wavelengthData), key=lambda x: abs(i - x[1]))
            if abs(i - position[1]) < 1:
                tens.append(position[0])

    fifties = []
    for i in range(low, high):
        if i % 50 == 0:
            position = min(enumerate(wavelengthData), key=lambda x: abs(i - x[1]))
            if abs(i - position[1]) < 1:
                labelPos = position[0]
                labelTxt = int(round(position[1]))
                fifties.append([labelPos, labelTxt])

    return [tens, fifties]




#####################################################################################################################
# Helper Function - Save Spectrum Data to Disk
#
#   Parameters:
#       saveData (list) - [spectrumImage, graphData] or [spectrumImage, graphData, waterfallImage]
#           spectrumImage: the OpenCV image of the spectrum display
#           graphData: [wavelengthData, intensityData] for CSV export
#           waterfallImage: (optional) the OpenCV image of the waterfall display
#
#   Returns:
#       str - status message showing when the last save occurred
#
#   How it works:
#       1. Creates the captures folder if it doesn't exist
#       2. Generates a timestamp for the filename (e.g., "Spectrum-20260226--153000")
#       3. Saves the spectrum graph as a PNG image
#       4. If waterfall is enabled, saves the waterfall display as a separate PNG
#       5. Saves wavelength and intensity data as a CSV file (Wavelength,Intensity)
#####################################################################################################################

CAPTURES_DIR = "Spectrometer_Captures"

def saveSnapshot(saveData):
    """Save spectrum graph as PNG and intensity data as CSV into CAPTURES_DIR."""
    os.makedirs(CAPTURES_DIR, exist_ok=True)
    now = time.strftime("%Y%m%d--%H%M%S")
    timeNow = time.strftime("%H:%M:%S")
    spectrumImage = saveData[0]
    graphData = saveData[1]

    if len(saveData) > 2:
        waterfallImage = saveData[2]
        path = os.path.join(CAPTURES_DIR, f"waterfall-{now}.png")
        cv2.imwrite(path, waterfallImage)
        print(f"[INFO] Waterfall image saved: {path}")

    imgPath = os.path.join(CAPTURES_DIR, f"spectrum-{now}.png")
    cv2.imwrite(imgPath, spectrumImage)
    print(f"[INFO] Spectrum image saved: {imgPath}")

    csvPath = os.path.join(CAPTURES_DIR, f"Spectrum-{now}.csv")
    with open(csvPath, "w") as f:
        f.write("Wavelength,Intensity\n")
        for wl, intensity in zip(graphData[0], graphData[1]):
            f.write(f"{wl},{intensity}\n")
    print(f"[INFO] CSV data saved: {csvPath}")

    statusMessage = f"Last Save: {timeNow}"
    return statusMessage




#####################################################################################################################
# Command-Line Argument Parsing
#   - --device N    : USB camera device number (default 0). Find yours with: v4l2-ctl --list-devices
#   - --fps N       : Camera frame rate (default 30)
#   - --fullscreen  : Launch in fullscreen mode (designed for 800x480 RPi screens)
#   - --waterfall   : Enable the waterfall display (spectral changes over time in a scrolling heatmap)
#   - These two display modes are mutually exclusive
#####################################################################################################################

parser = argparse.ArgumentParser(description="LUSI Science Module - USB Spectrometer")
parser.add_argument("--device", type=int, default=0, help="USB camera device number (default: 0)")
parser.add_argument("--fps", type=int, default=30, help="Camera frame rate (default: 30)")
displayGroup = parser.add_mutually_exclusive_group()
displayGroup.add_argument("--fullscreen", help="Run in fullscreen mode (800x480)", action="store_true")
displayGroup.add_argument("--waterfall", help="Enable waterfall display (windowed only)", action="store_true")
args = parser.parse_args()

dispFullscreen = args.fullscreen
dispWaterfall = args.waterfall
dev = args.device
fps = args.fps




#####################################################################################################################
# USB Camera Initialization
#   - If --device is specified, opens that device path directly
#   - If the default device (0) fails, auto-scans /dev/video* to find a working USB camera
#   - The Pi's camera subsystem creates many video device nodes (e.g., /dev/video10-23)
#     but only some are actual capture devices - the rest are metadata/control nodes
#   - Before trying to open a device, we check /sys/class/video4linux/ to filter out
#     non-capture nodes (only devices with index "0" in their sysfs entry are primary
#     capture devices - the rest are metadata/output/subdev nodes)
#   - Opens cameras by path string (e.g., "/dev/video14") instead of integer index,
#     which is more reliable for high device numbers
#   - Sets the resolution to 800x600 and the requested frame rate
#   - The expected resolution is 800x600 - other resolutions may cause issues
#####################################################################################################################

def findCaptureDevices():
    """Find /dev/video* devices that are actual video capture nodes (not metadata/subdev).
    Checks /sys/class/video4linux/ to filter by device index and capability."""
    candidates = []
    sysDevices = sorted(glob.glob("/sys/class/video4linux/video*"))

    for sysPath in sysDevices:
        devName = os.path.basename(sysPath)
        devPath = f"/dev/{devName}"

        # Only consider devices that actually exist
        if not os.path.exists(devPath):
            continue

        # Check the sysfs "index" file - index 0 = primary capture device
        indexFile = os.path.join(sysPath, "index")
        try:
            with open(indexFile, "r") as f:
                idx = int(f.read().strip())
                if idx != 0:
                    continue  # Skip non-primary nodes (metadata, output, subdev)
        except Exception:
            continue

        # Read the device name for logging
        nameFile = os.path.join(sysPath, "name")
        try:
            with open(nameFile, "r") as f:
                camName = f.read().strip()
        except Exception:
            camName = "Unknown"

        candidates.append((devPath, camName))

    return candidates


def tryOpenCamera(devicePath):
    """Try to open a camera by path string. Returns the VideoCapture object or None."""
    testCap = cv2.VideoCapture(devicePath, cv2.CAP_V4L2)
    if testCap.isOpened():
        ret, testFrame = testCap.read()
        if ret and testFrame is not None:
            return testCap
        testCap.release()
    return None


# First try the user-specified device
userDevPath = f"/dev/video{dev}"
print(f"[INFO] Opening USB camera ({userDevPath}) at {fps} FPS...")
cap = tryOpenCamera(userDevPath)

# If that failed, auto-scan only the real capture devices
if cap is None:
    captureDevices = findCaptureDevices()
    if captureDevices:
        print(f"[INFO] {userDevPath} not available. Found {len(captureDevices)} capture device(s):")
        for devPath, camName in captureDevices:
            print(f"[INFO]   {devPath} - {camName}")

        for devPath, camName in captureDevices:
            if devPath == userDevPath:
                continue  # Already tried this one
            print(f"[INFO]   Trying {devPath} ({camName})...", end="")
            cap = tryOpenCamera(devPath)
            if cap is not None:
                dev = int(devPath.replace("/dev/video", ""))
                print(f" OK")
                break
            else:
                print(f" skip")

if cap is None:
    print("[ERROR] No working camera found")
    print("[INFO] Check that the USB camera is plugged into a blue USB 3.0 port")
    print("[INFO] If the camera was just disconnected, unplug and replug it")
    exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
cap.set(cv2.CAP_PROP_FPS, fps)

actualWidth = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
actualHeight = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
actualFps = cap.get(cv2.CAP_PROP_FPS)
print(f"[INFO] Camera opened: /dev/video{dev} at {int(actualWidth)}x{int(actualHeight)} @ {actualFps} FPS")




#####################################################################################################################
# Window Setup
#   - Creates one or two OpenCV windows depending on the display mode:
#       - Spectrum window: always shown (graph + camera preview + status banner)
#       - Waterfall window: optional (scrolling heatmap of spectral changes over time)
#   - In fullscreen mode the window fills the entire screen (designed for 800x480 RPi LCDs)
#   - In windowed mode the windows are resizable and positioned at the top-left corner
#####################################################################################################################

TITLE_SPECTRUM = "LUSI Spectrometer - Spectrum"
TITLE_WATERFALL = "LUSI Spectrometer - Waterfall"

if dispWaterfall:
    cv2.namedWindow(TITLE_WATERFALL, cv2.WINDOW_GUI_NORMAL)
    cv2.resizeWindow(TITLE_WATERFALL, FRAME_WIDTH, STACK_HEIGHT)
    cv2.moveWindow(TITLE_WATERFALL, 200, 200)

if dispFullscreen:
    cv2.namedWindow(TITLE_SPECTRUM, cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty(TITLE_SPECTRUM, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
else:
    cv2.namedWindow(TITLE_SPECTRUM, cv2.WINDOW_GUI_NORMAL)
    cv2.resizeWindow(TITLE_SPECTRUM, FRAME_WIDTH, STACK_HEIGHT)
    cv2.moveWindow(TITLE_SPECTRUM, 0, 0)




#####################################################################################################################
# State Variables
#   - savpoly: current Savitzky-Golay filter polynomial order (adjustable with o/l keys)
#   - mindist: current minimum distance between labeled peaks (adjustable with i/k keys)
#   - thresh: current threshold for peak detection (adjustable with u/j keys)
#   - holdPeaks: when True, the graph shows the highest intensity ever seen at each pixel
#   - measure: when True, a crosshair cursor shows the wavelength at the mouse position
#   - recPixels: when True, clicking on the graph records pixel positions for calibration
#   - clickArray: list of [x, y] positions clicked during pixel recording mode
#   - cursorX / cursorY: current mouse position on the graph
#   - intensity: array of intensity values (one per pixel column) - updated every frame
#   - saveMsg: status text showing when data was last saved
#####################################################################################################################

savpoly = DEFAULT_SAVPOLY
mindist = DEFAULT_MINDIST
thresh = DEFAULT_THRESH

holdPeaks = False
measure = False
recPixels = False

clickArray = []
cursorX = 0
cursorY = 0

intensity = [0] * FRAME_WIDTH
saveMsg = "No data saved"

# Blank waterfall image (filled black, updated each frame if waterfall is enabled)
waterfall = np.zeros([GRAPH_HEIGHT, FRAME_WIDTH, 3], dtype=np.uint8)




#####################################################################################################################
# Mouse Event Handler
#   - Tracks the mouse position (for measurement cursor and pixel recording)
#   - Records click positions when pixel recording mode is active
#   - The mouseYOffset accounts for the banner and preview strip above the graph
#####################################################################################################################

def handleMouse(event, x, y, flags, param):
    """Handle mouse move and click events on the spectrum window."""
    global clickArray, cursorX, cursorY
    mouseYOffset = BANNER_HEIGHT + PREVIEW_HEIGHT

    if event == cv2.EVENT_MOUSEMOVE:
        cursorX = x
        cursorY = y

    if event == cv2.EVENT_LBUTTONDOWN:
        mouseX = x
        mouseY = y - mouseYOffset
        clickArray.append([mouseX, mouseY])

cv2.setMouseCallback(TITLE_SPECTRUM, handleMouse)




#####################################################################################################################
# Load Calibration Data and Generate Graticule
#   - readCalibration() loads caldata.txt (or defaults if not found)
#   - generateGraticule() computes pixel positions for the 10 nm and 50 nm grid lines
#   - These are computed once on startup and again after every recalibration
#####################################################################################################################

calData = readCalibration(FRAME_WIDTH)
wavelengthData = calData[0]
calMsg1 = calData[1]
calMsg2 = calData[2]
calMsg3 = calData[3]

graticuleData = generateGraticule(wavelengthData)
tens = graticuleData[0]
fifties = graticuleData[1]

font = cv2.FONT_HERSHEY_SIMPLEX




#####################################################################################################################
# Main Loop - Capture Frames, Process Spectrum, Display Graph
#
#   Each iteration:
#       1. Captures a frame from the USB camera
#       2. Crops a horizontal band from the center of the frame
#       3. Converts to grayscale and averages 3 rows of pixels for noise reduction
#       4. Applies Savitzky-Golay smoothing (unless peak hold is active)
#       5. Draws the intensity vs. wavelength graph with colored bars
#       6. Detects and labels peaks above the threshold
#       7. Draws measurement cursor, pixel recording markers, and status messages
#       8. Stacks the banner, preview, and graph into a single display image
#       9. Handles keyboard input for all controls
#####################################################################################################################

print("[INFO] Spectrometer running. Press 'q' to quit.")
print("[INFO] Press 'h' for peak hold, 'm' for measurement cursor, 'p' to record pixels, 'c' to calibrate")


# Signal handler for clean Ctrl+C shutdown (releases the camera so /dev/videoN stays available)
def cleanShutdown(signum, frame):
    print("\n[INFO] Ctrl+C detected - releasing camera and closing windows...")
    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Spectrometer shut down")
    exit(0)

signal.signal(signal.SIGINT, cleanShutdown)

while cap.isOpened():
    ret, frame = cap.read()

    if not ret:
        print("[ERROR] Failed to capture frame from camera")
        break


    # ============================================================
    # Step 1: Crop the camera frame to a horizontal band centered on the spectrum
    # ============================================================
    cropY = int((FRAME_HEIGHT / 2) - 40)
    cropX = 0
    cropH = PREVIEW_HEIGHT
    cropW = FRAME_WIDTH
    cropped = frame[cropY:cropY + cropH, cropX:cropX + cropW]

    # Convert to grayscale for intensity measurement
    bwImage = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
    rows, cols = bwImage.shape
    halfway = int(rows / 2)

    # Draw indicator lines on the preview showing where intensity is sampled (3px region)
    cv2.line(cropped, (0, halfway - 2), (FRAME_WIDTH, halfway - 2), (255, 255, 255), 1)
    cv2.line(cropped, (0, halfway + 2), (FRAME_WIDTH, halfway + 2), (255, 255, 255), 1)


    # ============================================================
    # Step 2: Build the status banner (dark background with text)
    # ============================================================
    banner = np.zeros([BANNER_HEIGHT, FRAME_WIDTH, 3], dtype=np.uint8)
    banner[:] = (40, 40, 40)  # Dark gray background

    # Title text
    cv2.putText(banner, "LUSI Science Module - Spectrometer", (10, 25), font, 0.6, (0, 200, 255), 1, cv2.LINE_AA)
    cv2.putText(banner, f"Device: /dev/video{dev}  |  FPS: {actualFps}", (10, 50), font, 0.4, (180, 180, 180), 1, cv2.LINE_AA)
    cv2.putText(banner, "Press 'q' to quit  |  's' to save  |  'h' for help", (10, 70), font, 0.35, (140, 140, 140), 1, cv2.LINE_AA)

    # Calibration status (right side)
    cv2.putText(banner, calMsg1, (490, 20), font, 0.4, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(banner, calMsg2, (490, 38), font, 0.4, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(banner, calMsg3, (490, 56), font, 0.4, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(banner, saveMsg, (490, 74), font, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

    # Processing status (far right)
    holdMsg = "Holdpeaks ON" if holdPeaks else "Holdpeaks OFF"
    cv2.putText(banner, holdMsg, (660, 20), font, 0.35, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(banner, f"Savgol: {savpoly}", (660, 38), font, 0.35, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(banner, f"Peak Dist: {mindist}", (660, 56), font, 0.35, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(banner, f"Threshold: {thresh}", (660, 74), font, 0.35, (0, 255, 255), 1, cv2.LINE_AA)


    # ============================================================
    # Step 3: Build the spectrum graph (white background with graticule)
    # ============================================================
    graph = np.zeros([GRAPH_HEIGHT, FRAME_WIDTH, 3], dtype=np.uint8)
    graph.fill(255)

    # Vertical graticule lines every 10 nm (light gray)
    for position in tens:
        cv2.line(graph, (position, 15), (position, GRAPH_HEIGHT), (200, 200, 200), 1)

    # Vertical graticule lines every 50 nm (black, with labels)
    textOffset = 12
    for posData in fifties:
        cv2.line(graph, (posData[0], 15), (posData[0], GRAPH_HEIGHT), (0, 0, 0), 1)
        cv2.putText(graph, f"{posData[1]}nm", (posData[0] - textOffset, 12), font, 0.4, (0, 0, 0), 1, cv2.LINE_AA)

    # Horizontal reference lines every 64 pixels (light gray)
    for i in range(GRAPH_HEIGHT):
        if i >= 64 and i % 64 == 0:
            cv2.line(graph, (0, i), (FRAME_WIDTH, i), (100, 100, 100), 1)


    # ============================================================
    # Step 4: Read intensity data from the camera (3-row averaging)
    # ============================================================
    for i in range(cols):
        dataMinus1 = bwImage[halfway - 1, i]
        dataZero = bwImage[halfway, i]
        dataPlus1 = bwImage[halfway + 1, i]
        data = (int(dataMinus1) + int(dataZero) + int(dataPlus1)) / 3
        data = np.uint8(data)

        if holdPeaks:
            if data > intensity[i]:
                intensity[i] = data
        else:
            intensity[i] = data


    # ============================================================
    # Step 5: Update waterfall display (if enabled)
    # ============================================================
    if dispWaterfall:
        wData = np.zeros([1, FRAME_WIDTH, 3], dtype=np.uint8)
        for idx in range(len(intensity)):
            rgb = wavelengthToRgb(round(wavelengthData[idx]))
            luminosity = intensity[idx] / 255
            bVal = int(round(rgb[0] * luminosity))
            gVal = int(round(rgb[1] * luminosity))
            rVal = int(round(rgb[2] * luminosity))
            wData[0, idx] = (rVal, gVal, bVal)
        waterfall = np.insert(waterfall, 0, wData, axis=0)
        waterfall = waterfall[:-1].copy()


    # ============================================================
    # Step 6: Apply smoothing filter (unless holding peaks)
    # ============================================================
    if not holdPeaks:
        intensity = savitzkyGolay(intensity, 17, savpoly)
        intensity = np.array(intensity)
        intensity = intensity.astype(int)


    # ============================================================
    # Step 7: Draw the intensity data as colored vertical bars
    # ============================================================
    for idx in range(len(intensity)):
        rgb = wavelengthToRgb(round(wavelengthData[idx]))
        r = rgb[0]
        g = rgb[1]
        b = rgb[2]
        # OpenCV origin is top-left, so we draw from the bottom up
        cv2.line(graph, (idx, GRAPH_HEIGHT), (idx, GRAPH_HEIGHT - intensity[idx]), (b, g, r), 1)
        cv2.line(graph, (idx, GRAPH_HEIGHT - 1 - intensity[idx]), (idx, GRAPH_HEIGHT - intensity[idx]), (0, 0, 0), 1, cv2.LINE_AA)


    # ============================================================
    # Step 8: Detect peaks and draw labels
    # ============================================================
    threshVal = int(thresh)
    maxIntensity = max(intensity) if max(intensity) > 0 else 1
    indexes = peakIndexes(intensity, thres=threshVal / maxIntensity, minDist=mindist)

    for i in indexes:
        height = intensity[i]
        yPos = GRAPH_HEIGHT - 10 - height
        wavelength = round(wavelengthData[i], 1)
        # Yellow label box with black border
        cv2.rectangle(graph, ((i - textOffset) - 2, yPos), ((i - textOffset) + 60, yPos - 15), (0, 255, 255), -1)
        cv2.rectangle(graph, ((i - textOffset) - 2, yPos), ((i - textOffset) + 60, yPos - 15), (0, 0, 0), 1)
        cv2.putText(graph, f"{wavelength}nm", (i - textOffset, yPos - 3), font, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
        # Flagpole connecting label to the peak
        cv2.line(graph, (i, yPos), (i, yPos + 10), (0, 0, 0), 1)


    # ============================================================
    # Step 9: Draw measurement cursor (if active)
    # ============================================================
    if measure:
        adjY = cursorY - (BANNER_HEIGHT + PREVIEW_HEIGHT)
        cv2.line(graph, (cursorX, adjY - 20), (cursorX, adjY + 20), (0, 0, 0), 1)
        cv2.line(graph, (cursorX - 20, adjY), (cursorX + 20, adjY), (0, 0, 0), 1)
        if 0 <= cursorX < FRAME_WIDTH:
            cv2.putText(graph, f"{round(wavelengthData[cursorX], 2)}nm", (cursorX + 5, adjY - 5), font, 0.4, (0, 0, 0), 1, cv2.LINE_AA)


    # ============================================================
    # Step 10: Draw pixel recording cursor and selected points (if active)
    # ============================================================
    if recPixels:
        adjY = cursorY - (BANNER_HEIGHT + PREVIEW_HEIGHT)
        cv2.line(graph, (cursorX, adjY - 20), (cursorX, adjY + 20), (0, 0, 0), 1)
        cv2.line(graph, (cursorX - 20, adjY), (cursorX + 20, adjY), (0, 0, 0), 1)
        if 0 <= cursorX < FRAME_WIDTH:
            cv2.putText(graph, f"{cursorX}px", (cursorX + 5, adjY - 5), font, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
    else:
        # Keep click array empty when not in recording mode
        clickArray = []

    # Draw circles at all selected calibration points
    if clickArray:
        for point in clickArray:
            mouseX = point[0]
            mouseY = point[1]
            cv2.circle(graph, (mouseX, mouseY), 5, (0, 0, 0), -1)
            cv2.putText(graph, str(mouseX), (mouseX + 5, mouseY), font, 0.4, (0, 0, 0))


    # ============================================================
    # Step 11: Assemble and display the spectrum window
    # ============================================================
    spectrumVertical = np.vstack((banner, cropped, graph))

    # Dividing lines between sections
    cv2.line(spectrumVertical, (0, BANNER_HEIGHT), (FRAME_WIDTH, BANNER_HEIGHT), (255, 255, 255), 1)
    cv2.line(spectrumVertical, (0, BANNER_HEIGHT + PREVIEW_HEIGHT), (FRAME_WIDTH, BANNER_HEIGHT + PREVIEW_HEIGHT), (255, 255, 255), 1)

    cv2.imshow(TITLE_SPECTRUM, spectrumVertical)


    # ============================================================
    # Step 12: Assemble and display the waterfall window (if enabled)
    # ============================================================
    if dispWaterfall:
        waterfallVertical = np.vstack((banner, cropped, waterfall))

        # Dividing lines
        cv2.line(waterfallVertical, (0, BANNER_HEIGHT), (FRAME_WIDTH, BANNER_HEIGHT), (255, 255, 255), 1)
        cv2.line(waterfallVertical, (0, BANNER_HEIGHT + PREVIEW_HEIGHT), (FRAME_WIDTH, BANNER_HEIGHT + PREVIEW_HEIGHT), (255, 255, 255), 1)

        # Dashed graticule lines every 50 nm on the waterfall
        for posData in fifties:
            yStart = BANNER_HEIGHT + PREVIEW_HEIGHT + 2
            for y in range(yStart, STACK_HEIGHT):
                if y % 20 == 0:
                    cv2.line(waterfallVertical, (posData[0], y), (posData[0], y + 1), (0, 0, 0), 2)
                    cv2.line(waterfallVertical, (posData[0], y), (posData[0], y + 1), (255, 255, 255), 1)
            cv2.putText(waterfallVertical, f"{posData[1]}nm", (posData[0] - textOffset, STACK_HEIGHT - 5), font, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow(TITLE_WATERFALL, waterfallVertical)


    # ============================================================
    # Step 13: Handle keyboard input
    # ============================================================
    keyPress = cv2.waitKey(1)

    if keyPress == ord("q"):
        print("[INFO] Quit requested")
        break

    elif keyPress == ord("h"):
        holdPeaks = not holdPeaks
        state = "ON" if holdPeaks else "OFF"
        print(f"[INFO] Peak hold toggled {state}")

    elif keyPress == ord("s"):
        graphData = [wavelengthData, intensity]
        if dispWaterfall:
            saveData = [spectrumVertical, graphData, waterfallVertical]
        else:
            saveData = [spectrumVertical, graphData]
        saveMsg = saveSnapshot(saveData)

    elif keyPress == ord("c"):
        if not clickArray:
            print("[WARN] No calibration points selected. Press 'p' to enter pixel recording mode first.")
        else:
            calComplete = writeCalibration(clickArray)
            if calComplete:
                calData = readCalibration(FRAME_WIDTH)
                wavelengthData = calData[0]
                calMsg1 = calData[1]
                calMsg2 = calData[2]
                calMsg3 = calData[3]
                graticuleData = generateGraticule(wavelengthData)
                tens = graticuleData[0]
                fifties = graticuleData[1]

    elif keyPress == ord("x"):
        clickArray = []
        print("[INFO] Calibration points cleared")

    elif keyPress == ord("m"):
        recPixels = False
        measure = not measure
        state = "ON" if measure else "OFF"
        print(f"[INFO] Measurement cursor toggled {state}")

    elif keyPress == ord("p"):
        measure = False
        recPixels = not recPixels
        state = "ON" if recPixels else "OFF"
        print(f"[INFO] Pixel recording mode toggled {state}")

    elif keyPress == ord("o"):
        savpoly = min(savpoly + 1, 15)

    elif keyPress == ord("l"):
        savpoly = max(savpoly - 1, 0)

    elif keyPress == ord("i"):
        mindist = min(mindist + 1, 100)

    elif keyPress == ord("k"):
        mindist = max(mindist - 1, 0)

    elif keyPress == ord("u"):
        thresh = min(thresh + 1, 100)

    elif keyPress == ord("j"):
        thresh = max(thresh - 1, 0)




#####################################################################################################################
# Cleanup
#   - Releases the USB camera
#   - Closes all OpenCV windows
#####################################################################################################################

print("[INFO] Releasing camera and closing windows...")
cap.release()
cv2.destroyAllWindows()
print("[INFO] Spectrometer shut down")
