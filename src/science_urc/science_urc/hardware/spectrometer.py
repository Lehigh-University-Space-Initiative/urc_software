"""USB spectrometer interface for the LUSI Science Module."""

import numpy as np


class Spectrometer:
    def __init__(self, device=0, capture=None):
        self.closed = False
        self.capture = capture

        if self.capture is None:
            import cv2

            self.capture = cv2.VideoCapture(device)

        if not self.capture.isOpened():
            self.capture.release()
            raise RuntimeError(f"Could not open spectrometer camera {device}")

    def read_intensity(self):
        if self.closed:
            raise RuntimeError("Spectrometer is closed")

        success, frame = self.capture.read()

        if not success or frame is None:
            raise RuntimeError("Failed to read spectrometer frame")

        # Sample the middle three rows, as in the standalone script
        if frame.ndim != 3 or frame.shape[2] != 3 or frame.shape[0] < 3:
            raise RuntimeError("Invalid spectrometer frame")

        import cv2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        middle = gray.shape[0] // 2

        intensity = gray[middle - 1:middle + 2].mean(axis=0)
        return np.asarray(intensity, dtype=float)

    def close(self):
        if self.closed:
            return

        self.closed = True
        self.capture.release()
        