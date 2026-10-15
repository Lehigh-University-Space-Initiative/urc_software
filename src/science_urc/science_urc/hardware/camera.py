"""USB camera interface for the LUSI Science Module."""


class ScienceCamera:
    def __init__(self, index=1, capture=None):
        self.index = index
        self.capture = capture
        self.closed = False

        if self.capture is None:
            import cv2

            self.capture = cv2.VideoCapture(index)

        if not self.capture.isOpened():
            self.capture.release()
            raise RuntimeError(f"Could not open camera at index {index}")

    def read_frame(self):
        if self.closed:
            raise RuntimeError("Camera is closed")

        success, frame = self.capture.read()

        if not success or frame is None:
            raise RuntimeError("Failed to read camera frame")

        return frame

    def close(self):
        if self.closed:
            return

        self.closed = True
        self.capture.release()