#####################################################################################################################
# Importing Program Libraries
#   - cv2:
#       - Provides OpenCV’s video capture and device control features
#       - Enables detection of connected camera devices by index number
#       - Supports reading single frames to verify if a camera is accessible
#####################################################################################################################

import cv2





#####################################################################################################################
# Camera Scanning Function
#   - Scans for connected camera devices across possible index values
#   - Each index corresponds to a unique camera recognized by the operating system
#   - Verifies connection by attempting to open the camera and read a frame
#   - Prints a list showing which indices correspond to valid, accessible cameras
#   - Typically:
#       - Index 0: Built-in laptop webcam
#       - Index 1+: External USB cameras (ex. ELP USB camera)
#####################################################################################################################

def listCameras(totalIndices=5):
    print("Scanning for available cameras...")
    for cameraIndex in range(totalIndices):
        cameraCapture = cv2.VideoCapture(cameraIndex)  # Attempts to open the camera at this index
        success, capturedFrame = cameraCapture.read()  # Try reading one frame to confirm camera access (success=True if frame was captured)

        if success:
            print(f":::: Camera detected at index {cameraIndex}")  # Successfully accessed and functioning
            cameraCapture.release()  # Release device handle to prevent resource lock
        else:
            print(f":::: No camera detected at index {cameraIndex}")  # No response from this index (device not present)

    print("Scan complete.")  # End of scan summary and loop completion





#####################################################################################################################
# Main Execution
#   - Calls the listCameras() function
#   - Scans up to index 4 (the first 5 potential camera devices)
#   - Adjust totalIndices if more or fewer camera devices are expected
#####################################################################################################################

listCameras()