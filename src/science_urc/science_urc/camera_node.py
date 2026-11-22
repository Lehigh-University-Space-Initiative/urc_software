"""
ROS 2 camera node for the LUSI Science Module.

Runs on: the Science Raspberry Pi.
Launched by: science_urc/launch/science_launch.py (run mode `science`).

Publishes:
  /science/camera/image_raw (sensor_msgs/Image) - raw BGR frames
  /science/camera/image_raw/compressed (sensor_msgs/CompressedImage) - JPEG frames

How it connects to the system:
  - The base station's web_video_server streams the compressed topic to the web UI,
    so only JPEG frames cross the radio (raw frames are only sent if something subscribes)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from science_urc.hardware.camera import ScienceCamera


class CameraNode(Node):
    def __init__(self, camera=None):
        super().__init__("science_camera_node")

        self.declare_parameter("camera_index", 1)
        self.declare_parameter("frame_rate", 10.0)
        # JPEG quality from 1 (smallest) to 100 (best) for the compressed stream
        self.declare_parameter("jpeg_quality", 70)

        camera_index = self.get_parameter("camera_index").value
        frame_rate = self.get_parameter("frame_rate").value
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        if frame_rate <= 0:
            raise ValueError("Frame rate must be greater than zero")

        self.camera = camera if camera is not None else ScienceCamera(camera_index)

        self.image_publisher = self.create_publisher(
            Image, "/science/camera/image_raw", 10
        )

        # System: the "/compressed" suffix is the image_transport naming convention,
        # which web_video_server uses when asked for type=ros_compressed
        self.compressed_publisher = self.create_publisher(
            CompressedImage, "/science/camera/image_raw/compressed", 10
        )

        self.create_timer(1.0 / frame_rate, self.publish_frame)
        self.get_logger().info("Science Module camera started")

    def publish_frame(self):
        try:
            frame = self.camera.read_frame()

            height, width = frame.shape[:2]

            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "science_camera"
            msg.height = height
            msg.width = width
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = width * 3
            msg.data = frame.tobytes()

            self.image_publisher.publish(msg)

            # Encoding JPEG only when someone is listening (saves Pi CPU otherwise)
            if self.compressed_publisher.get_subscription_count() > 0:
                self.compressed_publisher.publish(
                    self.compress_frame(frame, msg.header)
                )

        except Exception as exc:
            self.get_logger().error(f"Failed to publish camera frame: {exc}")

    def compress_frame(self, frame, header):
        """
        Encode a BGR frame as a JPEG CompressedImage message.

        Args:
            frame: numpy BGR image from the camera.
            header: header to copy (same stamp and frame_id as the raw image).

        Returns:
            sensor_msgs/CompressedImage holding the JPEG bytes.
        """
        import cv2

        success, encoded = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not success:
            raise RuntimeError("JPEG encoding failed")

        compressed = CompressedImage()
        compressed.header = header
        compressed.format = "jpeg"
        compressed.data = encoded.tobytes()
        return compressed

    def shutdown(self):
        self.get_logger().info("Shutting down science camera")
        self.camera.close()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                node.shutdown()
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()


if __name__ == "__main__":
    main()
