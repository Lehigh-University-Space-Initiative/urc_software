"""ROS 2 camera node for the LUSI Science Module."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from science_urc.hardware.camera import ScienceCamera


class CameraNode(Node):
    def __init__(self, camera=None):
        super().__init__("science_camera_node")

        self.declare_parameter("camera_index", 1)
        self.declare_parameter("frame_rate", 10.0)

        camera_index = self.get_parameter("camera_index").value
        frame_rate = self.get_parameter("frame_rate").value

        if frame_rate <= 0:
            raise ValueError("Frame rate must be greater than zero")

        self.camera = camera if camera is not None else ScienceCamera(camera_index)

        self.image_publisher = self.create_publisher(
            Image, "/science/camera/image_raw", 10
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

        except Exception as exc:
            self.get_logger().error(f"Failed to publish camera frame: {exc}")

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