"""ROS 2 spectrometer node for the LUSI Science Module."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from science_urc.hardware.spectrometer import Spectrometer


class SpectrometerNode(Node):
    def __init__(self, spectrometer=None):
        super().__init__("science_spectrometer_node")

        # Camera settings for the USB spectrometer
        self.declare_parameter("device", 0)
        self.declare_parameter("sample_rate", 10.0)

        device = self.get_parameter("device").value
        sample_rate = self.get_parameter("sample_rate").value

        if sample_rate <= 0:
            raise ValueError("Sample rate must be greater than zero")

        # Use the real camera unless a test spectrometer was provided
        self.spectrometer = (
            spectrometer if spectrometer is not None
            else Spectrometer(device=device)
        )

        # Publish one intensity value for each camera pixel column
        self.intensity_publisher = self.create_publisher(
            Float32MultiArray,
            "/science/spectrometer/intensity",
            10,
        )

        self.create_timer(1.0 / sample_rate, self.publish_intensity)

        self.get_logger().info("Science Module spectrometer started")

    def publish_intensity(self):
        try:
            intensity = self.spectrometer.read_intensity()

            msg = Float32MultiArray()
            msg.data = [float(value) for value in intensity]

            self.intensity_publisher.publish(msg)

        except Exception as exc:
            self.get_logger().error(
                f"Failed to publish spectrometer intensity: {exc}"
            )

    def shutdown(self):
        # Release the USB camera when the node stops
        self.get_logger().info("Shutting down spectrometer")
        self.spectrometer.close()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SpectrometerNode()
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