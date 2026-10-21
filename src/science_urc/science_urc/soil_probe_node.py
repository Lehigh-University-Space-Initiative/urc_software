
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from science_urc.hardware.soil_probe import SoilProbe


class SoilProbeNode(Node):
    def __init__(self, probe=None):
        super().__init__("soil_probe_node")

        self.declare_parameter("port", "/dev/serial0")
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("sample_rate", 0.2)

        port = self.get_parameter("port").value
        baudrate = self.get_parameter("baudrate").value
        sample_rate = float(self.get_parameter("sample_rate").value)

        if sample_rate <= 0:
            raise ValueError("sample_rate must be positive")

        # Use an existing probe for testing, or connect to the real hardware.
        self.probe = probe
        if self.probe is None:
            self.probe = SoilProbe(port=port, baudrate=baudrate)

        # Give each soil measurement its own ROS topic.
        self.soil_publishers = {
            name: self.create_publisher(
                Float32, f"/science/soil/{name}", 10
            )
            for name in (
                "moisture",
                "temperature",
                "ph",
                "conductivity",
                "nitrogen",
                "phosphorus",
                "potassium",
            )
        }

        # Read the probe at the configured sampling rate.
        self.timer = self.create_timer(
            1.0 / sample_rate, self.publish_measurements
        )

    def publish_measurements(self):
        try:
            readings = self.probe.read_measurements()
        except Exception as exc:
            self.get_logger().error(f"Soil probe read failed: {exc}")
            return

        for name, publisher in self.soil_publishers.items():
            value = readings.get(name)

            # Skip failed readings instead of publishing incorrect zeros.
            if value is None:
                self.get_logger().warning(
                    f"No valid soil {name} reading"
                )
                continue

            message = Float32()
            message.data = float(value)
            publisher.publish(message)

    def destroy_node(self):
        try:
            # Release the serial port and GPIO pin when shutting down.
            if self.probe is not None:
                self.probe.close()
        finally:
            return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SoilProbeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
