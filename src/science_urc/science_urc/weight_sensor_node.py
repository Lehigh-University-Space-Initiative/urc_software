
"""ROS 2 node for the LUSI Science Module weight sensor."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from science_urc.hardware.weight_sensor import WeightSensor


class WeightSensorNode(Node):
    """Reads the HX711 load cell and publishes its measurements."""

    def __init__(self, weight_sensor=None, parameter_overrides=None):
        super().__init__(
            "weight_sensor_node",
            parameter_overrides=parameter_overrides,
        )
        # Use a fake sensor during testing or the real HX711 on the Pi.
        self.sensor = (
            weight_sensor
            if weight_sensor is not None
            else WeightSensor()
        )
        # No calibration factor is assumed until one is provided.
        self.declare_parameter("reference_unit", 0.0)
        reference_unit = self.get_parameter("reference_unit").value

        if reference_unit != 0.0:
            try:
                self.sensor.set_calibration(reference_unit)
                self.get_logger().info("Weight sensor reference unit configured")
            except (ValueError, RuntimeError, OSError, AttributeError) as exc:
                self.get_logger().error(
                    f"Failed to configure weight sensor calibration: {exc}"
                )
        else:
            self.get_logger().warn(
                "No weight sensor calibration factor configured"
            )

        # Publish a new measurement about once per second.
        # These values must be calibrated before we can trust them as grams.
        self.weight_publisher = self.create_publisher(
            Float32,
            "/science/weight_g",
            10
        )

        # Tare the load cell before starting measurements.
        # Make sure the sample container is empty when the node starts.
        self.sensor.tare()

        # Read and publish the sensor value once per second.
        self.create_timer(1.0, self.publish_weight)

        self.get_logger().info("Science Module weight sensor started")

    def publish_weight(self):
        """Read the load cell and publish its current measurement."""

        try:
            # Don't publish values as grams until the sensor is calibrated.
            if not getattr(self.sensor, "calibrated", False):
                self.get_logger().warn(
                    "Weight sensor is not calibrated; skipping weight publication",
                    throttle_duration_sec=10.0,
                )
                return

            weight = self.sensor.read_weight()

            msg = Float32()
            msg.data = float(weight)

            self.weight_publisher.publish(msg)

        except Exception as exc:
            # Log sensor errors instead of crashing the ROS node.
            self.get_logger().error(
                f"Failed to read weight sensor: {exc}"
            )

    def shutdown(self):
        """Close the weight sensor connection."""

        self.get_logger().info("Shutting down weight sensor")
        self.sensor.close()


def main(args=None):
    """Start the weight sensor node and clean up when it exits."""

    rclpy.init(args=args)
    node = None

    try:
        node = WeightSensorNode()
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
