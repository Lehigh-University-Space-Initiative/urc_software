
"""ROS 2 node for controlling the LUSI Science Module motors."""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from science_urc.hardware.motor_controller import MotorController


class ScienceNode(Node):
    """Handles ROS commands for the Science Module mechanisms."""

    def __init__(self, motor_controller=None):
        super().__init__("science_node")

        # Use a fake motor controller for testing or the real one on the Pi.
        self.motor = (
            motor_controller
            if motor_controller is not None
            else MotorController()
        )

        # Stop the auger or platform if no new command arrives for 2 seconds.
        self.command_timeout = 2.0
        self.last_auger_command = None
        self.last_platform_command = None

        # Once a hardware error happens, don't accept more motor commands.
        # The node needs to be restarted after the issue is fixed.
        self.hardware_fault = False

        # Auger speed: Float32 from 0 to 100 percent.
        self.create_subscription(
            Float32,
            "/science/auger_speed",
            self.auger_callback,
            10
        )

        # Platform command: String such as "up,50" or "down,25".
        self.create_subscription(
            String,
            "/science/platform_command",
            self.platform_callback,
            10
        )

        # Soil dropper angle: Float32 from 0 to 180 degrees.
        self.create_subscription(
            Float32,
            "/science/soil_drop_angle",
            self.soil_drop_callback,
            10
        )

        # Chamber lid angle: Float32 from 0 to 180 degrees.
        self.create_subscription(
            Float32,
            "/science/chamber_lid_angle",
            self.chamber_lid_callback,
            10
        )

        # Check for motor command timeouts every 0.1 seconds.
        self.create_timer(0.1, self.check_timeout)

        self.get_logger().info("Science Module ROS node started")

    def handle_hardware_fault(self, exc):
        """Stop the motors and block new commands after a hardware error."""

        # Only handle the first fault so we don't keep sending stop commands.
        if self.hardware_fault:
            return

        self.hardware_fault = True

        # Clear the watchdog timers since the motors should no longer run.
        self.last_auger_command = None
        self.last_platform_command = None

        self.get_logger().error(
            f"Science Module hardware fault: {exc}"
        )

        # Try to stop everything, even if only one motor had a problem.
        try:
            self.motor.stop_all()
            self.get_logger().warning(
                "Stop command sent after hardware fault"
            )

        except Exception as stop_exc:
            # If communication with the Pi fails, stopping may not work.
            self.get_logger().error(
                f"Failed to stop Science Module motors: {stop_exc}"
            )

    def auger_callback(self, msg):
        """Set the auger speed using a ROS command."""

        if self.hardware_fault:
            self.get_logger().warning(
                "Auger command ignored: hardware fault active"
            )
            return

        try:
            self.motor.set_auger_speed(msg.data)

            # Update the auger watchdog after a successful command.
            self.last_auger_command = time.monotonic()

        except ValueError as exc:
            self.get_logger().warning(
                f"Invalid auger command: {exc}"
            )

        except Exception as exc:
            self.handle_hardware_fault(exc)

    def platform_callback(self, msg):
        """Move the sampling platform up or down at the requested speed."""

        if self.hardware_fault:
            self.get_logger().warning(
                "Platform command ignored: hardware fault active"
            )
            return

        try:
            # Split commands like "down,50" into direction and speed.
            direction, speed = msg.data.split(",")

            direction = direction.strip()
            speed = float(speed)

            self.motor.set_platform_speed(speed, direction)

            # The platform has its own watchdog, separate from the auger.
            self.last_platform_command = time.monotonic()

        except (ValueError, TypeError) as exc:
            self.get_logger().warning(
                f"Invalid platform command: {exc}"
            )

        except Exception as exc:
            self.handle_hardware_fault(exc)

    def soil_drop_callback(self, msg):
        """Set the soil dropper servo angle."""

        if self.hardware_fault:
            self.get_logger().warning(
                "Soil dropper command ignored: hardware fault active"
            )
            return

        try:
            self.motor.set_soil_drop_angle(msg.data)

        except ValueError as exc:
            self.get_logger().warning(
                f"Invalid soil dropper command: {exc}"
            )

        except Exception as exc:
            self.handle_hardware_fault(exc)

    def chamber_lid_callback(self, msg):
        """Set the chamber lid servo angle."""

        if self.hardware_fault:
            self.get_logger().warning(
                "Chamber lid command ignored: hardware fault active"
            )
            return

        try:
            self.motor.set_chamber_lid_angle(msg.data)

        except ValueError as exc:
            self.get_logger().warning(
                f"Invalid chamber lid command: {exc}"
            )

        except Exception as exc:
            self.handle_hardware_fault(exc)

    def check_timeout(self):
        """Stop the auger or platform if commands stop coming in."""

        # Don't send normal motor commands after a hardware fault.
        if self.hardware_fault:
            return

        now = time.monotonic()

        # Check the auger timeout.
        if self.last_auger_command is not None:
            if now - self.last_auger_command > self.command_timeout:
                try:
                    self.motor.set_auger_speed(0)
                    self.last_auger_command = None

                    self.get_logger().warning(
                        "Auger command timed out; motor stopped"
                    )

                except Exception as exc:
                    self.handle_hardware_fault(exc)
                    return

        # Check the platform timeout separately.
        if self.last_platform_command is not None:
            if now - self.last_platform_command > self.command_timeout:
                try:
                    self.motor.set_platform_speed(0)
                    self.last_platform_command = None

                    self.get_logger().warning(
                        "Platform command timed out; motor stopped"
                    )

                except Exception as exc:
                    self.handle_hardware_fault(exc)
                    return

    def shutdown(self):
        """Stop the motors and close the hardware connection."""

        self.get_logger().info("Shutting down Science Module")

        # MotorController.close() also calls stop_all().
        self.motor.close()


def main(args=None):
    """Start the Science Module node and clean up when it exits."""

    rclpy.init(args=args)
    node = None

    try:
        node = ScienceNode()

        # Keep the node running and listening for ROS commands.
        rclpy.spin(node)

    except KeyboardInterrupt:
        # Allow Ctrl+C to shut down the node normally.
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
