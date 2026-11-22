#!/usr/bin/env python3
"""
Voice-command behavior state machine: turns spoken commands into rover drive commands.

Runs on: presumably the rover main computer (VERIFY once a launch file starts it).
Launched by: nothing yet; start it by hand with `ros2 run voice_commands_urc behavior_controller`.

Subscribes to:
  behavior_command (std_msgs/String) - one of "stay", "follow", "come", "fetch"
  person_position (geometry_msgs/Point) - person's position relative to the rover, in meters
    (x forward, y to the rover's left)

Publishes:
  cmd_vel (geometry_msgs/Twist) - linear.x forward speed in m/s, angular.y turn rate in deg/s

Behaviors:
  stay   - holding still (the default)
  come   - driving to within 2 m of the person, then switching to stay
  follow - staying about 2 m from the person for as long as positions keep arriving
  fetch  - placeholder; holds still until the approach/pickup/return sequence is written
  ("give" from docs/state_machine_design_plan.png is not implemented yet)

How it connects to the system:
  - Nothing publishes behavior_command or person_position yet
    (a speech recognizer and a person tracker are still to be built)
  - cmd_vel is the same topic JoyMapper (manual driving) and WaypointFollower publish on
  - This node only publishes on cmd_vel while it is actively driving, plus one stop message
    when it stops, so it does not override manual driving the rest of the time
  - Turn commands use angular.y in deg/s to match DriveTrainManager and JoyMapper
    (see "Known integration issues" in README.md: WaypointFollower uses angular.z in rad/s)
  - Built for the URC Autonomy mission's astronaut-assistance tasks
"""

import math

# Syntax: rclpy is the ROS 2 Python client library; Node is the base class for every node
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist

# Commands this node understands (anything else is logged and ignored)
KNOWN_COMMANDS = {'stay', 'follow', 'come', 'fetch'}

# Distance to keep from the person, in meters
DESIRED_DISTANCE_M = 2.0

# Person positions older than this are treated as lost, in seconds
POSITION_TIMEOUT_S = 1.0

# Largest heading error that still produces a proportional turn, in radians (about 34 degrees)
MAX_TURN_ANGLE_RAD = 0.6

# Only drive forward while the person is within this angle of straight ahead, in radians
FACING_TOLERANCE_RAD = math.radians(30)

# Forward speed per meter of extra distance, and its cap, in m/s
FORWARD_GAIN = 0.3
MAX_FORWARD_SPEED_MPS = 0.4

# How often the state machine decides what to do, in seconds (10 Hz)
UPDATE_PERIOD_S = 0.1


class BehaviorController(Node):
    """Tracks the current voice-command behavior and drives the rover to carry it out."""

    def __init__(self):
        """
        Set up the topics, the timer, and the starting behavior.

        Steps:
          1. Starting in "stay" with no known person position
          2. Subscribing to behavior_command and person_position
          3. Creating the cmd_vel publisher
          4. Starting a 10 Hz timer that runs update()
        """
        super().__init__('behavior_controller')

        self.behavior = 'stay'
        self.person_position = None
        self.last_person_update = None

        # Remembering whether the last thing sent on cmd_vel was a nonzero drive command
        # (lets the node send exactly one stop instead of flooding cmd_vel with zeros)
        self.is_driving = False

        # System: behavior_command will come from a speech-recognition node (not built yet)
        self.create_subscription(String, 'behavior_command', self.on_command, 10)

        # System: person_position will come from a person-tracking node (not built yet)
        self.create_subscription(Point, 'person_position', self.on_person_position, 10)

        # System: cmd_vel is read by DriveTrainManager on the main computer
        self.drive_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_timer(UPDATE_PERIOD_S, self.update)

    def on_command(self, message):
        """
        Switch to a new behavior when a voice command arrives.

        Args:
            message: std_msgs/String holding the spoken command.
        """
        # Normalizing the text so " Follow" and "follow" match
        command = message.data.strip().lower()

        if command not in KNOWN_COMMANDS:
            self.get_logger().warning(f'Unknown command: {command}')
            return

        self.behavior = command
        self.get_logger().info(f'Behavior: {command}')

        # Stopping before the new behavior takes over, so a half-finished motion doesn't carry on
        self.stop()

    def on_person_position(self, message):
        """
        Store the latest person position and when it arrived.

        Args:
            message: geometry_msgs/Point, the person's position relative to the rover in meters.
        """
        self.person_position = message
        self.last_person_update = self.get_clock().now()

    def update(self):
        """
        Decide what to send on cmd_vel for the current behavior (runs at 10 Hz).

        Steps:
          1. Working out a drive command for "come" or "follow" when a fresh position exists
          2. Switching "come" to "stay" once the rover is close enough
          3. Publishing the drive command, or one stop if the rover was driving and shouldn't be
        """
        movement = None

        # "stay" and "fetch" never drive (fetch is not implemented yet)
        if self.behavior in {'come', 'follow'} and self.has_fresh_position():
            distance = math.hypot(self.person_position.x, self.person_position.y)

            if distance <= DESIRED_DISTANCE_M:
                # Arrival ends "come"; "follow" just waits here until the person moves away
                if self.behavior == 'come':
                    self.behavior = 'stay'
                    self.get_logger().info('Arrived; behavior: stay')
            else:
                movement = self.move_toward_person(distance)

        if movement is not None:
            self.drive_publisher.publish(movement)
            self.is_driving = True
        else:
            self.stop()

    def has_fresh_position(self):
        """
        Check whether a person position has arrived recently enough to steer by.

        Return value:
            True if the last position is newer than POSITION_TIMEOUT_S.
        """
        if self.last_person_update is None:
            return False

        # Syntax: subtracting two ROS times gives a Duration; nanoseconds / 1e9 converts to seconds
        age_s = (self.get_clock().now() - self.last_person_update).nanoseconds / 1e9
        return age_s < POSITION_TIMEOUT_S

    def move_toward_person(self, distance):
        """
        Build a drive command that turns toward the person and closes the gap.

        Args:
            distance: current straight-line distance to the person, in meters.

        Returns:
            geometry_msgs/Twist with linear.x in m/s and angular.y in deg/s.
        """
        # Syntax: a fresh Twist starts with every speed at zero
        movement = Twist()

        # Heading to the person (positive means the person is to the rover's left)
        angle = math.atan2(self.person_position.y, self.person_position.x)

        # Clamping the heading error so a person directly beside the rover doesn't cause a spin
        clamped_angle = max(-MAX_TURN_ANGLE_RAD, min(MAX_TURN_ANGLE_RAD, angle))

        # Converting to deg/s and flipping the sign for DriveTrainManager's convention
        # (there, a positive angular.y speeds up the left wheels, which turns the rover right)
        movement.angular.y = -math.degrees(clamped_angle)

        # Driving forward only while roughly facing the person, at a speed proportional to the gap
        if abs(angle) < FACING_TOLERANCE_RAD:
            gap = distance - DESIRED_DISTANCE_M
            movement.linear.x = min(MAX_FORWARD_SPEED_MPS, FORWARD_GAIN * gap)

        return movement

    def stop(self):
        """Send one zero command if the rover was driving; otherwise send nothing."""
        # Publishing zeros only once, so manual driving on cmd_vel isn't overridden
        if self.is_driving:
            self.drive_publisher.publish(Twist())
            self.is_driving = False


def main():
    """Start the node, and make sure the rover stops if the node shuts down mid-drive."""
    rclpy.init()
    controller = BehaviorController()

    # Syntax: spin() blocks and runs callbacks until Ctrl+C or shutdown
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Sending the final stop only while ROS is still up (publishing after shutdown raises)
        if rclpy.ok():
            controller.stop()
        controller.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
