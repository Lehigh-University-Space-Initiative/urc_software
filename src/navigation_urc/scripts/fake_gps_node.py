#!/usr/bin/env python3
"""
Fake GPS for testing WaypointFollower without rover hardware

Listens to /cmd_vel, moves a simulated rover position based on it, and publishes that position back out on /gps_data
That closes the control loop entirely in software, so the waypoint follower can be tested on a laptop

Started by:
    navigation_sim_launch.py (run mode "nav_sim" in run_nodes.sh)

Run it alone with:
    ros2 run navigation_urc fake_gps_node.py --ros-args -p start_lat:=38.4061 -p start_lon:=-110.7918

Parameters:
    start_lat, start_lon - starting position in degrees (default: Mars Desert Research Station, Hanksville UT)
    publish_rate_hz - how often to update and publish the simulated position
"""

# Imports

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from cross_pkg_messages.msg import GPSData

EARTH_RADIUS_M = 6371000.0  # Same constant WaypointFollower uses




# ---------------------------------
# FakeGPS
# ---------------------------------

class FakeGPS(Node):
    """
    Simulated rover that integrates /cmd_vel into a position and reports it as GPS fixes

    Heading convention:
        Compass-style: 0 = north, clockwise positive (matching WaypointFollower's bearing)
        That differs from ROS's usual math angle (counterclockwise positive, 0 = east)
    """

    def __init__(self):
        """
        Declare the parameters, set up the topics, and start the update timer
        """
        # Syntax: super().__init__("FakeGPS") runs the rclpy Node constructor, naming this node "FakeGPS"
        super().__init__("FakeGPS")

        self.declare_parameter("start_lat", 38.4061)
        self.declare_parameter("start_lon", -110.7918)
        self.declare_parameter("publish_rate_hz", 10.0)

        self.start_lat = self.get_parameter("start_lat").value
        self.start_lon = self.get_parameter("start_lon").value
        rate_hz = self.get_parameter("publish_rate_hz").value

        # Simulated position as meters east/north of the start point, plus a compass heading in degrees
        self.x_east_m = 0.0
        self.y_north_m = 0.0
        self.heading_deg = 0.0

        # Latest drive command received; (0, 0) until WaypointFollower sends one
        self.linear_x = 0.0
        self.angular_z = 0.0

        self.gps_pub = self.create_publisher(GPSData, "gps_data", 10)
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        # Syntax: create_timer(period, callback) calls callback every period seconds
        self.dt = 1.0 / rate_hz
        self.create_timer(self.dt, self.tick)

        self.get_logger().info(f"FakeGPS running, starting at ({self.start_lat}, {self.start_lon})")

    def cmd_vel_callback(self, msg):
        """
        Remember the latest drive command

        Args:
            msg: geometry_msgs/Twist from WaypointFollower (linear.x in m/s, angular.z in rad/s)
        """
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def tick(self):
        """
        Advance the simulated rover by one time step and publish its new position

        Steps:
            1. Turn by angular.z (converted from ROS's counterclockwise convention to compass heading)
            2. Move forward by linear.x along the new heading
            3. Convert the east/north offset back to latitude/longitude and publish it as a GPS fix
        """
        # angular.z is counterclockwise-positive (turning left = positive), but heading is compass-style (clockwise-positive)
        # So a positive angular.z must DECREASE heading_deg, not increase it
        self.heading_deg -= math.degrees(self.angular_z) * self.dt
        self.heading_deg %= 360.0

        # North is +y and east is +x, so with a compass heading, east uses sin and north uses cos
        heading_rad = math.radians(self.heading_deg)
        self.x_east_m += self.linear_x * math.sin(heading_rad) * self.dt
        self.y_north_m += self.linear_x * math.cos(heading_rad) * self.dt

        # Converting the meters offset back into degrees (the inverse of the small-distance haversine relationship)
        # Longitude lines get closer together away from the equator, hence the cos(latitude) correction
        lat = self.start_lat + math.degrees(self.y_north_m / EARTH_RADIUS_M)
        lon = self.start_lon + math.degrees(self.x_east_m / (EARTH_RADIUS_M * math.cos(math.radians(self.start_lat))))

        msg = GPSData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = 1  # Arbitrary "valid fix" marker (no real meaning in simulation)
        msg.lla.x = lat
        msg.lla.y = lon
        msg.lla.z = 0.0
        msg.speed = self.linear_x
        msg.course = self.heading_deg
        msg.sats = 8
        self.gps_pub.publish(msg)




# ---------------------------------
# Entry point
# ---------------------------------

def main(args=None):
    """
    Start ROS, run the FakeGPS node until Ctrl+C, then shut down cleanly
    """
    rclpy.init(args=args)
    node = FakeGPS()
    rclpy.spin(node)  # Runs the timer and subscription callbacks until shutdown
    node.destroy_node()
    rclpy.shutdown()


# Syntax: this runs main() only when the file is executed directly, not when it's imported
if __name__ == "__main__":
    main()
