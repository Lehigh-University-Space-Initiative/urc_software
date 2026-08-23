#!/usr/bin/env python3
# Fake GPS for testing WaypointFollower without real hardware.
# Listens to /cmd_vel, moves a simulated position based on it, publishes that
# position back out as /gps_data -- closing the loop entirely in software.

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from cross_pkg_messages.msg import GPSData

EARTH_RADIUS_M = 6371000.0  # same constant WaypointFollower uses


class FakeGPS(Node):
    def __init__(self):
        super().__init__('FakeGPS')

        # Default start point: Mars Desert Research Station, Hanksville UT.
        self.declare_parameter('start_lat', 38.4061)
        self.declare_parameter('start_lon', -110.7918)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.start_lat = self.get_parameter('start_lat').value
        self.start_lon = self.get_parameter('start_lon').value
        rate_hz = self.get_parameter('publish_rate_hz').value

        # Simulated position, as meters offset from the start point (east/north),
        # plus a compass heading in degrees (0 = north, clockwise-positive --
        # matching WaypointFollower's bearing convention, not ROS's usual math angle).
        self.x_east_m = 0.0
        self.y_north_m = 0.0
        self.heading_deg = 0.0

        # Latest drive command received; (0, 0) until WaypointFollower sends one.
        self.linear_x = 0.0
        self.angular_z = 0.0

        self.gps_pub = self.create_publisher(GPSData, 'gps_data', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.dt = 1.0 / rate_hz
        self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f'FakeGPS running, starting at ({self.start_lat}, {self.start_lon})')

    def cmd_vel_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def tick(self):
        # angular.z follows ROS's counterclockwise-positive convention (turning
        # left = positive), but our heading is compass-style (clockwise-positive),
        # so a positive angular.z must DECREASE heading_deg, not increase it.
        self.heading_deg -= math.degrees(self.angular_z) * self.dt
        self.heading_deg %= 360.0

        heading_rad = math.radians(self.heading_deg)
        self.x_east_m += self.linear_x * math.sin(heading_rad) * self.dt
        self.y_north_m += self.linear_x * math.cos(heading_rad) * self.dt

        # Convert the local meters offset back into lat/lon -- inverse of the
        # haversine relationship WaypointFollower uses, including the same
        # cos(latitude) correction for longitude.
        lat = self.start_lat + math.degrees(self.y_north_m / EARTH_RADIUS_M)
        lon = self.start_lon + math.degrees(
            self.x_east_m / (EARTH_RADIUS_M * math.cos(math.radians(self.start_lat))))

        msg = GPSData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = 1  # arbitrary "valid fix" marker, no real meaning in sim
        msg.lla.x = lat
        msg.lla.y = lon
        msg.lla.z = 0.0
        msg.speed = self.linear_x
        msg.course = self.heading_deg
        msg.sats = 8
        self.gps_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeGPS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
