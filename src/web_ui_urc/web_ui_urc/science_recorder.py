#!/usr/bin/env python3
"""
Science snapshot recorder: saves the latest Science readings when the web page asks.

Runs on: the base station.
Launched by: web_ui_urc/launch/web_ui_launch.py (run mode `web_ui`).

Subscribes to:
  /science/weight_g (std_msgs/Float32) - sample weight in grams
  /science/soil/<name> (std_msgs/Float32) - the 7 soil-probe values
  /science/spectrometer/intensity (std_msgs/Float32MultiArray) - raw intensity per pixel
  /science/camera/image_raw/compressed (sensor_msgs/CompressedImage) - JPEG science camera frame
  /web_ui/save_snapshot (std_msgs/String) - request from the web page; data is an optional note

Publishes:
  /web_ui/snapshot_status (std_msgs/String) - one line saying what was saved (shown on the page)

Each snapshot writes into output_dir (default /root/science_data, which launchScript.sh mounts
from ./ground_station_volume/science_data on the base station):
  readings.csv              - one row per snapshot: time, note, weight, and soil values
  <time>_spectrum.csv       - pixel and intensity columns, if a spectrum has arrived
  <time>_camera.jpg         - the latest camera frame, if one has arrived

How it connects to the system:
  - The Science Raspberry Pi publishes the /science/* topics (science_urc)
  - The Science page's "Save snapshot" button publishes /web_ui/save_snapshot through rosbridge
"""

import csv
from datetime import datetime
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Float32MultiArray, String

# Soil-probe topic names under /science/soil/, in the order they appear in readings.csv
SOIL_FIELDS = ['moisture', 'temperature', 'ph', 'conductivity', 'nitrogen', 'phosphorus',
               'potassium']

# Column names for readings.csv (units match the science_urc soil probe driver)
CSV_HEADER = (['time', 'note', 'weight_g', 'moisture_pct', 'temperature_c', 'ph',
               'conductivity_us_cm', 'nitrogen_mg_kg', 'phosphorus_mg_kg', 'potassium_mg_kg'])


class ScienceRecorder(Node):
    """Keep the latest value of each Science topic and write them out on request."""

    def __init__(self):
        """
        Set up subscriptions and the output folder.

        Steps:
          1. Reading the output folder parameter and creating it if needed
          2. Subscribing to every Science topic, storing only the newest message
          3. Subscribing to the save request and creating the status publisher
        """
        super().__init__('science_recorder')

        self.declare_parameter('output_dir', '/root/science_data')
        self.output_dir = self.get_parameter('output_dir').value
        os.makedirs(self.output_dir, exist_ok=True)

        # Newest value seen on each topic (None until the first message arrives)
        self.latest = {'weight_g': None, **{name: None for name in SOIL_FIELDS}}
        self.latest_spectrum = None
        self.latest_jpeg = None

        # System: published by weight_sensor_node on the Science Pi
        self.create_subscription(Float32, '/science/weight_g', self.store('weight_g'), 10)

        # System: published by soil_probe_node on the Science Pi (one topic per value)
        for name in SOIL_FIELDS:
            self.create_subscription(Float32, f'/science/soil/{name}', self.store(name), 10)

        # System: published by spectrometer_node on the Science Pi
        self.create_subscription(Float32MultiArray, '/science/spectrometer/intensity',
                                 self.on_spectrum, 10)

        # System: published by camera_node on the Science Pi
        self.create_subscription(CompressedImage, '/science/camera/image_raw/compressed',
                                 self.on_camera, 10)

        # System: published by the web page's "Save snapshot" button via rosbridge
        self.create_subscription(String, '/web_ui/save_snapshot', self.on_save, 10)
        self.status_publisher = self.create_publisher(String, '/web_ui/snapshot_status', 10)

        self.get_logger().info(f'Saving science snapshots to {self.output_dir}')

    def store(self, key):
        """
        Make a callback that saves a Float32 message's value under the given key.

        Args:
            key: name in self.latest to overwrite.

        Returns:
            A subscription callback function.
        """
        def callback(message):
            self.latest[key] = message.data
        return callback

    def on_spectrum(self, message):
        """Keep the newest spectrometer intensity list."""
        self.latest_spectrum = list(message.data)

    def on_camera(self, message):
        """Keep the newest JPEG bytes from the science camera."""
        self.latest_jpeg = bytes(message.data)

    def on_save(self, message):
        """
        Write a snapshot and publish a one-line status.

        Args:
            message: std_msgs/String whose data is an optional note typed on the page.

        Steps:
          1. Appending one row to readings.csv (writing the header first if the file is new)
          2. Writing the spectrum CSV and camera JPEG if they have arrived
          3. Publishing what was saved, or the error, to /web_ui/snapshot_status
        """
        stamp = datetime.now()
        prefix = stamp.strftime('%Y%m%d-%H%M%S')
        saved = []
        try:
            readings_path = os.path.join(self.output_dir, 'readings.csv')
            is_new = not os.path.exists(readings_path)
            # Syntax: newline='' stops the csv module from adding blank lines on some systems
            with open(readings_path, 'a', newline='') as file:
                writer = csv.writer(file)
                if is_new:
                    writer.writerow(CSV_HEADER)
                # Leaving a cell empty when that value has never arrived
                row = [stamp.isoformat(timespec='seconds'), message.data.strip()]
                row += ['' if self.latest[key] is None else round(self.latest[key], 3)
                        for key in ['weight_g'] + SOIL_FIELDS]
                writer.writerow(row)
            saved.append('readings')

            if self.latest_spectrum is not None:
                spectrum_path = os.path.join(self.output_dir, f'{prefix}_spectrum.csv')
                with open(spectrum_path, 'w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(['pixel', 'intensity'])
                    writer.writerows(enumerate(self.latest_spectrum))
                saved.append('spectrum')

            if self.latest_jpeg is not None:
                with open(os.path.join(self.output_dir, f'{prefix}_camera.jpg'), 'wb') as file:
                    file.write(self.latest_jpeg)
                saved.append('photo')

            status = f'Saved {", ".join(saved)} at {stamp.strftime("%H:%M:%S")}'
        except OSError as exc:
            status = f'Save failed: {exc}'
            self.get_logger().error(status)

        self.status_publisher.publish(String(data=status))
        self.get_logger().info(status)


def main():
    """Start the node and keep it running until Ctrl+C."""
    rclpy.init()
    node = ScienceRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
