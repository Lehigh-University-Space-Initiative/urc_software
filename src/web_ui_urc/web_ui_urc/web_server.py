#!/usr/bin/env python3
"""
Static web server for the rover's browser interface.

Runs on: the base station.
Launched by: web_ui_urc/launch/web_ui_launch.py (run mode `web_ui`).

Serves the files in web/ (index.html, app.js, style.css, vendor/roslib) over HTTP, by default on
port 8000, so a Steam Deck or laptop on the same network can open http://<base-station-ip>:8000.

How it connects to the system:
  - The page itself talks to ROS through rosbridge (WebSocket, port 9090) and loads camera video
    from web_video_server (HTTP, port 8080); both are started by the same launch file
  - This node only hands out the page files; it never touches ROS topics
"""

import functools
import os
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

# Syntax: ament_index_python finds where colcon installed this package's share/ folder
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node


class QuietHandler(SimpleHTTPRequestHandler):
    """Serve files without printing a log line for every request."""

    def log_message(self, format, *args):
        """Drop per-request logs (the page polls nothing, but images and scripts add up)."""
        pass

    def end_headers(self):
        """Tell browsers not to cache, so a rebuilt page shows up on the next reload."""
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()


class WebServer(Node):
    """Run an HTTP server for the web page in a background thread."""

    def __init__(self):
        """
        Read parameters and start serving.

        Steps:
          1. Reading the port and the folder to serve (defaults to this package's web/ folder)
          2. Starting a threaded HTTP server so several browsers can load the page at once
        """
        super().__init__('web_ui_server')

        self.declare_parameter('port', 8000)
        self.declare_parameter('web_root', '')

        port = int(self.get_parameter('port').value)
        web_root = self.get_parameter('web_root').value or os.path.join(
            get_package_share_directory('web_ui_urc'), 'web')

        # Syntax: functools.partial fixes the handler's "directory" argument for every request
        handler = functools.partial(QuietHandler, directory=web_root)
        self.server = ThreadingHTTPServer(('0.0.0.0', port), handler)

        # Running the server in a daemon thread so rclpy.spin() keeps the node responsive
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.get_logger().info(f'Web UI at http://<this-computer>:{port} (serving {web_root})')

    def destroy_node(self):
        """Stop the HTTP server before the node goes away."""
        self.server.shutdown()
        self.server.server_close()
        return super().destroy_node()


def main():
    """Start the node and keep it running until Ctrl+C."""
    rclpy.init()
    node = WebServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
