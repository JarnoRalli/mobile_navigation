"""
gps_visualizer.py

A ROS 2 node that visualizes live GPS coordinates in a web browser.

This module combines a ROS 2 subscriber node with a Flask-SocketIO server
to provide real-time GPS visualization on a Leaflet map. The node subscribes
to the GPS topic (`/mobile_navigation/gps` by default, publishing
`sensor_msgs/msg/NavSatFix`), and emits updates to a web page. The web page
displays the current position as a marker and draws a trail of all received
GPS points.

Usage:
    ros2 run mobile_navigation gps_visualizer

Dependencies:
    - rclpy
    - sensor_msgs
    - flask
    - flask-socketio
"""

import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from flask import Flask, render_template_string
from flask_socketio import SocketIO

# ======================
# Flask + SocketIO setup
# ======================

app: Flask = Flask(__name__)
# Use 'threading' async_mode to avoid conflicts with ROS 2 spinning
socketio: SocketIO = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# HTML page with Leaflet map
HTML_PAGE: str = """
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <title>Live GPS</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css" />
  <script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>
  <script src="https://cdn.socket.io/4.7.4/socket.io.min.js"></script>
  <style>
    html, body, #map { height: 100%; margin: 0; }
  </style>
</head>
<body>
  <div id="map"></div>
  <script>
    const socket = io();
    let map = L.map('map').setView([60.1699, 24.9384], 16);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19
    }).addTo(map);
    let marker = L.marker([0, 0]).addTo(map);
    let path = [];

    socket.on('gps_update', (data) => {
      const { lat, lon } = data;
      marker.setLatLng([lat, lon]);
      map.panTo([lat, lon]);
      // Add to path
      path.push([lat, lon]);
      if (path.length > 1) {
        if (window.polyline) {
          window.polyline.setLatLngs(path);
        } else {
          window.polyline = L.polyline(path, { color: 'red' }).addTo(map);
        }
      }
    });
  </script>
</body>
</html>
"""


@app.route("/")
def index() -> str:
    """
    Serve the main HTML page for the GPS map.

    Returns
    -------
    str
        Rendered HTML page as a string.
    """
    return render_template_string(HTML_PAGE)


def start_flask() -> None:
    """
    Start the Flask-SocketIO server in a background thread.

    This function blocks when called, so it is intended to be run inside a
    separate thread to allow ROS 2 spinning in the main thread.
    """
    socketio.run(app, host="0.0.0.0", port=5000)


def broadcast_gps(lat: float, lon: float) -> None:
    """
    Emit a GPS update to all connected web clients.

    Parameters
    ----------
    lat : float
        Latitude of the GPS position.
    lon : float
        Longitude of the GPS position.
    """
    socketio.emit("gps_update", {"lat": lat, "lon": lon})


class GPSListener(Node):
    """
    ROS 2 node that subscribes to GPS messages and broadcasts them to the web map.

    Subscribes to:
        /mobile_navigation/gps (sensor_msgs.msg.NavSatFix)
    """

    def __init__(self) -> None:
        """
        Initialize the GPSListener node and subscribe to the GPS topic.
        """
        super().__init__("gps_listener")
        self.subscription = self.create_subscription(
            NavSatFix,
            "/mobile_navigation/gps",  # replace with your topic if different
            self.listener_callback,
            10,
        )

    def listener_callback(self, msg: NavSatFix) -> None:
        """
        Callback for incoming GPS messages.

        Parameters
        ----------
        msg : sensor_msgs.msg.NavSatFix
            The incoming GPS message containing latitude and longitude.
        """
        lat: float = msg.latitude
        lon: float = msg.longitude
        self.get_logger().info(f"GPS: {lat}, {lon}")
        broadcast_gps(lat, lon)


def main(args: Optional[list] = None) -> None:
    """
    Main function that starts both the Flask server and the ROS 2 node.

    Parameters
    ----------
    args : list, optional
        Command line arguments passed to rclpy.init.
    """
    # Start Flask in a background thread
    flask_thread = threading.Thread(target=start_flask, daemon=True)
    flask_thread.start()

    # Start ROS 2 node
    rclpy.init(args=args)
    node = GPSListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
