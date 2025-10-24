import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from mobile_navigation.sources.factory import SourceFactory


class GPSIMUNode(Node):
    """
    ROS 2 node that publishes GPS and IMU messages from a configurable mobile data source.
    The publishing rate matches the rate at which the data source provides messages.

    Publishes
    --------
    /gps : sensor_msgs/NavSatFix
        GPS coordinates in WGS84 (latitude, longitude, altitude).
    /imu : sensor_msgs/Imu
        IMU orientation (quaternion) and linear/angular acceleration.
    """

    def __init__(self) -> None:
        super().__init__("gps_imu_node")

        # Declare parameters
        self.declare_parameter("debug", False)
        self.declare_parameter("host", None)
        self.declare_parameter("port", None)
        self.declare_parameter("gps_topic", "/gps")
        self.declare_parameter("imu_topic", "/imu")

        # Get parameters
        self.debug: bool = self.get_parameter("debug").value
        self.host: str = self.get_parameter("host").value
        self.port: int = self.get_parameter("port").value
        self.gps_topic: str = self.get_parameter("gps_topic").value
        self.imu_topic: str = self.get_parameter("imu_topic").value

        # Create publishers
        self.gps_pub = self.create_publisher(NavSatFix, self.gps_topic, 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)

        # Create the data source via factory (example: SensorLog)
        self.source = SourceFactory.create_source("sensorlog", (self.host, self.port))

        # Start a background thread that publishes as fast as data arrives
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()

    def _publish_loop(self) -> None:
        """Continuously reads data from source and publishes ROS messages."""
        while rclpy.ok():
            gps_msg, imu_msg = self.source.get_next()
            if gps_msg and imu_msg:
                self.gps_pub.publish(gps_msg)
                self.imu_pub.publish(imu_msg)

                if self.debug:
                    self.get_logger().info(
                        f"Received GPS: lat={gps_msg.latitude:.6f}, lon={gps_msg.longitude:.6f}, alt={gps_msg.altitude:.2f} | "
                        f"IMU: ax={imu_msg.linear_acceleration.x:.2f}, ay={imu_msg.linear_acceleration.y:.2f}, az={imu_msg.linear_acceleration.z:.2f}"
                    )


def main(args=None):
    rclpy.init(args=args)
    node = GPSIMUNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
