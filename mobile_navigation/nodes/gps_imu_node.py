import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistStamped
from mobile_navigation.sources.factory import SourceFactory
from rclpy.parameter import Parameter


class GPSIMUNode(Node):
    """
    ROS 2 node that publishes GPS and IMU messages from a configurable mobile data source.
    The publishing rate matches the rate at which the data source provides messages.

    Publishes
    --------
    /mobile_navigation/gps : sensor_msgs/NavSatFix
        GPS coordinates in WGS84 (latitude, longitude, altitude).
    /mobile_navigation/imu : sensor_msgs/Imu
        IMU orientation (quaternion) and linear/angular acceleration.
    /mobile_navigation/twist : geometry_msgs.msg
        Linear velocity and angular rotation
    """

    def __init__(self) -> None:
        super().__init__("gps_imu_node")

        # Declare parameters
        self.declare_parameter("debug", False)
        self.declare_parameter("host", Parameter.Type.STRING)
        self.declare_parameter("port", Parameter.Type.INTEGER)
        self.declare_parameter("gps_topic", "/mobile_navigation/gps")
        self.declare_parameter("imu_topic", "/mobile_navigation/imu")
        self.declare_parameter("twist_topic", "/mobile_navigation/twist")

        # Get parameters
        self.debug: bool = self.get_parameter("debug").get_parameter_value().bool_value
        self.host: str = self.get_parameter("host").get_parameter_value().string_value
        self.port: int = self.get_parameter("port").get_parameter_value().integer_value
        self.gps_topic: str = (
            self.get_parameter("gps_topic").get_parameter_value().string_value
        )
        self.imu_topic: str = (
            self.get_parameter("imu_topic").get_parameter_value().string_value
        )
        self.twist_topic: str = (
            self.get_parameter("twist_topic").get_parameter_value().string_value
        )

        # Validate
        if not self.host or not self.port:
            raise RuntimeError("You must provide valid 'host' and 'port' parameters")

        # Create publishers
        self.gps_pub = self.create_publisher(NavSatFix, self.gps_topic, 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.twist_pub = self.create_publisher(TwistStamped, self.twist_topic, 10)

        # Create the data source via factory (example: SensorLog)
        self.source = SourceFactory.create_source("sensorlog", (self.host, self.port))

        # Start a background thread that publishes as fast as data arrives
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()

    def _publish_loop(self) -> None:
        """
        Continuously reads data from the source and publishes ROS messages.

        This loop blocks until at least one valid message is received from the source,
        then publishes all GNSS and IMU messages extracted in the current batch.
        Useful for streaming SensorLog data over TCP and feeding ROS topics.

        Returns
        -------
        None

        Notes
        -----
        - The function will block on `self.source.get_next()` until at least one
        valid message is available.
        - Publishes each GNSS message to `self.gps_pub` and each IMU message to
        `self.imu_pub`.
        - If `self.debug` is True, prints human-readable information about each
        message to the ROS logger.
        - Designed to run inside a ROS 2 node while `rclpy.ok()` is True.
        """
        while rclpy.ok():
            gps_msgs, imu_msgs, twist_msgs = self.source.get_next()

            for gps_msg in gps_msgs:
                self.gps_pub.publish(gps_msg)
                if self.debug:
                    stamp = gps_msg.header.stamp
                    timestamp_sec = stamp.sec + stamp.nanosec * 1e-9
                    self.get_logger().info(
                        f"GPS: lat={gps_msg.latitude:.6f}, "
                        f"lon={gps_msg.longitude:.6f}, alt={gps_msg.altitude:.2f}, timestamp={timestamp_sec:.6f}"
                    )

            for imu_msg in imu_msgs:
                self.imu_pub.publish(imu_msg)
                if self.debug:
                    stamp = imu_msg.header.stamp
                    timestamp_sec = stamp.sec + stamp.nanosec * 1e-9
                    self.get_logger().info(
                        f"IMU: ax={imu_msg.linear_acceleration.x:.2f}, "
                        f"ay={imu_msg.linear_acceleration.y:.2f}, "
                        f"az={imu_msg.linear_acceleration.z:.2f}, "
                        f"timestamp={timestamp_sec:.6f}"
                    )

            for twist_msg in twist_msgs:
                self.twist_pub.publish(twist_msg)
                if self.debug:
                    stamp = twist_msg.header.stamp
                    timestamp_sec = stamp.sec + stamp.nanosec * 1e-9
                    self.get_logger().info(
                        f"TWIST: x={twist_msg.twist.linear.x:.2f}, "
                        f"y={twist_msg.twist.linear.y:.2f}, "
                        f"z={twist_msg.twist.linear.z:.2f}, "
                        f"timestamp={timestamp_sec:.6f}"
                    )


def main(args=None):
    rclpy.init(args=args)
    node = GPSIMUNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
