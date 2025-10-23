import socket
import json
import re
from typing import Optional, Tuple
from sensor_msgs.msg import NavSatFix, Imu
from builtin_interfaces.msg import Time
from mobile_navigation.interfaces.sensorlog_parser import SensorLogData
from mobile_navigation.utils.time_utils import timestamp_from_unix


class SensorLogHandler:
    """
    Handles streaming SensorLog JSON data over TCP and converts it to ROS messages.

    Parameters
    ----------
    host : str
        IP address of the SensorLog device.
    port : int
        TCP port of the SensorLog stream.
    """

    def __init__(self, host: str, port: int) -> None:
        self.host: str = host
        self.port: int = port
        self.sock: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))
        self.buffer: str = ""

    def _extract_json_objects(self, data: str):
        """
        Extract all complete JSON objects from a string.

        Parameters
        ----------
        data : str
            Incoming string from TCP socket, may contain multiple JSON objects.

        Returns
        -------
        list[str]
            List of JSON strings (each a complete JSON object).
        """
        pattern = r"\{.*?\}"
        matches = re.findall(pattern, data, re.DOTALL)
        return matches

    def _parse_line(self, line: str) -> Optional[SensorLogData]:
        """
        Parses a single JSON line into a SensorLogData object.

        Parameters
        ----------
        line : str
            JSON string representing one SensorLog message.

        Returns
        -------
        SensorLogData or None
            Validated SensorLogData object, or None if parsing fails.
        """
        try:
            json_obj = json.loads(line)
            return SensorLogData(**json_obj)
        except (json.JSONDecodeError, ValueError) as e:
            print(f"JSON parse/validation error: {e}")
            return None

    def get_next(self) -> Tuple[Optional[NavSatFix], Optional[Imu]]:
        """
        Reads from TCP socket, parses SensorLog JSON, and returns ROS messages.

        Blocks until a valid JSON message is received.
        """

        device_boot_time: float | None = None

        while True:
            # Receive data from TCP socket
            data: str = self.sock.recv(4096).decode("utf-8")
            if not data:
                return None, None
            self.buffer += data

            # Extract all complete JSON objects
            json_objects = self._extract_json_objects(self.buffer)
            for obj_str in json_objects:
                sensor_data = self._parse_line(obj_str)
                if sensor_data is None:
                    # Skip invalid objects
                    continue

                # --- Compute device boot time (once, using first GPS sample) ---
                if (
                    device_boot_time is None
                    and sensor_data.locationTimestamp_since1970 > 0
                ):
                    device_boot_time = (
                        sensor_data.locationTimestamp_since1970
                        - sensor_data.motionTimestamp_sinceReboot
                    )

                # --- Construct ROS messages ---
                gps_msg = NavSatFix()
                gps_msg.latitude = sensor_data.locationLatitude
                gps_msg.longitude = sensor_data.locationLongitude
                gps_msg.altitude = sensor_data.locationAltitude
                gps_msg.position_covariance_type = (
                    NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                )
                cov = sensor_data.locationHorizontalAccuracy**2
                gps_msg.position_covariance = [
                    cov,
                    0,
                    0,
                    0,
                    cov,
                    0,
                    0,
                    0,
                    sensor_data.locationVerticalAccuracy**2,
                ]
                gps_msg.header.stamp = timestamp_from_unix(
                    sensor_data.locationTimestamp_since1970
                )
                gps_msg.header.frame_id = "gps"

                imu_msg = Imu()
                imu_msg.orientation.w = sensor_data.motionQuaternionW
                imu_msg.orientation.x = sensor_data.motionQuaternionX
                imu_msg.orientation.y = sensor_data.motionQuaternionY
                imu_msg.orientation.z = sensor_data.motionQuaternionZ
                imu_msg.angular_velocity.x = sensor_data.motionRotationRateX
                imu_msg.angular_velocity.y = sensor_data.motionRotationRateY
                imu_msg.angular_velocity.z = sensor_data.motionRotationRateZ
                imu_msg.linear_acceleration.x = sensor_data.motionUserAccelerationX
                imu_msg.linear_acceleration.y = sensor_data.motionUserAccelerationY
                imu_msg.linear_acceleration.z = sensor_data.motionUserAccelerationZ

                if device_boot_time is not None:
                    imu_unix_time = (
                        device_boot_time + sensor_data.motionTimestamp_sinceReboot
                    )
                    imu_msg.header.stamp = Time(
                        sec=int(imu_unix_time), nanosec=int((imu_unix_time % 1) * 1e9)
                    )
                else:
                    # fallback to ROS time if GPS not yet received
                    from rclpy.clock import Clock
                    from rclpy.clock import ClockType

                    now = Clock(clock_type=ClockType.SYSTEM_TIME).now().to_msg()
                    imu_msg.header.stamp = now

                imu_msg.header.frame_id = "imu_link"

                # Remove parsed JSON from buffer
                self.buffer = self.buffer.replace(obj_str, "", 1)

                return gps_msg, imu_msg
