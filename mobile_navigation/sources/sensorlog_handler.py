import socket
import json
import re
from typing import List, Tuple
from sensor_msgs.msg import NavSatFix, Imu
from mobile_navigation.interfaces.sensorlog_parser import GNSS, IMU


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

        if not isinstance(host, str):
            raise ValueError(f"'host' must be of type str, not {type(host).__name__}")

        if not isinstance(port, int):
            raise ValueError(f"'port' must be of type int, not {type(port).__name__}")

        self.host: str = host
        self.port: int = port
        self.sock: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))
        self.buffer: str = ""

    def get_next(self) -> Tuple[List[NavSatFix], List[Imu]]:
        """
        Reads from a TCP socket, parses SensorLog JSON, and returns ROS messages.

        This function blocks until at least one valid GNSS or IMU message is extracted
        from the buffer. It can handle multiple JSON objects received in a single
        socket read.

        Returns
        -------
        gps_msg_list : List[NavSatFix]
            A list of GNSS messages extracted from the current buffer.
        imu_msg_list : List[Imu]
            A list of IMU messages extracted from the current buffer.

        Notes
        -----
        - The function will continue reading from the socket until at least one valid
        message is received.
        - IMU and GNSS timestamps are taken directly from the SensorLog JSON.
        - If multiple messages are received at once, all valid messages are returned
        in the respective lists.
        """
        while True:
            # Receive data from TCP socket
            data: str = self.sock.recv(4096).decode("utf-8")
            if not data:
                continue  # keep waiting until we get data

            self.buffer += data

            gps_msg_list: List[NavSatFix] = []
            imu_msg_list: List[Imu] = []

            json_objects = list(re.finditer(r"\{.*?\}", self.buffer, re.DOTALL))
            for match in json_objects:
                obj_str = match.group()
                _, end = match.span()
                json_object = json.loads(obj_str)

                if GNSS.is_valid_message(json_object):
                    gps_msg_list.append(GNSS(**json_object).to_ros_message())
                if IMU.is_valid_message(json_object):
                    imu_msg_list.append(IMU(**json_object).to_ros_message())

                # Remove exactly this object from the buffer
                self.buffer = self.buffer[end:]

            # Only return if at least one valid message was found
            if gps_msg_list or imu_msg_list:
                return gps_msg_list, imu_msg_list
