from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("mobile_navigation"),
        "config",
        "default_config.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="mobile_navigation",
                executable="gps_imu_node",
                name="gps_imu_node",
                parameters=[config_file],  # load YAML as parameters
            )
        ]
    )
