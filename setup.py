from setuptools import find_packages, setup

package_name = "mobile_navigation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/gps_imu.launch.py"]),
        ("share/" + package_name + "/config", ["config/default_config.yaml"]),
    ],
    install_requires=["setuptools", "pydantic>=2.0", "rclpy", "sensor-msgs"],
    zip_safe=True,
    maintainer="Jarno Ralli",
    maintainer_email="jarno@ralli.fi",
    description="ROS 2 package that publishes GPS and IMU data from mobile phones",
    license="BSD-3-Clause",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "gps_imu_node = mobile_navigation.nodes.gps_imu_node:main",
        ],
    },
)
