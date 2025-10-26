# Mobile Navigation

[![License](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg)](LICENSE)

`mobile_navigation` is a ROS 2 package for streaming **GPS and IMU data from mobile devices** (e.g., iPhone, Android). It publishes ROS 2 messages (`NavSatFix` and `Imu`) that can be directly used in localization, navigation, sensor fusion, or other robotics applications. While [SensorLog](https://sensorlog.berndthomas.net/) is one supported apps, the package is designed to work with multiple sources that provide GPS/IMU data.

---

## Features

* Streams **GNSS (GPS) and IMU** data from mobile devices.
* Supports multiple applications or devices through a **modular source interface**.
* Parses incoming data and publishes ROS 2 topics:

  * `/mobile_navigation/gps` → [`sensor_msgs/NavSatFix`](https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html)
  * `/mobile_navigation/imu` → [`sensor_msgs/Imu`](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html)
* Optional **debug mode** for logging raw incoming data.
* Compatible with **rosbag recording and playback**.

---

## Installation

1. Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone git@github.com:JarnoRalli/mobile_navigation.git
```

2. Install dependencies:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:

```bash
colcon build --symlink-install
```

4. Source the workspace:

```bash
source install/setup.bash
```

---

## Device Setup

`mobile_navigation` works with any mobile application that can provide GPS and IMU data. Configuration may vary depending on the app and device.

### Example: SensorLog on iPhone

For more information regarding the coordinate system frames used in iPhone/SensorLog, take a look at the [documentation](./documentation/iPhone/README.md).

1. Install **SensorLog** from the App Store.
2. Configure streaming settings:

   * `Log format`: `JSON`
   * `Fill missing data with previous data`: `Disabled`
   * `Log to stream`: `Enabled`
   * `Unbiased: user acceleration, etc.`: `Enabled`
   * `Coordinates, speed, altitude, course`: `Enabled`
   * `Sensor Configuration`
      * `auto navi`
      * `best for navi`
      * `X true N` (related to [CMAttitudeReferenceFrame](https://developer.apple.com/documentation/coremotion/cmattitudereferenceframe))
3. Start recording or streaming data to the specified host and port (TCP JSON stream).

---

## Running the ROS 2 Node

### Direct Node Execution

```bash
ros2 run mobile_navigation gps_imu_node --ros-args -p host:=<IP> -p port:=<PORT> -p debug:=True
```

Replace `<IP>` and `<PORT>` with your device’s network configuration if using a streaming source.

### Using Launch Files

Edit the launch file to set the correct `host` and `port`, then run:

```bash
ros2 launch mobile_navigation gps_imu.launch.py
```

---

## Working with Rosbags

### Recording

Launch the node:

```bash
ros2 launch mobile_navigation gps_imu.launch.py
```

In a separate terminal, record the topics:

```bash
ros2 bag record --topics /mobile_navigation/gps \
/mobile_navigation/imu \
/mobile_navigation/twist \
-o mobile_navigation_bag
```

### Playback

To replay the recorded bag:

```bash
ros2 bag play mobile_navigation_bag
```

To view messages while replaying:

```bash
ros2 topic echo /mobile_navigation/gps &
ros2 topic echo /mobile_navigation/imu &
ros2 topic echo /mobile_navigation/twist &
ros2 bag play mobile_navigation_bag
```

---

## ROS Topics

| Topic                    | Type                    | Description                                                       |
| ------------------------ | ----------------------- | ----------------------------------------------------------------- |
| `/mobile_navigation/gps` | `sensor_msgs/NavSatFix` | Device-reported GNSS coordinates                                  |
| `/mobile_navigation/imu` | `sensor_msgs/Imu`       | Device IMU orientation, angular velocity, and linear acceleration |

---

## Extending to Other Devices

`mobile_navigation` is designed with a **source factory pattern**, allowing new devices or applications to be supported with minimal code changes. To add support for a new data source:

1. Implement a new source class that converts device data to ROS messages.
2. Register the new source in the factory.
3. Run the node specifying your new source type.

---

## License

This project is licensed under the [BSD 3-Clause License](LICENSE).

