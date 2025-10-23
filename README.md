# Mobile Navigation

[![License](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg)](LICENSE)

`mobile_navigation` is a ROS 2 package that streams **GPS and IMU data from mobile devices** (e.g., iPhone, Android) using applications like SensorLog. It provides ROS 2 `NavSatFix` and `Imu` messages that can be integrated into other ROS 2 projects for localization, navigation, and sensor fusion.

---

## Features

- Streams **GNSS (GPS) and IMU** data from mobile devices over TCP.
- Parses JSON messages from SensorLog or other supported applications.
- Publishes ROS 2 topics:
  - `/gps` → [`sensor_msgs/NavSatFix`](https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html)
  - `/imu` → [`sensor_msgs/Imu`](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html)
- Maintains accurate timestamps:
- Maintains proper timestamps:
  - GPS messages use device-provided UNIX timestamps.
  - IMU messages are aligned with GPS via device boot time or fallback to ROS time.
- Modular design with **source factories** to support multiple apps or devices.
- Optional **debug mode** for logging received data.

---

## Installation

1. Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone git@github.com:JarnoRalli/mobile_navigation.git
```

2. Install dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace

```bash
source install/setup.bash
```

---

## Setup on iPhone (SensorLog)

1. Install `SensorLog` from the App Store.
2. Open the app, go to `Configuration` and set `Log Format` to `JSON`.
3. Under `Streaming Settings` enable `log to stream`.
4. Return to the main page and press `Record`.

---

## Run the GPS+IMU node

```bash
ros2 run mobile_navigation gps_imu_node --ros-args -p host:=<IP> -p port:=<PORT> -p debug:=True
```

Set `<IP>` and `<PORT>`to the correct values as seen in `SensorLog`.

---

## ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/gps` | `sensor_msgs/NavSatFix` | Device-reported GNSS coordinates |
| `/imu` | `sensor_msgs/Imu` | Device IMU orientation, angular velocity, and linear acceleration |

