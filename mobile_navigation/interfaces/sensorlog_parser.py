from typing import Any
import math
from pydantic import BaseModel
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistStamped
from mobile_navigation.utils.time_utils import seconds_to_ros_time
from mobile_navigation.utils.coordinate_frames import ios_to_enu


class IMU(BaseModel):
    # IMU
    motionQuaternionW: float
    motionQuaternionX: float
    motionQuaternionY: float
    motionQuaternionZ: float
    motionRotationRateX: float
    motionRotationRateY: float
    motionRotationRateZ: float
    motionUserAccelerationX: float
    motionUserAccelerationY: float
    motionUserAccelerationZ: float
    motionTimestamp_sinceReboot: float
    motionAttitudeReferenceFrame: str

    @classmethod
    def is_valid_message(cls, message: Any):
        if not isinstance(message, dict):
            return False
        return set(cls.__fields__.keys()).issubset(message.keys())

    def to_ros_message(self) -> Imu:
        """
        Convert the iOS IMU data to a ROS 2 `sensor_msgs/Imu` message in ENU frame.

        This method performs the following transformations:
        1. Converts the iOS quaternion (xTrueNorthZVertical) to ROS ENU quaternion using `ios_to_enu`.
        2. Swaps the angular velocity axes to match ROS ENU:
        - iOS X → ENU Y
        - iOS Y → ENU X
        - iOS Z → ENU Z
        3. Swaps the linear acceleration axes to match ROS ENU:
        - iOS X → ENU Y
        - iOS Y → ENU X
        - iOS Z → ENU Z
        4. Sets the message header with the IMU timestamp and frame ID `"imu_link"`.

        Coordinate frames:
        ------------------
        iOS xTrueNorthZVertical:
            X → points toward True North
            Y → points West
            Z → points Up

        ROS ENU:
            X → East
            Y → North
            Z → Up

        Returns
        -------
        sensor_msgs.msg.Imu
            A ROS 2 IMU message with orientation, angular velocity, and linear acceleration
            expressed in the ENU coordinate frame.
        """

        if self.motionAttitudeReferenceFrame != "XTrueNorthZVertical":
            raise ValueError(
                f"'motionAttitudeReferenceFrame' needs to be 'XTrueNorthZVertical', got '{self.motionAttitudeReferenceFrame}'"
            )

        imu_msg = Imu()
        # Convert to ENU coordinate frame
        enu = ios_to_enu(
            q_ios=[
                self.motionQuaternionX,
                self.motionQuaternionY,
                self.motionQuaternionZ,
                self.motionQuaternionW,
            ]
        )
        imu_msg.orientation.w = enu[3]
        imu_msg.orientation.x = enu[0]
        imu_msg.orientation.y = enu[1]
        imu_msg.orientation.z = enu[2]
        # We swap the rotation axis since we want the output in ENU coordinate frame
        imu_msg.angular_velocity.x = self.motionRotationRateY
        imu_msg.angular_velocity.y = self.motionRotationRateX
        imu_msg.angular_velocity.z = self.motionRotationRateZ
        # We swap the rotation axis since we want the output in ENU coordinate frame
        imu_msg.linear_acceleration.x = self.motionUserAccelerationY
        imu_msg.linear_acceleration.y = self.motionUserAccelerationX
        imu_msg.linear_acceleration.z = self.motionUserAccelerationZ
        imu_msg.header.stamp = seconds_to_ros_time(self.motionTimestamp_sinceReboot)
        imu_msg.header.frame_id = "imu_link"
        return imu_msg


class GNSS(BaseModel):
    # GNSS
    locationLatitude: float
    locationLongitude: float
    locationAltitude: float
    locationHorizontalAccuracy: float
    locationVerticalAccuracy: float
    locationTimestamp_since1970: float

    @classmethod
    def is_valid_message(cls, message: Any):
        if not isinstance(message, dict):
            return False
        return set(cls.__fields__.keys()).issubset(message.keys())

    def to_ros_message(self) -> NavSatFix:
        gps_msg = NavSatFix()
        gps_msg.latitude = self.locationLatitude
        gps_msg.longitude = self.locationLongitude
        gps_msg.altitude = self.locationAltitude
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        cov = self.locationHorizontalAccuracy**2
        gps_msg.position_covariance = [
            cov,
            0,
            0,
            0,
            cov,
            0,
            0,
            0,
            self.locationVerticalAccuracy**2,
        ]
        gps_msg.header.stamp = seconds_to_ros_time(self.locationTimestamp_since1970)
        gps_msg.header.frame_id = "gps"
        return gps_msg


class TWIST(BaseModel):
    # Twist
    locationTimestamp_since1970: float
    locationTrueHeading: float
    locationSpeed: float
    motionRotationRateX: float
    motionRotationRateY: float
    motionRotationRateZ: float
    motionAttitudeReferenceFrame: str

    @classmethod
    def is_valid_message(cls, message: Any):
        if not isinstance(message, dict):
            return False
        return set(cls.__fields__.keys()).issubset(message.keys())

    def to_ros_message(self) -> TwistStamped:
        """
        Convert iOS IMU and GPS-derived data to a ROS 2 `geometry_msgs/TwistStamped` message in ENU frame.

        This method performs the following transformations:
        1. Swaps angular velocity axes from iOS xTrueNorthZVertical to ROS ENU:
        - iOS X → ENU Y
        - iOS Y → ENU X
        - iOS Z → ENU Z
        2. Converts heading (from GPS) and speed into linear velocity in ROS ENU:
        - X (East) = speed * sin(heading)
        - Y (North) = speed * cos(heading)
        - Z (Up) = 0 (no vertical speed)
        3. Sets the message header with the GPS/IMU timestamp and frame ID `"map"`.

        Coordinate frames:
        ------------------
        iOS xTrueNorthZVertical:
            X → points toward True North
            Y → points West
            Z → points Up

        ROS ENU:
            X → East
            Y → North
            Z → Up

        Notes
        -----
        - `locationTrueHeading` is assumed in degrees from North, clockwise.
        - If heading or speed is invalid (-1), linear velocity is set to zero.

        Returns
        -------
        geometry_msgs.msg.TwistStamped
            A ROS 2 TwistStamped message with linear and angular velocities expressed in ENU.
        """

        if self.motionAttitudeReferenceFrame != "XTrueNorthZVertical":
            raise ValueError(
                f"'motionAttitudeReferenceFrame' needs to be 'XTrueNorthZVertical', got '{self.motionAttitudeReferenceFrame}'"
            )

        msg = TwistStamped()

        msg.header.stamp = seconds_to_ros_time(self.locationTimestamp_since1970)
        msg.header.frame_id = "map"
        # We swap the rotation axis since we want the output in ENU coordinate frame
        msg.twist.angular.x = self.motionRotationRateY
        msg.twist.angular.y = self.motionRotationRateX
        msg.twist.angular.z = self.motionRotationRateZ

        if self.locationTrueHeading != -1 and self.locationSpeed != -1:
            heading_rad = math.radians(self.locationTrueHeading)
            # Heading in ENU
            msg.twist.linear.x = math.sin(heading_rad) * self.locationSpeed
            msg.twist.linear.y = math.cos(heading_rad) * self.locationSpeed
            msg.twist.linear.z = 0.0
        else:
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0

        return msg
