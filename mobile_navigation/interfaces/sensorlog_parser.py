from typing import Any
from pydantic import BaseModel
from sensor_msgs.msg import NavSatFix, Imu
from mobile_navigation.utils.time_utils import seconds_to_ros_time


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

    @classmethod
    def is_valid_message(cls, message: Any):
        if not isinstance(message, dict):
            return False
        return set(cls.__fields__.keys()).issubset(message.keys())

    def to_ros_message(self) -> Imu:
        imu_msg = Imu()
        imu_msg.orientation.w = self.motionQuaternionW
        imu_msg.orientation.x = self.motionQuaternionX
        imu_msg.orientation.y = self.motionQuaternionY
        imu_msg.orientation.z = self.motionQuaternionZ
        imu_msg.angular_velocity.x = self.motionRotationRateX
        imu_msg.angular_velocity.y = self.motionRotationRateY
        imu_msg.angular_velocity.z = self.motionRotationRateZ
        imu_msg.linear_acceleration.x = self.motionUserAccelerationX
        imu_msg.linear_acceleration.y = self.motionUserAccelerationY
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
