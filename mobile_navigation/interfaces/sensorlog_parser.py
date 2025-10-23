from pydantic import BaseModel


class SensorLogData(BaseModel):
    # GPS
    locationLatitude: float
    locationLongitude: float
    locationAltitude: float
    locationHorizontalAccuracy: float
    locationVerticalAccuracy: float
    locationTimestamp_since1970: float

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

    # Optional / extra fields
    motionPitch: float | None = None
    motionRoll: float | None = None
    motionYaw: float | None = None
    deviceID: str | None = None
