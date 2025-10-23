from builtin_interfaces.msg import Time


def timestamp_from_unix(ts: float) -> Time:
    """
    Convert a UNIX timestamp (seconds since 1970) to ROS Time.
    """
    sec = int(ts)
    nanosec = int((ts - sec) * 1e9)
    return Time(sec=sec, nanosec=nanosec)
