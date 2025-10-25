from builtin_interfaces.msg import Time


def timestamp_from_unix(ts: float) -> Time:
    """
    Convert a UNIX timestamp (seconds since 1970) to a ROS 2 Time message.

    Parameters
    ----------
    ts : float
        UNIX timestamp in seconds (may include fractional seconds).

    Returns
    -------
    builtin_interfaces.msg.Time
        ROS 2 Time message with seconds and nanoseconds fields set.
    """
    sec = int(ts)
    nanosec = int((ts - sec) * 1e9)
    return Time(sec=sec, nanosec=nanosec)


def relative_to_ros_time(seconds_relative: float) -> Time:
    """
    Convert a time relative to an arbitrary start (seconds) to a ROS 2 Time message.

    Parameters
    ----------
    seconds_relative : float
        Time in seconds relative to some reference point (e.g., system boot).

    Returns
    -------
    builtin_interfaces.msg.Time
        ROS 2 Time message representing the relative time.
    """
    t = Time()
    t.sec = int(seconds_relative)
    t.nanosec = int((seconds_relative % 1) * 1e9)
    return t


def seconds_to_ros_time(seconds: float) -> Time:
    """
    Convert a float timestamp in seconds to a ROS 2 Time message.

    Parameters
    ----------
    seconds : float
        Timestamp in seconds, can include fractional seconds.

    Returns
    -------
    builtin_interfaces.msg.Time
        ROS 2 Time message with corresponding seconds and nanoseconds.
    """
    t = Time()
    t.sec = int(seconds)
    t.nanosec = int((seconds % 1) * 1e9)
    return t
