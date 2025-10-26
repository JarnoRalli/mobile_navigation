from typing import Iterable
import numpy as np
from scipy.spatial.transform import Rotation as R


def ios_to_enu(q_ios: Iterable[float]) -> np.ndarray:
    """
    Convert an iOS quaternion (xTrueNorthZVertical) to ROS ENU frame.

    The iOS `xTrueNorthZVertical` frame uses:
        X = North
        Y = West
        Z = Up

    ROS ENU uses:
        X = East
        Y = North
        Z = Up

    Therefore, the rotation from iOS -> ENU is a -90° rotation about the Z-axis.

    Parameters
    ----------
    q_ios : array-like of shape (4,)
        Quaternion from iOS in the order [x, y, z, w].

    Returns
    -------
    q_enu : np.ndarray of shape (4,)
        Quaternion in ENU frame, ordered [x, y, z, w].

    Examples
    --------
    >>> q_ios = [0.0, 0.0, 0.0, 1.0]  # flat, facing north
    >>> q_enu = ios_to_enu(q_ios)
    >>> np.round(q_enu, 4)
    array([0.    , 0.    , -0.7071, 0.7071])
    """
    # Convert input to SciPy rotation object
    q_ios = R.from_quat(q_ios)

    # Define rotation from iOS -> ENU: -90° about Z-axis
    q_rot = R.from_euler("z", -90, degrees=True)

    # Apply rotation: q_enu = q_rot * q_ios
    q_enu = q_rot * q_ios

    # Return quaternion in [x, y, z, w] order
    return q_enu.as_quat()
