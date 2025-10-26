# iPhone

> ⚠️ **Important:** `mobile_navigation` publishes IMU and Twist in ENU coordinate frame, but the conversions have not been properly tested!

## Device Motion

Reference frame of the device's motion can be chosen using the structure [CMAttitudeReferenceFrame](https://developer.apple.com/documentation/coremotion/cmattitudereferenceframe).
Currently it has the following options:

* [xArbitraryZVertical](https://developer.apple.com/documentation/coremotion/cmattitudereferenceframe/xarbitraryzvertical)
    * A reference frame where the Z axis is vertical and the X axis points in an arbitrary direction in the horizontal plane.
* [xArbitraryCorrectedZVertical](https://developer.apple.com/documentation/coremotion/cmattitudereferenceframe/xarbitrarycorrectedzvertical)
    * A reference frame where the Z axis is vertical and has improved rotation accuracy, and the X axis points in an arbitrary direction in the horizontal plane.
* [xMagneticNorthZVertical](https://developer.apple.com/documentation/coremotion/cmattitudereferenceframe/xmagneticnorthzvertical)
    * A reference frame where the Z axis is vertical and the X axis points to the magnetic north pole.
* [xTrueNorthZVertical](https://developer.apple.com/documentation/coremotion/cmattitudereferenceframe/xtruenorthzvertical)
    * A reference frame where the Z axis is vertical and the X axis points to the geographic north pole.

When a device’s orientation matches the frame of reference’s orientation, the roll, pitch, and yaw rotation values of a reported CMAttitude contain the value 0. As a person rotates the device, the roll, pitch, and yaw values reflect the amount of rotation (in radians) relative to the frame of reference. The following figure shows how to interpret these values around each axis. Rotation values are in the range $-\pi$ to $\pi$.

For more information, take a look at [Getting processed device-motion data](https://developer.apple.com/documentation/coremotion/getting-processed-device-motion-data?language=objc).

<figure align="center">
    <img src="./media-4251993@2x.png" width="400">
    <figcaption>Figure 1. Orientation of device motion data.</figcaption>
</figure>

Figure 1. shows the orientation of the device motion data (source: [Getting processed device-motion data](https://developer.apple.com/documentation/coremotion/getting-processed-device-motion-data?language=objc)).


### Converting xTrueNorthZVertical to ENU

From navigation point of view, `xTrueNorthZVertical` is particularly interesting. Following table shows the relationship between iOS (xTrueNorthZVertical) and ROS ENU:

| Axis | iOS (xTrueNorthZVertical) | ROS ENU |
| -----| --------------------------|---------|
| X    | North                     | East    |
| Y    | West                      | North   |
| Z    | Up                        | Up      |

If the pose is given using quaternions, in xTrueNorthZVertical reference frame, this can be converted into ENU by rotating -90 degrees with respect to the Z-axis:

$q_{rot} = [x,y,z,w] = [0,0,sin(-\pi/4),cos(-\pi/4)]$

, and

$q_{ios} = [x_{ios},y_{ios},z_{ios},w_{ios}]$

, then

$q_{enu} = q_{rot}*q_{ios}$


### Placing the Phone in a Car

You should place the phone so that it lies flat on the dashboard and top of the phone (Y-axis from Figure 1.) points towards where the car is headed.
IMU and twist are converted automatically into ENU coordinate frame.

