# OakDLiteVIO
Visual Odometry for an Oak-D Lite Camera + MPU6050 IMU
## Calibration - Intrinsics
calib.json contains the intrinsics for L and R cameras on the format:
[fx 0 cx
 0 fy cy
 0 0 1]
 where fx, fy are focal length in pixels and cx, cy are the principal point (optical center / pinhole)

## VO Pipeline:
### Publish stereo images in DDS topic with timestamps at around 30hz
### Read topic for stereo images and create ORB detections
### Find matching features in both cameras, perform triangulation
### Publish ORB features prevalent in both cameras with 2D camera location and 3D position
### Solve PnP for ORB features, publish estimated Camera pose in 10hz


## VIO EKF Pipeline:
### Estimate inertial pose in prediction step at 100hz from MPU6050
### Use camera pose in correction step at 10hz.


## Lack:
### Camera - IMU extrinsics