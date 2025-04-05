# OakDLiteVIO
Visual Odometry for an Oak-D Lite Camera + MPU6050 IMU
## Calibration - Intrinsics
calib.json contains the intrinsics for L and R cameras on the format:
[fx 0 cx
 0 fy cy
 0 0 1]
 where fx, fy are focal length in pixels and cx, cy are the principal point (optical center / pinhole)