import depthai as dai
import json
import numpy as np
with open ("calib.json", "r") as read_file:
    calib_data = json.load(read_file)
    width = calib_data['width']
    height = calib_data['height']

with dai.Device() as device:
    calib = device.readCalibration()
    r_intrinsics = calib.getCameraIntrinsics(dai.CameraBoardSocket.RIGHT, width, height)
    print('Right mono camera instrinsics:', r_intrinsics)
    l_intrinsics = calib.getCameraIntrinsics(dai.CameraBoardSocket.LEFT, width, height)
    print('Left mono camera instrinsics:', l_intrinsics)

    extrinsics = np.array(calib.getCameraExtrinsics(
        dai.CameraBoardSocket.LEFT,
        dai.CameraBoardSocket.RIGHT
    ))
    
    print('Extrinsics:', extrinsics)
    # Extract R and T
    R = extrinsics[:3, :3]
    T = extrinsics[:3, 3].reshape(3, 1)

    distcoeff_l = np.array(calib.getDistortionCoefficients(dai.CameraBoardSocket.LEFT))
    distcoeff_r = np.array(calib.getDistortionCoefficients(dai.CameraBoardSocket.RIGHT))
    
    
    calib_dict = {
        "width": width,
        "height": height,
        "right_intrinsics": r_intrinsics,
        "left_intrinsics": l_intrinsics,
        "rotation_left_to_right": R.tolist(),
        "translation_left_to_right": T.tolist(),
        "distortion_left": distcoeff_l.tolist(),
        "distortion_right": distcoeff_r.tolist()
    }

    with open("calib.json", "w") as f:
        json.dump(calib_dict, f, indent=4)
