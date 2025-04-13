import cv2
import numpy as np
import paho.mqtt.client as mqtt
import json
import base64

TOPIC_LEFT_IMG = "oakd/stereo/left"
TOPIC_RIGHT_IMG = "oakd/stereo/right"
TOPIC_MATCHES = "oakd/stereo/features/matches"

image_buffer = {'left': None, 'right': None, 'timestamp': None}
matches_buffer = None
l_intrinsics = None
l_distcoeffs = None
r_distcoeffs = None
r_intrinsics = None
E = None
R_l_r = None
T_l_r = None

def skew(t):
    return np.array([
        [0, -t[2], t[1]],
        [t[2], 0, -t[0]],
        [-t[1], t[0], 0]
    ])

def calculate_essential_matrix(R, t):
    # Placeholder for essential matrix calculation
    T = skew(t)
    E = T @ R
    return E

with open ("calib.json", "r") as read_file:
    calib_data = json.load(read_file)
    r_intrinsics = np.array(calib_data['right_intrinsics'])
    l_intrinsics = np.array(calib_data['left_intrinsics'])
    R_l_r = np.array(calib_data['rotation_left_to_right'])
    T_l_r = np.array(calib_data['translation_left_to_right'])
    l_distcoeffs = np.array(calib_data['distortion_left'])
    r_distcoeffs = np.array(calib_data['distortion_right'])
    T_l_r = T_l_r.squeeze()
    print(f'Rotation from left to right: {R_l_r.shape}')
    print(f'Translation from left to right: {T_l_r.shape}')
    # Calculate essential matrix
    E = calculate_essential_matrix(R_l_r, T_l_r)
    print('Essential matrix:', E)


def decode_image(payload):
    img_payload = json.loads(payload.decode('utf-8'))
    img_data = base64.b64decode(img_payload['image'])
    np_arr = np.frombuffer(img_data, np.uint8)
    timestamp = img_payload['timestamp']
    #print(f"[INFO] Decoding image with timestamp: {timestamp}")
    img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
    return img, timestamp

def perform_triangulation(matches):
    dist_coeffs = np.zeros(5)
    points_left = np.array([match['left'] for match in matches])
    points_right = np.array([match['right'] for match in matches])
    # Projection matrices
    ## Left camera
    P1 = l_intrinsics @ np.hstack((np.eye(3), np.zeros((3, 1))))  # [I | 0] in homogeneous coordinates
    ## Right camera
    P2 = r_intrinsics @ np.hstack((R_l_r, T_l_r.reshape(3, 1)))  # [R | t] in homogeneous coordinates
    points_l_h = points_left.T  # shape (2, N)
    points_r_h = points_right.T  # shape (2, N)

    points_hom = cv2.triangulatePoints(P1, P2, points_l_h, points_r_h)
    points_3d = (points_hom[:3, :] / points_hom[3, :]).T
    pnp = cv2.solvePnP(points_3d, points_left, l_intrinsics, l_distcoeffs)
    # Write the PnP result to poses.txt
    if pnp[0]:  # Check if solvePnP was successful
        rvec, tvec = pnp[1], pnp[2]
        with open("poses.txt", "a") as f:
            f.write(f"Rotation Vector: {rvec.flatten().tolist()}\n")
            f.write(f"Translation Vector: {tvec.flatten().tolist()}\n")

    




def on_points():
    if image_buffer['left'] is not None and image_buffer['right'] is not None and matches_buffer is not None:
        perform_triangulation(matches_buffer)

def on_message(client, userdata, msg):
    global matches_buffer

    if msg.topic == TOPIC_LEFT_IMG:
        image_buffer['left'], _ = decode_image(msg.payload)
    elif msg.topic == TOPIC_RIGHT_IMG:
        image_buffer['right'], _ = decode_image(msg.payload)
    elif msg.topic == TOPIC_MATCHES:
        matches_buffer = json.loads(msg.payload.decode('utf-8'))

    on_points()

# MQTT setup
client = mqtt.Client()
client.on_message = on_message
client.connect("localhost", 1883, 60)
client.subscribe(TOPIC_LEFT_IMG)
client.subscribe(TOPIC_RIGHT_IMG)
client.subscribe(TOPIC_MATCHES)

print("[INFO] Listening and visualizing stereo matches...")
client.loop_forever()
