import cv2
import numpy as np
import paho.mqtt.client as mqtt
import json
import base64

TOPIC_LEFT_IMG = "oakd/stereo/left"
TOPIC_RIGHT_IMG = "oakd/stereo/right"
TOPIC_MATCHES = "oakd/stereo/features/matches"

image_buffer = {'left': None, 'right': None}

orb = cv2.ORB_create(nfeatures=1000)
matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# Filtering thresholds
EPIPOLAR_THRESHOLD = 2  # pixels
MIN_DISPARITY = 2       # pixels

def decode_image(payload):
    np_arr = np.frombuffer(payload, np.uint8)
    return cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

def serialize_matches(kp_left, kp_right, matches):
    # Format: list of dicts with left/right point pairs
    matched_points = []
    for m in matches:
        pt_left = kp_left[m.queryIdx].pt
        pt_right = kp_right[m.trainIdx].pt
        matched_points.append({
            'left': [pt_left[0], pt_left[1]],
            'right': [pt_right[0], pt_right[1]]
        })
    return json.dumps(matched_points)

def match_and_publish(img_left, img_right):
    kp_left, des_left = orb.detectAndCompute(img_left, None)
    kp_right, des_right = orb.detectAndCompute(img_right, None)

    if des_left is None or des_right is None:
        print("[WARN] No descriptors found.")
        return

    matches = matcher.match(des_left, des_right)
    
    # Filter matches by epipolar constraint and disparity
    filtered_matches = []
    for m in matches:
        y_diff = abs(kp_left[m.queryIdx].pt[1] - kp_right[m.trainIdx].pt[1])
        x_left = kp_left[m.queryIdx].pt[0]
        x_right = kp_right[m.trainIdx].pt[0]
        disparity = x_left - x_right

        if y_diff < EPIPOLAR_THRESHOLD and disparity > MIN_DISPARITY:
            filtered_matches.append(m)

    # Serialize and publish
    serialized = serialize_matches(kp_left, kp_right, filtered_matches)
    client.publish(TOPIC_MATCHES, serialized)

def on_message(client, userdata, msg):
    if msg.topic == TOPIC_LEFT_IMG:
        image_buffer['left'] = decode_image(msg.payload)
    elif msg.topic == TOPIC_RIGHT_IMG:
        image_buffer['right'] = decode_image(msg.payload)

    if image_buffer['left'] is not None and image_buffer['right'] is not None:
        match_and_publish(image_buffer['left'], image_buffer['right'])
        image_buffer['left'] = None
        image_buffer['right'] = None

# MQTT Setup
client = mqtt.Client()
client.on_message = on_message
client.connect("localhost", 1883, 60)
client.subscribe(TOPIC_LEFT_IMG)
client.subscribe(TOPIC_RIGHT_IMG)

print("[INFO] Publishing filtered stereo matches to MQTT...")
client.loop_forever()
