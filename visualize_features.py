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

def decode_image(payload):
    img_payload = json.loads(payload.decode('utf-8'))
    img_data = base64.b64decode(img_payload['image'])
    np_arr = np.frombuffer(img_data, np.uint8)
    timestamp = img_payload['timestamp']
    print(f"[INFO] Decoding image with timestamp: {timestamp}")
    img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
    return img, timestamp

def draw_matches(img_left, img_right, matches):
    height = max(img_left.shape[0], img_right.shape[0])
    width = img_left.shape[1] + img_right.shape[1]

    combined = np.zeros((height, width), dtype=np.uint8)
    combined[:img_left.shape[0], :img_left.shape[1]] = img_left
    combined[:img_right.shape[0], img_left.shape[1]:] = img_right

    combined = cv2.cvtColor(combined, cv2.COLOR_GRAY2BGR)

    for match in matches:
        pt_left = tuple(map(int, match['left']))
        pt_right = tuple(map(int, match['right']))
        pt_right_shifted = (int(pt_right[0] + img_left.shape[1]), int(pt_right[1]))

        color = (0, 255, 0)
        cv2.circle(combined, pt_left, 3, color, -1)
        cv2.circle(combined, pt_right_shifted, 3, color, -1)
        cv2.line(combined, pt_left, pt_right_shifted, color, 1)

    return combined

def try_draw():
    if image_buffer['left'] is not None and image_buffer['right'] is not None and matches_buffer is not None:
        vis = draw_matches(image_buffer['left'], image_buffer['right'], matches_buffer)
        cv2.imshow("Stereo Feature Matches", vis)
        cv2.waitKey(1)

def on_message(client, userdata, msg):
    global matches_buffer

    if msg.topic == TOPIC_LEFT_IMG:
        image_buffer['left'], _ = decode_image(msg.payload)
    elif msg.topic == TOPIC_RIGHT_IMG:
        image_buffer['right'], _ = decode_image(msg.payload)
    elif msg.topic == TOPIC_MATCHES:
        matches_buffer = json.loads(msg.payload.decode('utf-8'))

    try_draw()

# MQTT setup
client = mqtt.Client()
client.on_message = on_message
client.connect("localhost", 1883, 60)
client.subscribe(TOPIC_LEFT_IMG)
client.subscribe(TOPIC_RIGHT_IMG)
client.subscribe(TOPIC_MATCHES)

print("[INFO] Listening and visualizing stereo matches...")
client.loop_forever()
