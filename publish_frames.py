import depthai as dai
import cv2
import paho.mqtt.client as mqtt
import numpy as np
import time

# MQTT Setup
MQTT_BROKER = "localhost"  # or the IP of the MQTT broker
MQTT_PORT = 1883
MQTT_TOPIC_LEFT = "oakd/stereo/left"
MQTT_TOPIC_RIGHT = "oakd/stereo/right"
FREQUENCY = 20
FRAME_PERIOD = 1.0 / FREQUENCY  # 0.05 seconds


client = mqtt.Client()
client.connect(MQTT_BROKER, MQTT_PORT, 60)

# DepthAI Pipeline
pipeline = dai.Pipeline()

cam_left = pipeline.createMonoCamera()
cam_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
cam_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

cam_right = pipeline.createMonoCamera()
cam_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
cam_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

# Stereo depth to enable rectification
stereo = pipeline.createStereoDepth()
stereo.setOutputRectified(True)
stereo.setRectifyEdgeFillColor(0)  # black, or use 255 for white
stereo.initialConfig.setConfidenceThreshold(255)

cam_left.out.link(stereo.left)
cam_right.out.link(stereo.right)

xout_left = pipeline.createXLinkOut()
xout_left.setStreamName("rectified_left")
stereo.rectifiedLeft.link(xout_left.input)

xout_right = pipeline.createXLinkOut()
xout_right.setStreamName("rectified_right")
stereo.rectifiedRight.link(xout_right.input)

# Start device
with dai.Device(pipeline) as device:
    q_left = device.getOutputQueue("rectified_left", maxSize=4, blocking=False)
    q_right = device.getOutputQueue("rectified_right", maxSize=4, blocking=False)

    print(f"[INFO] Publishing rectified stereo at {FREQUENCY} Hz over MQTT...")
    last_time = time.time()

    while True:
        current_time = time.time()
        elapsed = current_time - last_time

        if elapsed >= FRAME_PERIOD:
            in_left = q_left.tryGet()
            in_right = q_right.tryGet()

            if in_left is not None and in_right is not None:
                img_left = in_left.getCvFrame()
                img_right = in_right.getCvFrame()

                _, buffer_left = cv2.imencode('.jpg', img_left)
                _, buffer_right = cv2.imencode('.jpg', img_right)

                client.publish(MQTT_TOPIC_LEFT, buffer_left.tobytes())
                client.publish(MQTT_TOPIC_RIGHT, buffer_right.tobytes())

                last_time = current_time
        else:
            time.sleep(FRAME_PERIOD - elapsed)
