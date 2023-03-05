"""
This script activates on bootup and monitors the arduino GPIO pin for training == 1.
Then, it will record the videos of the infant using the webcam and save them to the xxx folder.
Once training is done, we perform pose detection by running the pose detection script on all the videos.
The pose detection script will output the frame no, coordinates, confidence, and joint angles.
This data will be saved to a csv file, and published to the Arduino IOT cloud. 
"""

import os
import time
import csv
import numpy as np
import pandas as pd
import RPi.GPIO as GPIO
import jetson_utils
import jetson_inference 

#set up GPIO pin
GPIO.setmode(GPIO.BOARD)
GPIO.setup(18, GPIO.IN)

VIDEO_PATH = './videos'
CSV_PATH = './pose_data_csv'

MODEL = "densenet121-body"

CONF_THRESHOLD = 0.5

# kepypoints includeo shoulder, elbow, wrist, hip, knee, ankle, neck
KEYPOINTS = [
    'neck', 'r_shoulder', 'r_elbow', 'r_wrist', 'l_shoulder', 'l_elbow', 'l_wrist', 'r_hip', 'r_knee', 'r_ankle', 'l_hip', 'l_knee', 'l_ankle'
]

# Define the frame rate and resolution for video recording
FRAME_RATE = 30
# RESOLUTION = (640, 480)

# Initialize the camera and the video recorder
camera = jetson_utils.gstCamera(640, 480, "v4l2src device=/dev/video0 ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)UYVY ! videoconvert ! video/x-raw, format=(string)BGRx ! videoconvert ! appsink max-buffers=1 drop=True" % RESOLUTION)
video_writer = jetson_utils.VideoWriter("%s/video_%d.mp4" % (VIDEO_PATH, time.time()), cv2.VideoWriter_fourcc(*"mp4v"), FRAME_RATE, RESOLUTION)


# Initialize the pose detection network
pose_net = jetson_inference.poseEstimation(MODEL, RESOLUTION[0], RESOLUTION[1])

def pose_detection()

def get_angle(p1, p2, p3):
    #p1 p2 p3 are the np array coordinates of the keypoints
    a = np.linalg.norm(p2 - p3)
    b = np.linalg.norm(p1 - p3)
    c = np.linalg.norm(p1 - p2)
    angle = np.arccos((b**2 + c**2 - a**2) / (2 * b * c))
    return np.degrees(angle)


def main():
    #monitor gpio pin for training == 1
    while GPIO.input(18) == GPIO.HIGH:
        print("training started")
        #start reScording using utils
        image, width, height = camera.Capture(format = "rgb8")

        # save the image to the video file
        video_writer.Write(image)






        frame = 0
        #start recording using utils



if __name__ == '__main__':

    main()