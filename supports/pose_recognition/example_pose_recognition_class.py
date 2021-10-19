import math
import time

import cv2
import numpy as np

import pose_recognition as pr

# camera Input
cap = cv2.VideoCapture("../../assets/test.mp4")
# object for the class pose_recognition.py
detector = pr.PoseDetector()
# Initializing few previous variable to zero
p_time = 0  # previous_time for FPS
c0, c1 = 0, 0  # x, y coordinates of Centroid
xp, yp, distance, angle = 0, 0, 0, 0  # Variables

# Creating a canvas to draw the line on the image
img_canvas = np.zeros((480, 640, 3), np.uint8)

while True:

    success, img = cap.read()
    # img = cv2.flip(img, 1)
    landmark_list, world_landmark_list = detector.find_body(img, draw=True)

    # Calculating FPS
    c_time = time.time()
    fps = 1 // (c_time - p_time)
    p_time = c_time

    try:
        if landmark_list is not None:
            angle = detector.find_angle(12, 14, 16)  # Angle between the right_elbow
            distance = detector.dist2p(11, 12)  # Distance between two shoulder
            cent = detector.centroid(img, draw=True)

            # To clear the printed line from the image
            right_wrist_x, right_wrist_y, _ = landmark_list[16]  # Right wrist
            dist_bw_centroid_and_right_wrist = int(math.hypot(right_wrist_x - c0, right_wrist_y - c1))

            cv2.putText(img, f'FPS : {str(int(fps))}', (10, 30),
                        cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 2)

            cv2.putText(img, f'Distance b/w Shoulders :{str(int(distance))}', (10, 90),
                        cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 2)

            cv2.putText(img, f'angle b/w R_elbow :{str(int(angle))}', (10, 120),
                        cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 2)
        else:

            cv2.putText(img, " Landmarks Not In Frame ", (100, 250),
                        cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)

    except Exception as Error:
        pass
    cv2.imshow("Image", img)
    if cv2.waitKey(1) & 0xFF == 27:
        break
cap.release()

cv2.destroyAllWindows()
