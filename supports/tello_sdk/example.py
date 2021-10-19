"""
@brief      This example to takoff Tello, stream out video from its camera,
            print out sensor and state information and then land it back.

@details    All SDK contains their own thread which make it possible to 
            simulataneously send command, read sensor and stream video camera
            from/to Tello.

@author     Tushar Saini (saini.tusahr007@gmail.com)
"""

import time

import cv2

# Import SDK library
from Command import Command
from Sensor import Sensor
from Stream import Stream

# Create SDK object
command = Command(log=False)
sensor = Sensor(log=False)

# Enter SDK mode and takeoff
command.command()

# Enable and start camera stream
command.enable_stream()
stream = Stream(address='udp://@0.0.0.0:11111').start()

# Set flight timeout
timeout_msec = 5000
initial_time = time.time()

# Flag to check if Tello has taken off
has_takeoff = False

# Image name
image_name = 'frame_captured.jpg'

while(True):
    try:
        # Takeoff once
        if not has_takeoff:
            has_takeoff = True
            command.takeoff()

        # Check if flight time not reach timeout
        flight_time = time.time() - initial_time
        if flight_time < timeout_msec:
            # Get sensor reading
            battery_level = sensor.get_battery()
            temperature = sensor.get_temp()
            height = sensor.get_tof()

            # Print sensor info. More API available in Sensor.py
            print("battery(%): {}".format(battery_level))
            print("temp(deg): {}".format(temperature))
            print("height(cm): {}".format(height))

            # Save frame to image file that later can be used
            # for object recognition and pose estimation
            cv2.imwrite(image_name, stream.frame)

            # Display the image
            cv2.imshow("example", stream.frame)
            cv2.waitKey(1)
        else:
            # Land Tello when reach flight timeout
            command.land()
            break
    except KeyboardInterrupt:
        # Land Tello when hit Ctrl+C in emergency
        command.land()
        break