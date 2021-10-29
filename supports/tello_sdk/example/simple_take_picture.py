import time
import cv2

# Import SDK library
from supports.tello_sdk.Command import Command
from supports.tello_sdk.Stream import Stream

# Create SDK object
command = Command(log=False)

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

while True:
    try:
        # Takeoff once
        if not has_takeoff:
            has_takeoff = True
            command.takeoff()

        # Check if flight time not reach timeout
        flight_time = time.time() - initial_time
        if flight_time < timeout_msec:

            # Save frame to image file that later can be used
            # for object recognition and pose estimation
            key = cv2.waitKey(1) & 0xff
            if key == ord('p'):
                print("picture")
                cv2.imwrite(image_name, stream.frame)
                continue

            # Display the image
            cv2.imshow("example", stream.frame)
            cv2.waitKey(1)
        else:
            # Land Tello when reach flight timeout
            command.land()
            command.disable_stream()
            break
    except KeyboardInterrupt:
        # Land Tello when hit Ctrl+C in emergency
        command.land()
        command.disable_stream()
        break
