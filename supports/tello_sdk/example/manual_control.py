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

        # Display the image
        cv2.imshow("example", stream.frame)
        cv2.waitKey(1)

        key = cv2.waitKey(1) & 0xff
        if key == 27:  # ESC
            command.land()
            command.disable_stream()
            break
        elif key == ord('w'):
            command.move_rc(0, 30, 0, 0)
        elif key == ord('s'):
            command.move_rc(0, -30, 0, 0)
        elif key == ord('a'):
            command.move_rc(30, 0, 0, 0)
        elif key == ord('d'):
            command.move_rc(-30, 0, 0, 0)
        elif key == ord('q'):
            command.move_rc(0, 0, 0, 30)
        elif key == ord('e'):
            command.move_rc(0, 0, 0, -30)
        elif key == ord('r'):
            command.move_rc(0, 0, 30, 0)
        elif key == ord('f'):
            command.move_rc(0, 0, -30, 0)
        elif key == ord(' '):
            command.move_rc(0, 0, 0, 0)

    except KeyboardInterrupt:
        # Land Tello when hit Ctrl+C in emergency
        command.land()
        command.disable_stream()
        break


