"""
@brief      This is main program that control state machine
"""

import logging

from State import StateEnum, State

# Configure log handler
logging.basicConfig(
    filename='log/flight.log',
    filemode='w',
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    level=logging.DEBUG,
    datefmt='%d/%m/%Y %H:%M:%S' )
log = logging.getLogger(__name__)

# Set initial state
state = State(log=True)
event = StateEnum.START

print("\nMain program started..")
log.info("Main program started..")

while True:
    if event == StateEnum.START:
        event = state.start()
    elif event == StateEnum.IDLE:
        event = state.idle()
    elif event == StateEnum.IMG_PROCESS:
        event = state.image_process()
    elif event == StateEnum.POSE_RECOG:
        event = state.pose_recognition()
    elif event == StateEnum.NAVI:
        event = state.navigation()
    elif event == StateEnum.EXIT:
        event = state.exit()
    else:
        break

print("\nMain program terminated..")
