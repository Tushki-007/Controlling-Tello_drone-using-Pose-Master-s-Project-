#!/usr/bin/python3

"""
@brief      This is Tello video API to retrieve video frame
@author     Tushar Saini (saini.tusahr007@gmail.com)
"""

import logging
import threading

import cv2
import numpy as np


class Stream(object):
    """docstring for Stream"""

    def __init__(self, address):
        # Enable logging.
        self.log = logging.getLogger(__name__)
        self.stream_vc = cv2.VideoCapture(address)
        (self.grabbed, self.frame) = self.stream_vc.read()
        self.stopped = False

        width, height = self.get_frame_size()
        self.blackframe = np.zeros((int(width), int(height), 1), np.uint8)

    def __del__(self):
        self.stream_vc.release()

    def start(self):
        thread = threading.Thread(target=self.get, args=())
        thread.daemon = True
        thread.start()
        return self

    def get(self):
        while not self.stopped:
            try:
                (self.grabbed, self.frame) = self.stream_vc.read()
            except Exception as err:
                (self.grabbed, self.frame) = (-1, self.blackframe)
                self.log.error(err)

    def stop(self):
        self.stopped = True

    def get_frame_size(self):
        width = int(self.stream_vc.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.stream_vc.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return width, height
