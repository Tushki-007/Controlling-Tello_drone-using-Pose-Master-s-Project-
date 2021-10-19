#!/usr/bin/python3

"""
@brief      This is Tello command API to send/receive message
@author     Tushar Saini (saini.tusahr007@gmail.com)
"""

import logging
import socket
import threading
import time

# Constant.
CMD_RES_TIMEOUT_SEC = 7
CMD_TIME_BETWEEN_SEC = 1
CMD_RETRY_NUM = 3
TRAVEL_DISTANCE_MIN_CM = 20
TRAVEL_DISTANCE_MAX_CM = 500
TRAVEL_ROTATION_MIN_ANGLE = 1
TRAVEL_ROTATION_MAX_ANGLE = 360
TRAVEL_SPEED_MIN_CM_S = 10
TRAVEL_SPEED_MAX_CM_S = 100
TRAVEL_RC_MIN_PERCENT = -100
TRAVEL_RC_MAX_PERCENT = 100
TRAVEL_GO_DISTANCE_MIN_CM = -500
TRAVEL_GO_DISTANCE_MAX_CM = 500
CMD_SUCCESS_RES = 'ok'
CMD_FAILED_RES = 'error'


class Command(object):
    """docstring for Command"""

    def __init__(self, log=False):
        # Enable/disable logging.
        self.log = logging.getLogger(__name__)
        self.log.disabled = not log

        # Create a UDP socket.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 9000))
        self.tello_address = ('192.168.10.1', 8889)

        # Receiving command thread.
        self.recv_thread = threading.Thread(target=self._command_receive_thread)
        self.recv_thread.daemon = True
        self.recv_thread.start()

        # Instance variable.
        self.response = None
        self.last_command_time = 0

    def __del__(self):
        self.sock.close()

    def _command_receive_thread(self):
        while True:
            try:
                self.response, _ = self.sock.recvfrom(1024)
            except Exception as err:
                self.log.error(err)
                break

    def _cmd_wrapper(self, command, retry=CMD_RETRY_NUM, timeout=CMD_RES_TIMEOUT_SEC):
        for retry in range(retry):
            res = self._cmd_response(command, timeout)
            if res != CMD_FAILED_RES:
                self.log.info('cmd:{} res:{}'.format(command, res))
                return res
            else:
                self.log.error('cmd:{} res:error'.format(command))
        self.log.error('cmd:{} res:retry exceeded'.format(command))
        return False

    def _cmd_response(self, command, timeout=CMD_RES_TIMEOUT_SEC):
        """
        Returns a response within timeout.
        """
        # Wait between command.
        diff = time.time() - self.last_command_time
        if diff < CMD_TIME_BETWEEN_SEC:
            time.sleep(diff)

        # Send command.
        self.response = None
        self.sock.sendto(command.encode('utf-8'), self.tello_address)

        # Ready for next command.
        start = time.time()
        while self.response is None:
            if (time.time() - start) > timeout:
                self.log.error('cmd:{} res:timeout'.format(command))
                return False

        try:
            response = self.response.decode().rstrip('\r\n')
        except UnicodeDecodeError as err:
            self.log.error('cmd:{} res:{}'.format(command, err))
            return False

        # Capture last command time.
        self.last_command_time = time.time()
        return response

    def _clamp(self, n, minn, maxn):
        # Clamp value to be within range min, max.
        return min(max(n, minn), maxn)

    def command(self):
        # Enter SDK mode.
        return self._cmd_wrapper('command')

    def takeoff(self):
        # Auto take-off
        return self._cmd_wrapper('takeoff', timeout=20)

    def land(self):
        # Auto landing.
        return self._cmd_wrapper('land', timeout=10)

    def enable_stream(self):
        # Enable video stream.
        return self._cmd_wrapper('streamon')

    def disable_stream(self):
        # Disable video stream.
        return self._cmd_wrapper('streamoff')

    def stop_motors(self):
        # Emergency & stop all motors.
        return self._cmd_wrapper('emergency')

    def move(self, direction, distance):
        """
        Support for up, down, left, right, forward, back, 
        rotate cw and ccw command.
        """
        if direction in ['up', 'down', 'left', 'right', 'forward', 'back']:
            distance = self._clamp(distance, TRAVEL_DISTANCE_MIN_CM, TRAVEL_DISTANCE_MAX_CM)
        elif direction in ['cw', 'ccw']:
            distance = self._clamp(distance, TRAVEL_ROTATION_MIN_ANGLE, TRAVEL_ROTATION_MAX_ANGLE)
        else:
            return False
        # Send command.
        command = ' '.join([direction, str(distance)])
        return self._cmd_wrapper(command)

    def move_rc(self, x, y, z, yaw):
        """
        Input parameters:
        x=left_right, y=forw_back, z=up_down, yaw=cw_ccw
        """
        # Run remote controller (RC) command.
        x = self._clamp(x, TRAVEL_RC_MIN_PERCENT, TRAVEL_RC_MAX_PERCENT)
        y = self._clamp(y, TRAVEL_RC_MIN_PERCENT, TRAVEL_RC_MAX_PERCENT)
        z = self._clamp(z, TRAVEL_RC_MIN_PERCENT, TRAVEL_RC_MAX_PERCENT)
        yaw = self._clamp(yaw, TRAVEL_RC_MIN_PERCENT, TRAVEL_RC_MAX_PERCENT)
        # Send command.
        command = ' '.join(['rc', str(x), str(y), str(z), str(yaw)])
        self.sock.sendto(command.encode('utf-8'), self.tello_address)
        return True

    def move_go(self, x, y, z, speed):
        # Run go command.
        x = self._clamp(x, TRAVEL_GO_DISTANCE_MIN_CM, TRAVEL_GO_DISTANCE_MAX_CM)
        y = self._clamp(y, TRAVEL_GO_DISTANCE_MIN_CM, TRAVEL_GO_DISTANCE_MAX_CM)
        z = self._clamp(z, TRAVEL_GO_DISTANCE_MIN_CM, TRAVEL_GO_DISTANCE_MAX_CM)
        speed = self._clamp(speed, TRAVEL_SPEED_MIN_CM_S, TRAVEL_SPEED_MAX_CM_S)
        # Send command.
        command = ' '.join(['go', str(x), str(y), str(z), str(speed)])
        self.sock.sendto(command.encode('utf-8'), self.tello_address)
        return True

    def move_curve(self, x, y, z, speed):
        # Run curve command.
        # x, y, z input is array, contain x1 and x2 etc.
        x1 = self._clamp(x[0], TRAVEL_GO_DISTANCE_MIN_CM, TRAVEL_GO_DISTANCE_MAX_CM)
        y1 = self._clamp(y[0], TRAVEL_GO_DISTANCE_MIN_CM, TRAVEL_GO_DISTANCE_MAX_CM)
        z1 = self._clamp(z[0], TRAVEL_GO_DISTANCE_MIN_CM, TRAVEL_GO_DISTANCE_MAX_CM)
        x2 = self._clamp(x[1], TRAVEL_GO_DISTANCE_MIN_CM, TRAVEL_GO_DISTANCE_MAX_CM)
        y2 = self._clamp(y[1], TRAVEL_GO_DISTANCE_MIN_CM, TRAVEL_GO_DISTANCE_MAX_CM)
        z2 = self._clamp(z[1], TRAVEL_GO_DISTANCE_MIN_CM, TRAVEL_GO_DISTANCE_MAX_CM)
        speed = self._clamp(speed, TRAVEL_SPEED_MIN_CM_S, TRAVEL_SPEED_MAX_CM_S)
        # Send command.
        command = ' '.join(['curve', str(x1), str(y1), str(z1), str(x2), str(y2), str(z2), str(speed)])
        self.sock.sendto(command.encode('utf-8'), self.tello_address)
        return True

    def roll(self, direction):
        # Support for flip command.
        command = ' '.join(['flip', direction[0]])  # take first char
        return self._cmd_wrapper(command)

    def stop(self):
        # Stop and hovers mode.
        return self._cmd_wrapper('stop')

    def set_speed(self, speed):
        # Speed in unit cm/s
        speed = self._clamp(speed, TRAVEL_SPEED_MIN_CM_S, TRAVEL_SPEED_MAX_CM_S)
        command = ' '.join(['speed', str(speed)])
        return self._cmd_wrapper(command)

    def set_wifi(self, ssid, password):
        # Set Tello WiFi credential.
        command = ' '.join(['wifi', str(ssid), str(password)])
        return self._cmd_wrapper(command)

    def set_ap(self, ssid, password):
        # Set Tello AP credential.
        command = ' '.join(['ap', str(ssid), str(password)])
        return self._cmd_wrapper(command)

    def get_speed(self):
        # Get speed in cm/s.
        return self._cmd_wrapper('speed?')

    def get_wifi_snr(self):
        # Get WiFi Signal-to-Noise ratio.
        return self._cmd_wrapper('wifi?')
