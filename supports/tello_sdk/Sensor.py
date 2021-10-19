#!/usr/bin/python3

"""
@brief      This is Tello sensor API to retrieve sensors informations
@author     Tushar Saini (saini.tusahr007@gmail.com)
"""

import logging
import socket
import threading
import time


class Sensor(object):
    """docstring for Sensor"""

    # Constant.
    SENSOR_LOG_SAMPLING_SEC = 1

    def __init__(self, log=False):
        # Enable/disable logging.
        self.log = logging.getLogger(__name__)
        self.log.disabled = not log
        self.last_log_time = time.time()

        # Create a UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 8890))

        # Receiving command thread.
        self.recv_thread = threading.Thread(target=self._sensor_receive_thread)
        self.recv_thread.daemon = True
        self.recv_thread.start()

        # Instance variable.
        self.response = None
        self.sensor_data = {
            'orient': [0, 0, 0],
            'speed': [0, 0, 0],
            'temp': [0, 0],
            'tof': 0,
            'height': 0,
            'battery': 0,
            'baro': 0.0,
            'time': 0,
            'acceleration': [0.0, 0.0, 0.0],
            'mid': 0,
            'mission_pad_coord': [0, 0, 0]
        }

    def __del__(self):
        self.sock.close()

    def _sensor_receive_thread(self):
        while True:
            try:
                self.response, ip = self.sock.recvfrom(1024)
            except Exception as err:
                self.log.error(err)
                break

    def _sensor_response(self, command):
        """
        Returns a response within timeout.
        """
        try:
            # Process data.
            data = self.response.decode('utf-8').rstrip('\r\n').rstrip(';')
            data = data.replace(';', ':').split(':')

            start_index = 0
            if command == 'orient':
                start_index = data.index('pitch')
                self.sensor_data.update({
                    command: [
                        int(data[start_index+1]),
                        int(data[start_index+3]),
                        int(data[start_index+5])
                    ]
                })
            elif command == 'speed':
                start_index = data.index('vgx')
                self.sensor_data.update({
                    command: [
                        int(data[start_index+1]),
                        int(data[start_index+3]),
                        int(data[start_index+5])
                    ]
                })
            elif command == 'temp':
                start_index = data.index('templ')
                self.sensor_data.update({
                    command: [
                        int(data[start_index+1]),
                        int(data[start_index+3])
                    ]
                })
            elif command == 'tof':
                start_index = data.index('tof')
                self.sensor_data.update({
                    command: int(data[start_index+1])
                })
            elif command == 'height':
                start_index = data.index('h')
                self.sensor_data.update({
                    command: int(data[start_index+1])
                })
            elif command == 'battery':
                start_index = data.index('bat')
                self.sensor_data.update({
                    command: int(data[start_index+1])
                })
            elif command == 'baro':
                start_index = data.index('baro')
                self.sensor_data.update({
                    command: float(data[start_index+1])
                })
            elif command == 'time':
                start_index = data.index('time')
                self.sensor_data.update({
                    command: int(data[start_index+1])
                })
            elif command == 'acceleration':
                start_index = data.index('agx')
                self.sensor_data.update({
                    command: [
                        float(data[start_index+1]),
                        float(data[start_index+3]),
                        float(data[start_index+5])
                    ]
                })

            # Tello EDU specific start here
            elif command == 'mission_pad_id':
                start_index = data.index('mid')
                self.sensor_data.update({
                    command: int(data[start_index+1])
                })
            elif command == 'mission_pad_coord':
                start_index = data.index('x')
                self.sensor_data.update({
                    command: [
                        int(data[start_index+1]),
                        int(data[start_index+3]),
                        int(data[start_index+5])
                    ]
                })
            else:
                self.log.error('Invalid sensor data name!!')

            # Sampling sensor info.
            if (time.time() - self.last_log_time) > self.SENSOR_LOG_SAMPLING_SEC:
                self.log.info('sensor: {}'.format(self.sensor_data))
                self.last_log_time = time.time()
        except UnicodeDecodeError as err:
            self.log.error('sensor:{} res:{}'.format(command, err))
            return False
        except AttributeError as err:
            self.log.error('sensor:{} res:{}'.format(command, err))
            return False
        except Exception as err:
            self.log.error('sensor:{} res:{}'.format(command, err))
            return False

        # Return true when passed all process.
        return True

    def get_orient(self):
        """
        Gets the orientation in format 
        pitch, roll, yaw in degrees.
        """
        status = 'orient'
        if self._sensor_response(status):
            return self.sensor_data[status]
        else:
            return False, False, False

    def get_speed(self):
        """
        Gets the speed in format x,y,z in cm/s.
        """
        status = 'speed'
        if self._sensor_response(status):
            return self.sensor_data[status]
        else:
            return False, False, False

    def get_temp(self):
        """
        Gets the core temperature in degrees celcius.
        """
        status = 'temp'
        if self._sensor_response(status):
            return self.sensor_data[status]
        else:
            return False, False

    def get_tof(self):
        """
        Gets the tof sensor distance in cm.
        """
        status = 'tof'
        return self.sensor_data[status] if self._sensor_response(status) else False

    def get_height(self):
        """
        Gets the height in cm.
        """
        status = 'height'
        return self.sensor_data[status] if self._sensor_response(status) else False

    def get_battery(self):
        """
        Gets the battery life in percentage.
        """
        status = 'battery'
        return self.sensor_data[status] if self._sensor_response(status) else False

    def get_barometer(self):
        """
        Gets the barometric pressure in ??
        """
        status = 'baro'
        return self.sensor_data[status] if self._sensor_response(status) else False

    def get_flight_time(self):
        """
        Gets the flight time in second.
        """
        status = 'time'
        return self.sensor_data[status] if self._sensor_response(status) else False

    def get_acceleration(self):
        """
        Gets the acceleration in format ax, ay, az in ??.
        """
        status = 'acceleration'
        if self._sensor_response(status):
            return self.sensor_data[status]
        else:
            return False, False, False
    
    def get_mission_pad_id(self):
        """
        Gets mission pad id

        If the mission pad detection function is not enabled, -2 is returned.
        If the detection function is enabled but no mission pad is detected, -1 is returned.
        """
        status = 'mission_pad_id'
        if self._sensor_response(status):
            return self.sensor_data[status]
        else:
            return False

    def get_mission_pad_coord(self):
        """
        Gets the mission pad coordinates in centimeters for x, y and z

        If the mission pad detection function is not enabled, -200 is returned.
        If the detection function is enabled but no mission pad is detected, -100 is returned.
        """
        status = 'mission_pad_coord'
        if self._sensor_response(status):
            return self.sensor_data[status]
        else:
            return False, False, False
