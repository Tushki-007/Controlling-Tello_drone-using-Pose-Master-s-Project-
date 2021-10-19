"""
@brief      Test file for Tello Sensor SDK
@author     Tushar Saini (saini.tusahr007@gmail.com)
"""

sensor_data = {
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

test_modes = ["edu", "normal"]

test_commands = {
    "edu": ['orient', 'speed', 'temp', 'tof', 'height', 'battery', 'baro', 'time', 'acceleration', 'mid', 'mission_pad_coord'],
    "normal": ['orient', 'speed', 'temp', 'tof', 'height', 'battery', 'baro', 'time', 'acceleration']
}

test_datas = {
    "edu": ['mid', '-1', 'x', '-100', 'y', '-100', 'z', '-100', 'mpry', '0,0,0', 'pitch', '1', 'roll', '9', 'yaw', '-73', 'vgx', '0', 'vgy', '0', 'vgz', '0', 'templ', '91', 'temph', '92', 'tof', '6553', 'h', '0', 'bat', '48', 'baro', '-58.65', 'time', '0', 'agx', '-80.00', 'agy', '-257.00', 'agz', '-979.00'],
    "normal": ['pitch', '1', 'roll', '9', 'yaw', '-73', 'vgx', '0', 'vgy', '0', 'vgz', '0', 'templ', '91', 'temph', '92', 'tof', '6553', 'h', '0', 'bat', '48', 'baro', '-58.65', 'time', '0', 'agx', '-80.00', 'agy', '-257.00', 'agz', '-979.00']
}

test_pass = True
print("\nTEST STARTED\n==================\n")

for mode in test_modes:
    data = test_datas[mode]
    commands = test_commands[mode]
    for command in commands:
        try:
            start_index = 0
            if command == 'orient':
                start_index = data.index('pitch')
                sensor_data.update({
                    command: [
                        int(data[start_index+1]),
                        int(data[start_index+3]),
                        int(data[start_index+5])
                    ]
                })
            elif command == 'speed':
                start_index = data.index('vgx')
                sensor_data.update({
                    command: [
                        int(data[start_index+1]),
                        int(data[start_index+3]),
                        int(data[start_index+5])
                    ]
                })
            elif command == 'temp':
                start_index = data.index('templ')
                sensor_data.update({
                    command: [
                        int(data[start_index+1]),
                        int(data[start_index+3])
                    ]
                })
            elif command == 'tof':
                start_index = data.index('tof')
                sensor_data.update({
                    command: int(data[start_index+1])
                })
            elif command == 'height':
                start_index = data.index('h')
                sensor_data.update({
                    command: int(data[start_index+1])
                })
            elif command == 'battery':
                start_index = data.index('bat')
                sensor_data.update({
                    command: int(data[start_index+1])
                })
            elif command == 'baro':
                start_index = data.index('baro')
                sensor_data.update({
                    command: float(data[start_index+1])
                })
            elif command == 'time':
                start_index = data.index('time')
                sensor_data.update({
                    command: int(data[start_index+1])
                })
            elif command == 'acceleration':
                start_index = data.index('agx')
                sensor_data.update({
                    command: [
                        float(data[start_index+1]),
                        float(data[start_index+3]),
                        float(data[start_index+5])
                    ]
                })

            # Tello EDU specific start here
            elif command == 'mid':
                start_index = data.index('mid')
                sensor_data.update({
                    command: int(data[start_index+1])
                })
            elif command == 'mission_pad_coord':
                start_index = data.index('x')
                sensor_data.update({
                    command: [
                        int(data[start_index+1]),
                        int(data[start_index+3]),
                        int(data[start_index+5])
                    ]
                })
        except Exception as err:
            test_pass = False
            print("TEST FAILED:{}".format(err))
            break

    print("{}: {}\n".format(mode, sensor_data))

if test_pass:
    print("==================\nTEST PASSED")