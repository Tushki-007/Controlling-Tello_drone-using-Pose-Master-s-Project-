"""
@brief      This is control system program
"""


class Control(object):
    def __init__(self, min_limit, max_limit):
        self.setpoint = 0
        self.last_error = 0
        self.cum_error = 0

        # Default gain
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0

        # Output limit
        self.min_limit = min_limit
        self.max_limit = max_limit

    def get_error_offset(self, pv):
        return (pv - self.setpoint)

    def update_setpoint(self, setpoint):
        self.setpoint = setpoint

    def update_gain(self, kp, ki, kd):
        if kp:
            self.kp = kp
        if ki:
            self.ki = ki
        if kd:
            self.kd = kd

    def update_limit(self, min_limit, max_limit):
        self.min_limit = min_limit
        self.max_limit = max_limit

    def reset(self):
        # Default gain
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0

        self.setpoint = 0
        self.last_error = 0
        self.cum_error = 0

    def calculate_pid(self, pv, sample_time_sec, invert_output=False):
        # Get offset
        offset = self.get_error_offset(pv)
        if offset == 0:
            self.cum_error = 0 # reset cumulated error
            pid = 0
        else:
            # Get proportional
            p_val = offset * self.kp
            # Get derivative
            d_val = ((offset - self.last_error) / sample_time_sec) * self.kd
            # Get cumulative error
            self.cum_error += (offset * self.ki)

            # Calculate PID
            pid = int(p_val + d_val + self.cum_error)
            # Invert output if enabled
            if invert_output:
                pid = pid * -1

            # Limit output
            if pid > self.max_limit: pid = self.max_limit
            if pid < self.min_limit: pid = self.min_limit

        # Keep for next iteration
        self.last_error = offset

        return pid
