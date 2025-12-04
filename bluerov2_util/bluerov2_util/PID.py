import time
from math import pi
class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, i_limit=None, angle = False):
        self.kp = float(kp); self.ki = float(ki); self.kd = float(kd)
        self.i_limit = None if i_limit is None else float(i_limit)
        self.integrator = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        self.prev_d = 0.0
        self.angle_controller = angle

    def reset(self):
        self.integrator = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        self.prev_d = 0.0

    def normalize_angle(self, angle):
        """ Ensure angle remains within [-π, π] """
        return (angle + pi) % (2 * pi) - pi

    def update(self, setpoint, measurement, feedforward=0.0, now=None):
        if now is None:
            now = time.time()
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = now - self.prev_time
        
        error = setpoint - measurement
        if self.angle_controller:
            error = self.normalize_angle(error)
            # print("Angle error before normalization:", error," After normalization:", error)
        # P
        p = self.kp * error

        # I
        if dt > 0.0:
            self.integrator += error * dt * self.ki
            if self.i_limit is not None:
                self.integrator = max(min(self.integrator, self.i_limit), -self.i_limit)
        i = self.integrator

        # D (filtered)
        d = 0.0
        if dt > 0.0:
            raw_d = (error - self.prev_error) / dt
            ## Todo:: Impelement kind of filter
            d = raw_d
            d *= self.kd

        # output
        out = p + i + d + feedforward
       

        # save state
        self.prev_error = error
        self.prev_time = now
        self.prev_d = d

        return out, dict(p=p, i=i, d=d, error=error)
