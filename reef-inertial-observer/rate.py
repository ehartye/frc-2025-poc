"""Quantized 100 Hz IMU acquisition and the bound at camera capture time."""

import math

RATE_BIAS = math.radians(0.02)
GAIN_ERROR = 0.001
ANGLE_QUANTUM = math.radians(0.001)
ANGULAR_ACCEL = 1.5


class RatePackets:
    """Produce timestamped relative IMU rotations from an independent desktop plant."""

    def __init__(self, bias, gain_error, fault):
        self.bias = math.radians(bias)
        self.gain = 1 + gain_error
        self.fault = fault
        self.packets = {}
        self.origin = None

    def sample(self, stamp, yaw):
        # Physical plant only. The estimator receives a stamped quantized relative
        # integral. The actual absolute heading is unavailable at its interface.
        if self.origin is None:
            self.origin = yaw
        if self.fault == "gyro-dropout" and stamp >= 1550:
            return
        relative = (yaw - self.origin) * self.gain + self.bias * stamp / 1000
        self.packets[stamp] = round(relative / ANGLE_QUANTUM) * ANGLE_QUANTUM

    def at_capture(self, stamp):
        # Camera service is 15 ms after capture, IMU service 20 ms after sampling.
        latest = (stamp - 5) // 10 * 10
        previous = latest - 10
        if latest not in self.packets or previous not in self.packets:
            return None
        age = (stamp - latest) / 1000
        angle = self.packets[latest]
        rate = (angle - self.packets[previous]) / 0.010
        estimate = angle + age * rate
        # Two consecutive, physically sampled IMU angles extrapolate up to 10 ms.
        # Quantization is amplified by the two weights. Bounded angular acceleration
        # covers using the preceding interval's mean instead of the current rate.
        error = (
            ANGLE_QUANTUM / 2 * (1 + 2 * age / 0.010)
            + ANGULAR_ACCEL * age * (age + 0.010) / 2
        )
        return estimate, error
