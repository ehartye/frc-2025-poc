"""Independent belt surfaces and a discrete, idler-encoder position-trim challenge."""

import numpy as np

QUANTUM_M = 0.060 / 4096


class BeltDrive:
    def __init__(self, acceleration, quantized=False, stuck=False):
        self.acceleration = acceleration
        self.quantized = quantized
        self.stuck = stuck
        self.speed = np.zeros(2)
        self.actual = np.zeros(2)
        self.goal = np.zeros(2)
        self.command = np.zeros(2)
        self.peak_position_error = 0.0
        self.age = 0.0
        self.steps = 0
        self.follow_fault = False

    def advance(self, target, dt):
        if self.quantized:
            # At 200 Hz, only 14.65 micrometre idler angle quanta reach the servo.
            if self.age < 1.0e-10:
                count = np.rint(self.actual / QUANTUM_M) * QUANTUM_M
                self.goal += 0.005 * target
                error = self.goal - count
                self.peak_position_error = max(
                    self.peak_position_error, max(abs(error))
                )
                if max(abs(error)) > 0.00025:
                    self.follow_fault = True
                # One finite signed 5 ms pulse per control tick. Reverse only after
                # a full observed count of accumulated position error.
                self.command = np.where(
                    abs(error) >= QUANTUM_M, np.sign(error) * 0.006, 0.0
                )
            self.age += dt
            if self.age >= 0.005 - 1.0e-9:
                self.age = 0.0
            demand = self.command
        else:
            demand = target
        before = self.speed.copy()
        self.speed += np.clip(
            demand - self.speed, -self.acceleration * dt, self.acceleration * dt
        )
        if self.stuck:
            self.speed[:] = 0.0
        self.actual += 0.5 * (before + self.speed) * dt
        self.steps += 1
        return self.speed
