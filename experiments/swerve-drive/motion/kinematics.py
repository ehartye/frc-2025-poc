"""Four independent modules in the robot frame: +X forward, +Y left, positive yaw CCW."""

from dataclasses import dataclass
import math
import numpy as np

ORDER = ("FL", "FR", "RL", "RR")
TAU = 2 * math.pi


def wrapped(angle):
    return (angle + math.pi) % TAU - math.pi


def vector_matrix(centers):
    """Map a chassis twist to FL, FR, RL, RR wheel vectors at the supplied axis centers."""
    centers = np.asarray(centers, dtype=float)
    if centers.shape != (4, 2) or not np.isfinite(centers).all():
        raise ValueError("Four finite module centers in metres are required")
    result = np.zeros((4, 2, 3))
    result[:, 0, 0] = result[:, 1, 1] = 1
    result[:, 0, 2] = -centers[:, 1]
    result[:, 1, 2] = centers[:, 0]
    if np.linalg.matrix_rank(result.reshape(8, 3)) != 3:
        raise ValueError("Module centers cannot resolve chassis yaw")
    return result


def module_targets(twist, centers, measured_angles, speed_limit, pivot=(0.0, 0.0)):
    matrix = vector_matrix(np.asarray(centers) - np.asarray(pivot))
    vectors = matrix @ np.asarray(twist)
    speeds = np.linalg.norm(vectors, axis=1)
    if speed_limit <= 0:
        raise ValueError("Positive common wheel-speed limit required")
    speeds *= min(1.0, speed_limit / max(speeds.max(), 1e-12))
    angle = np.arctan2(vectors[:, 1], vectors[:, 0])
    delta = wrapped(angle - measured_angles)
    reverse = np.abs(delta) > math.pi / 2
    delta = np.where(reverse, wrapped(delta + math.pi), delta)
    delta = np.where(speeds > 1e-9, delta, 0.0)
    return np.where(reverse, -speeds, speeds), measured_angles + delta


@dataclass(frozen=True)
class EncoderMap:
    wheel_diameter_m: float
    drive_ratio: float
    zero_rad: np.ndarray
    steer_sign: np.ndarray
    drive_sign: np.ndarray
    coupling_rotor_turns_per_azimuth_turn: np.ndarray

    def __post_init__(self):
        for values in (
            self.zero_rad,
            self.steer_sign,
            self.drive_sign,
            self.coupling_rotor_turns_per_azimuth_turn,
        ):
            if np.shape(values) != (4,) or not np.isfinite(values).all():
                raise ValueError("Every ordered module needs a finite calibration")
        if (
            self.wheel_diameter_m <= 0
            or self.drive_ratio <= 0
            or not all(
                np.isin(v, [-1, 1]).all() for v in (self.steer_sign, self.drive_sign)
            )
        ):
            raise ValueError("Invalid drive reduction, diameter or sense")

    def raw(self, accumulated_wheel_m, azimuth, counts):
        circum = math.pi * self.wheel_diameter_m
        turns = self.drive_ratio * accumulated_wheel_m / circum
        turn_scale = self.coupling_rotor_turns_per_azimuth_turn / TAU
        rotor = self.drive_sign * (turns + turn_scale * azimuth)
        absolute = np.mod(
            np.rint((self.zero_rad + self.steer_sign * azimuth) * counts / TAU), counts
        )
        return rotor, absolute.astype(int)

    def angle(self, absolute, counts):
        return wrapped(self.steer_sign * (absolute * TAU / counts - self.zero_rad))

    def delta_distance(self, rotor_delta, unwrapped_azimuth_delta):
        coupled = (
            self.coupling_rotor_turns_per_azimuth_turn * unwrapped_azimuth_delta / TAU
        )
        roll = self.drive_sign * rotor_delta - coupled
        return roll * (math.pi * self.wheel_diameter_m / self.drive_ratio)


def field_to_body(vector, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * vector[0] + s * vector[1], -s * vector[0] + c * vector[1]])


def body_to_field(vector, yaw):
    return field_to_body(vector, -yaw)


def integrate(pose, body_increment):
    """Apply an SE(2) body increment, including translation while turning."""
    dx, dy, dh = body_increment
    k = np.sinc(dh / TAU)
    distance = body_to_field([k * dx, k * dy], pose[2] + dh / 2)
    return pose + np.array([distance[0], distance[1], dh])
