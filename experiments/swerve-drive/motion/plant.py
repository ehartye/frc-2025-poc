"""Integrate independent wheel inertia, steering, tire contact and a planar rigid chassis."""

import json
import math
from pathlib import Path
import numpy as np

from kinematics import (
    TAU,
    EncoderMap,
    field_to_body,
    body_to_field,
    integrate,
    module_targets,
    vector_matrix,
    wrapped,
)

HERE = Path(__file__).resolve().parent
SETTINGS = json.loads((HERE / "model.json").read_bytes())


def historical_torque(demand, rate, motor, voltage):
    """Four-quadrant motor controller, back EMF and phase current at the actual rotor speed."""
    r = 12 / motor["stallCurrentA"]
    kt = motor["stallTorqueNm"] / motor["stallCurrentA"]
    kv = (12 - r * motor["freeCurrentA"]) / (motor["freeRpm"] * TAU / 60)
    emf = kv * rate
    amp = np.clip(demand / kt, -motor["phaseLimitA"], motor["phaseLimitA"])
    volts = np.clip(emf + r * amp, -voltage, voltage)
    amp = np.clip((volts - emf) / r, -motor["phaseLimitA"], motor["phaseLimitA"])
    return kt * amp, amp, volts


def motor_constants(motor):
    resistance = 12 / motor["stallCurrentA"]
    free_rate = motor["freeRpm"] * TAU / 60
    constant = (12 - resistance * motor["freeCurrentA"]) / free_rate
    viscous_drag = constant * motor["freeCurrentA"] / free_rate
    return resistance, constant, viscous_drag


def available_torque(demand, rate, motor, voltage):
    """Fit free speed/current and stalled resistance with reciprocal SI constants."""
    resistance, constant, drag = motor_constants(motor)
    emf = constant * rate
    limit = motor["phaseLimitA"]
    lower = np.maximum(-limit, (-voltage - emf) / resistance)
    upper = np.minimum(limit, (voltage - emf) / resistance)
    if np.any(lower > upper):
        raise ValueError("No current satisfies both terminal-voltage and phase limits")
    current = np.clip((demand + drag * rate) / constant, lower, upper)
    return constant * current - drag * rate, current, emf + resistance * current


def transmission_factor(power, efficiency):
    """Torque factor from motor to joint for forward power or backdrive."""
    return np.where(power < 0, 1 / efficiency, efficiency)


class Plant:
    def __init__(
        self,
        centers,
        pose,
        *,
        mass=65.0,
        inertia=5.0,
        mu=0.8,
        radii=None,
        loads=None,
        coupling=None,
        encoder_map=None,
        support=None,
        initial_angles=None,
        model="reciprocal",
    ):
        if model not in ("historical", "reciprocal"):
            raise ValueError("Unknown motor model")
        self.reciprocal = model == "reciprocal"
        self.motor = available_torque if self.reciprocal else historical_torque
        self.power_samples = []
        self.balance = {
            "maxMotorIdentityErrorW": 0.0,
            "minGearLossW": None,
            "reverseGearInputJ": 0.0,
            "gearLossJ": 0.0,
            "observations": 0,
        }
        self.xy = np.asarray(centers, dtype=float)
        self.matrix = vector_matrix(self.xy)
        self.mass, self.yaw_inertia, self.mu = mass, inertia, mu
        self.pose = np.array(pose, dtype=float)
        self.support = support
        self.cg = np.zeros(2) if support is None else support.cg
        self.arms = self.xy - self.cg
        self.velocity = np.zeros(3)  # X,Y in field frame and yaw rate
        self.radius = np.full(4, 0.0381) if radii is None else np.asarray(radii)
        self.load = (
            np.full(4, mass * 9.80665 / 4) if loads is None else np.asarray(loads)
        )
        if (
            (self.radius <= 0).any()
            or (self.load <= 0).any()
            or abs(sum(self.load) - mass * 9.80665) > 1e-3
        ):
            raise ValueError(
                "Each tire needs a positive radius and consistent normal load"
            )
        self.angle = (
            np.zeros(4)
            if initial_angles is None
            else np.asarray(initial_angles, dtype=float).copy()
        )
        self.azimuth_rate = np.zeros(4)
        self.wheel_speed = np.zeros(4)
        self.wheel_turns = np.zeros(4)
        self.target_speed = np.zeros(4)
        self.coupling = (
            np.zeros(4) if coupling is None else np.asarray(coupling, dtype=float)
        )
        self.encode = encoder_map or EncoderMap(
            0.0762, 5.5, np.zeros(4), np.ones(4), np.ones(4), self.coupling
        )
        self.peak = {
            "drivePhaseA": 0.0,
            "steerPhaseA": 0.0,
            "contactUtilization": 0.0,
            "wheelMps": 0.0,
            "azimuthRadS": 0.0,
        }
        self.peak_lateral_by_wheel = np.zeros(4)
        self.steps = 0

    def sensors(self):
        # The rotor is physically tied to wheel angle. Rolling distance uses the
        # estimated rolling diameter only when interpreting the measured turns.
        logical = self.wheel_turns * math.pi * self.encode.wheel_diameter_m
        rotor, absolute = self.encode.raw(
            logical, self.angle, SETTINGS["sensors"]["angleCountsPerTurn"]
        )
        quantum = SETTINGS["sensors"]["driveCountsPerMotorTurn"]
        return np.rint(rotor * quantum) / quantum, absolute

    def advance(
        self,
        body_target,
        measured_angle,
        *,
        stopped=False,
        disabled=False,
        locked_steer=None,
        voltage=9.5,
    ):
        cfg = SETTINGS
        ds = cfg["drive"]
        ss = cfg["steer"]
        c = cfg["control"]
        tire = cfg["contact"]
        target, goal = module_targets(
            body_target, self.xy, measured_angle, c["maxWheelMps"]
        )
        delta = wrapped(goal - measured_angle)
        steer_goal_rate = np.clip(12 * delta, -ss["rateLimitRadS"], ss["rateLimitRadS"])
        ramp = c["maxCommandAccelMps2"] * c["periodS"]
        compensated = target * np.maximum(0, np.cos(delta))
        self.target_speed += np.clip(compensated - self.target_speed, -ramp, ramp)
        if stopped:
            self.target_speed[:] = 0
            steer_goal_rate[:] = 0
        dt = c["physicsStepS"]
        old_rotor_rate = (
            self.wheel_speed / self.radius * ds["ratio"]
            + self.coupling * self.azimuth_rate
        )
        old_speed = self.wheel_speed.copy()
        old_rate = self.azimuth_rate.copy()
        for _ in range(round(c["periodS"] / dt)):
            heading = np.c_[np.cos(self.angle), np.sin(self.angle)]
            side = np.c_[-heading[:, 1], heading[:, 0]]
            body_v = field_to_body(self.velocity[:2], self.pose[2])
            per_wheel = self.matrix @ np.r_[body_v, self.velocity[2]]
            longitudinal = tire["longitudinalNSPerM"] * (
                self.wheel_speed - np.sum(per_wheel * heading, axis=1)
            )
            lateral = -tire["lateralNSPerM"] * np.sum(per_wheel * side, axis=1)
            norm = np.hypot(longitudinal, lateral)
            used_load = self.load.copy()
            scale = np.minimum(1.0, self.mu * self.load / np.maximum(1e-12, norm))
            longitudinal *= scale
            lateral *= scale
            contact = longitudinal[:, None] * heading + lateral[:, None] * side
            drive_rate = (
                self.wheel_speed / self.radius * ds["ratio"]
                + self.coupling * self.azimuth_rate
            )
            requested = ds["speedGainNSPerM"] * (self.target_speed - self.wheel_speed)
            df = ds["efficiency"]
            if self.reciprocal:
                df = transmission_factor(requested * drive_rate, df)
            motor, drive_a, drive_v = self.motor(
                requested * self.radius / (ds["ratio"] * df),
                drive_rate,
                ds,
                voltage,
            )
            if disabled:
                motor, drive_a, drive_v = self.motor(
                    -1e6 * np.sign(drive_rate), drive_rate, ds, 0.0
                )
            if self.reciprocal:
                df = transmission_factor(motor * drive_rate, ds["efficiency"])
            wheel_force = motor * ds["ratio"] * df / self.radius
            wheel_acc = (wheel_force - longitudinal) / ds["equivalentRotatingMassKg"]
            drag = (
                ss["dragCoefficient"]
                * self.load
                * ss["dragArmM"]
                * np.tanh(self.azimuth_rate / 0.08)
            )
            steer_acc = np.clip(
                65 * (steer_goal_rate - self.azimuth_rate),
                -ss["accelLimitRadS2"],
                ss["accelLimitRadS2"],
            )
            coupling_torque = motor * self.coupling * df
            steer_torque = ss["inertiaKgM2"] * steer_acc + drag - coupling_torque
            steer_rate = self.azimuth_rate * ss["ratio"]
            sf = ss["efficiency"]
            if self.reciprocal:
                sf = transmission_factor(steer_torque * steer_rate, sf)
            output, steer_a, steer_v = self.motor(
                steer_torque / (ss["ratio"] * sf),
                self.azimuth_rate * ss["ratio"],
                ss,
                voltage,
            )
            if disabled:
                output, steer_a, steer_v = self.motor(
                    -1e6 * np.sign(self.azimuth_rate),
                    self.azimuth_rate * ss["ratio"],
                    ss,
                    0.0,
                )
            if self.reciprocal:
                sf = transmission_factor(output * steer_rate, ss["efficiency"])
            swivel_torque = output * ss["ratio"] * sf
            if self.reciprocal:
                self.power_samples.append((drive_v, drive_a, steer_v, steer_a))
                for cfg_motor, torque, rate, current, volt, joint_w in (
                    (
                        ds,
                        motor,
                        drive_rate,
                        drive_a,
                        drive_v,
                        wheel_force * self.wheel_speed
                        + coupling_torque * self.azimuth_rate,
                    ),
                    (
                        ss,
                        output,
                        steer_rate,
                        steer_a,
                        steer_v,
                        swivel_torque * self.azimuth_rate,
                    ),
                ):
                    resistance, _, friction = motor_constants(cfg_motor)
                    shaft_w = torque * rate
                    loss = shaft_w - joint_w
                    error = (
                        volt * current
                        - shaft_w
                        - resistance * current**2
                        - friction * rate**2
                    )
                    ledger = self.balance
                    ledger["maxMotorIdentityErrorW"] = max(
                        ledger["maxMotorIdentityErrorW"], np.max(abs(error))
                    )
                    minimum = np.min(loss)
                    ledger["minGearLossW"] = (
                        minimum
                        if ledger["minGearLossW"] is None
                        else min(ledger["minGearLossW"], minimum)
                    )
                    ledger["reverseGearInputJ"] += dt * np.sum(
                        np.maximum(0.0, -joint_w)
                    )
                    ledger["gearLossJ"] += dt * np.sum(loss)
                    ledger["observations"] += len(loss)
            actual_acc = (swivel_torque + coupling_torque - drag) / ss["inertiaKgM2"]
            old_azimuth = self.azimuth_rate.copy()
            self.azimuth_rate += actual_acc * dt
            if locked_steer is not None:
                self.azimuth_rate[locked_steer] = 0.0
            self.angle += 0.5 * (old_azimuth + self.azimuth_rate) * dt
            before = self.wheel_speed.copy()
            self.wheel_speed += wheel_acc * dt
            self.wheel_turns += (
                0.5 * (before + self.wheel_speed) * dt / (TAU * self.radius)
            )
            cg_acc = contact.sum(axis=0) / self.mass
            angular_acc = (
                np.sum(
                    self.arms[:, 0] * contact[:, 1] - self.arms[:, 1] * contact[:, 0]
                )
                / self.yaw_inertia
            )
            origin_acc = (
                cg_acc
                - angular_acc * np.array([-self.cg[1], self.cg[0]])
                + self.velocity[2] ** 2 * self.cg
            )
            acceleration = np.r_[body_to_field(origin_acc, self.pose[2]), angular_acc]
            if self.support is not None:
                self.load = self.support.forces(
                    *origin_acc, self.velocity[2], angular_acc
                )
            self.pose += self.velocity * dt + 0.5 * acceleration * dt * dt
            self.velocity += acceleration * dt
            self.peak_lateral_by_wheel = np.maximum(
                self.peak_lateral_by_wheel, abs(lateral)
            )
            for label, value in (
                ("drivePhaseA", abs(drive_a)),
                ("steerPhaseA", abs(steer_a)),
                (
                    "contactUtilization",
                    scale * norm / np.maximum(1e-12, self.mu * used_load),
                ),
                ("wheelMps", abs(self.wheel_speed)),
                ("azimuthRadS", abs(self.azimuth_rate)),
            ):
                self.peak[label] = max(self.peak[label], float(max(value)))
        self.steps += 1
        return dict(
            azimuth_rad=self.angle.copy(),
            azimuth_rate_radps=self.azimuth_rate.copy(),
            azimuth_accel_radps2=(self.azimuth_rate - old_rate) / c["periodS"],
            wheel_mps=self.wheel_speed.copy(),
            wheel_accel_mps2=(self.wheel_speed - old_speed) / c["periodS"],
            drive_force_n=longitudinal,
            lateral_force_n=lateral,
            normal_n=used_load,
            drive_rotor_rpm=drive_rate * 60 / TAU,
            steer_rotor_rpm=self.azimuth_rate * ss["ratio"] * 60 / TAU,
            target_wheel_mps=self.target_speed.copy(),
            target_azimuth_rad=goal,
            coupling_torque_nm=-coupling_torque,
            drive_rotor_accel_radps2=(
                self.wheel_speed / self.radius * ds["ratio"]
                + self.coupling * self.azimuth_rate
                - old_rotor_rate
            )
            / c["periodS"],
            drive_phase_a=drive_a,
            steer_phase_a=steer_a,
            drive_voltage_v=drive_v,
            steer_voltage_v=steer_v,
        )


class Odometry:
    def __init__(self, plant, pose, sensor_map, gyro):
        self.pose = np.array(pose, dtype=float)
        self.map = sensor_map
        self.centers = plant.xy
        self.rotor, self.absolute = plant.sensors()
        self.prev_gyro = gyro
        self.error_by_wheel = np.zeros(4)
        self.angles = self.map.angle(
            self.absolute, SETTINGS["sensors"]["angleCountsPerTurn"]
        )

    def update(self, rotor, absolute, gyro):
        angle = self.map.angle(absolute, SETTINGS["sensors"]["angleCountsPerTurn"])
        turn = wrapped(angle - self.angles)
        midpoint = self.angles + turn / 2
        distance = self.map.delta_distance(rotor - self.rotor, turn)
        wheel = np.c_[np.cos(midpoint), np.sin(midpoint)] * distance[:, None]
        yaw = wrapped(gyro - self.prev_gyro)
        chassis_xy = wheel - np.c_[-self.centers[:, 1], self.centers[:, 0]] * yaw
        translation = chassis_xy.mean(axis=0)
        self.error_by_wheel = np.linalg.norm(chassis_xy - translation, axis=1)
        self.pose = integrate(self.pose, [*translation, yaw])
        self.angles, self.rotor, self.absolute, self.prev_gyro = (
            angle,
            rotor,
            absolute,
            gyro,
        )
