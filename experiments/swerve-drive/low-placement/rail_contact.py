"""Compression-only telescoping carriage and plunger response to a moving free payload.

Coordinates, inertias, support contact rows and the 13 Ritz plate modes come
from the registered extension fixture. A payload supplies an external wrench
and distributed plate load instead of being permanently glued to that mass.
"""

import numpy as np

from cam_model import candidate_gaps
import dynamics
import dynamics_run as rack


class RailContact:
    def __init__(self, state, dt, travel, speed):
        geo, keeper, _ = rack.registration()
        m, k, c, gravity, pitch, stations, weights, _ = dynamics.matrices(state)
        mass = rack.read(dynamics.HERE / "balance.json")["pipeMassKg"]
        self.rails, self.rail_slope = dynamics.contact_rows(travel, geo, keeper)
        self.speed = speed
        self.dt = dt
        n = len(gravity)
        self.b = np.zeros((8, n))
        self.b[:6, :6] = self.rails[0]
        self.ks = np.r_[np.full(6, 1e7), 0.0, 0.0]
        self.gaps = np.array([0.001, 0.001, 0.00035] * 2)
        # Equilibrium includes the carried weight. Dynamics below remove it once,
        # applying only actual unilateral tube reactions to the moving receiver.
        q0 = rack.equilibrium(m, k, gravity, self.b[:6], self.gaps, self.ks[:6])
        disturbance, self.v, self.roll, self.roll_rate = dynamics.project_legacy(state)
        self.q = q0 + disturbance
        self.start = self.q.copy()
        m -= (mass * weights[:, None] * stations).T @ stations
        gravity += dynamics.G * mass * (weights @ stations)
        pitch[5] -= mass * (state["p"][2] - 0.467)
        if np.linalg.eigvalsh(m).min() <= 0:
            raise ValueError("Free rack mass is not positive definite")
        self.cd = np.r_[
            0.12
            * np.sqrt(
                self.ks[:6]
                / np.einsum("ij,jk,ik->i", self.b[:6], np.linalg.inv(m), self.b[:6])
            ),
            1500.0,
            1500.0,
        ]
        inv = np.linalg.inv(m + 0.5 * dt * c + 0.25 * dt**2 * k)
        self.V = inv @ (m - 0.5 * dt * c - 0.25 * dt**2 * k)
        self.map = dt * inv
        self.D = -self.map @ k
        self.gravity = self.map @ gravity
        self.drive = self.map @ pitch
        self.cap = rack.table()
        self.grid = np.arange(len(self.cap)) * 0.00025
        clear = candidate_gaps(self.grid)
        self.clear_slope = np.gradient(
            np.where(np.isfinite(clear), clear, 0.0031), self.grid, axis=0
        )
        self.cap_slope = np.gradient(self.cap, self.grid, axis=0)
        self.indices = np.clip(
            np.rint(travel / 0.00025).astype(int), 0, len(self.cap) - 1
        )
        self.clear = candidate_gaps(travel)
        self.plunger = state["plunger"].copy()
        self.peak = np.zeros(8)
        self.max_pitch = 0.0
        self.max_heave = 0.0

    def advance(self, i, a, load):
        dt, b, ks, cd = self.dt, self.b, self.ks, self.cd
        j = self.indices[i]
        b[:6, :6] = self.rails[i]
        b[6:] = self.cap[j]
        gap = np.r_[self.gaps + b[:6] @ self.q, self.clear[i] + b[6:] @ self.q]
        relative = (
            np.r_[
                self.rail_slope[i] @ self.q[:6],
                self.cap_slope[j] @ self.q + self.clear_slope[j],
            ]
            * self.speed[i]
        )
        self.plunger = np.maximum(0, self.plunger - dt * 0.008 / 0.3)
        vfree = (
            self.V @ self.v
            + self.D @ self.q
            + self.gravity
            + self.drive * a
            + self.map @ load
        )
        rate = 0.5 * b @ (self.v + vfree) + relative
        active = np.flatnonzero(
            -(gap + 0.5 * dt * rate) >= np.r_[np.zeros(6), self.plunger]
        )
        fn = np.zeros(8)
        fixed = 0.25 * dt * ks + 0.5 * cd
        while len(active):
            R = b[active]
            rhs = -(0.5 * dt * ks[active] + cd[active]) * rate[active]
            rails = active < 6
            rhs[rails] -= ks[active[rails]] * gap[active[rails]]
            rhs -= fixed[active] * (R @ (self.map @ (b.T @ fn)))
            force = np.linalg.solve(
                np.eye(len(active)) + (fixed[active, None] * R) @ self.map @ R.T, rhs
            )
            over = np.flatnonzero((active >= 6) & (force > 750.0))
            if len(over):
                fn[active[over[0]]] = 750.0
                active = np.delete(active, over[0])
            elif min(force) < 0:
                active = np.delete(active, np.argmin(force))
            else:
                fn[active] = force
                break
        vn = vfree + self.map @ (b.T @ fn)
        self.q += 0.5 * dt * (self.v + vn)
        self.v = vn
        self.plunger = np.maximum(self.plunger, -gap[6:])
        self.peak = np.maximum(self.peak, fn)
        self.max_pitch = max(self.max_pitch, abs(self.q[5]))
        self.max_heave = max(self.max_heave, abs(self.q[4]))
