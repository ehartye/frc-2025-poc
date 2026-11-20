"""Finite 2025 trough force law in the attained field frame, matching SpatialContact.java."""

import math
import numpy as np

R, HALF = 0.05715, 0.1508125
M = 0.816466266
I = M * (R**2 + 0.0508**2) / 2
IP = M * (3 * (R**2 + 0.0508**2) + 4 * HALF**2) / 12
STIFF = 60000.0
DAMP = 1.2 * math.sqrt(STIFF * M)


class Trough:
    def __init__(self, target, dock, stations=17, mu=0.35):
        dx, dy, yaw = dock
        c, s = math.cos(yaw), math.sin(yaw)
        self.rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        self.pos = np.array([dx / 1000 - 0.490, dy / 1000, 0.0])
        slope = target["floor_slope"]
        self.scale = math.hypot(1, slope)
        self.floor = np.array([-slope, 0.0, 1.0]) / self.scale
        self.intercept = target["floor_intercept_m"]
        self.depth, self.top = target["lip_depth_m"], target["lip_top_m"]
        self.inset = target["panel_inset_m"]
        self.panels = []
        for p in target["panels"]:
            xy = np.array(p["bounds_xy"]) * [self.scale, 1.0]
            delta = np.roll(xy, -1, axis=0) - xy
            winding = np.sign(np.sum(xy[:, 0] * delta[:, 1] - xy[:, 1] * delta[:, 0]))
            normal = (
                winding
                * np.column_stack((-delta[:, 1], delta[:, 0]))
                / np.linalg.norm(delta, axis=1)[:, None]
            )
            holes = np.array(p["holes_xy_r"]) * [self.scale, 1.0, 1.0]
            self.panels.append((normal, np.sum(xy * normal, axis=1), holes))
        self.s = np.linspace(-HALF, HALF, stations)
        self.weights = np.ones(stations) / (stations - 1)
        self.weights[[0, -1]] *= 0.5
        self.n = stations
        self.mu = mu

    def state(self, w):
        P, V, A, L = w
        return self.pos + self.rot @ P, self.rot @ V, self.rot @ A, self.rot @ L

    def solve(self, w):
        p, v, a, l = self.state(w)
        force = np.zeros(3)
        torque = np.zeros(3)
        span = [0.0, 0.0]
        normals = [0.0, 0.0]
        compression = 0.0
        if (
            p[2] - HALF * abs(a[2]) - R > max(self.top, self.intercept + 0.25 * 0.26795)
            or p[0] + HALF * abs(a[0]) + R < 0
        ):
            return force, torque, span, normals, compression
        omega = l / IP + (1 / I - 1 / IP) * a * (l @ a)
        base = self.s[:, None] * a
        points = p + base
        for face in (0, 1):
            if face == 0:
                normal = np.broadcast_to(self.floor, points.shape)
                gap = points @ self.floor - self.intercept * self.floor[2]
            else:
                dr = points[:, [0, 2]] - np.clip(
                    points[:, [0, 2]], [0.0, 0.003175], [self.depth, self.top]
                )
                gap = np.linalg.norm(dr, axis=1)
                normal = np.zeros_like(points)
                normal[:, [0, 2]] = dr / np.maximum(1e-12, gap)[:, None]
            proj = normal @ a
            hr = np.sqrt(np.maximum(1e-18, 1.0 - proj**2))
            valid = gap <= R * hr
            if face:
                valid &= gap > 1e-12
            if not np.any(valid):
                continue
            arm = base - R * (normal - proj[:, None] * a) / hr[:, None]
            spots = p + arm
            if face == 0:
                xy = spots[:, :2] * [self.scale, 1.0]
                inside = np.zeros(self.n, dtype=bool)
                for ns, offset, holes in self.panels:
                    test = np.all(xy @ ns.T - offset >= self.inset, axis=1)
                    if np.any(test) and len(holes):
                        ds = xy[:, None, :] - holes[None, :, :2]
                        test &= np.all(
                            np.sum(ds * ds, axis=2) > (holes[:, 2] + self.inset) ** 2,
                            axis=1,
                        )
                    inside |= test
                valid &= inside
            else:
                valid &= (abs(spots[:, 1]) >= 0.008) & (abs(spots[:, 1]) <= 0.464)
            if not np.any(valid):
                continue
            depth = R * hr - gap
            speed = v + np.cross(omega, arm)
            vn = np.sum(speed * normal, axis=1)
            fn = self.weights * np.maximum(0.0, STIFF * depth - DAMP * vn) * valid
            engaged = fn > 0
            if not np.any(engaged):
                continue
            tan = speed - vn[:, None] * normal
            slip = np.linalg.norm(tan, axis=1)
            friction = -self.mu * fn * np.tanh(slip / 0.01) / np.maximum(1e-14, slip)
            fs = fn[:, None] * normal + friction[:, None] * tan
            force += fs.sum(axis=0)
            torque += np.cross(arm, fs).sum(axis=0)
            span[face] = float(np.ptp(self.s[engaged]))
            normals[face] = float(fn.sum())
            compression = max(compression, float(depth[engaged].max()))
        return self.rot.T @ force, self.rot.T @ torque, span, normals, compression
