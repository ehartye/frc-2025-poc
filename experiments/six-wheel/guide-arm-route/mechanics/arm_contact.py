"""Advance annular guide contact on each side's modal basis and recover lateral loads."""

import math
import numpy as np

from backing import GuideResponse
from beam import guide


class ContactResponse(GuideResponse):
    """Derived from the registered contact solver with independent basis lookup on each side."""

    def __init__(self, beam):
        super().__init__(beam)
        self.paired = beam.w.ndim == 2
        self.capture = None
        if self.paired:
            self.q = np.zeros_like(beam.w)
            self.v = np.zeros_like(beam.w)

    def step(self, pipe, dt):
        g = pipe.guide
        vert = guide.feed.HALF * abs(pipe.a[2]) + guide.feed.R * math.sqrt(
            max(0.0, 1 - pipe.a[2] ** 2)
        )
        if pipe.p[2] + vert < 0.625 or pipe.p[2] - vert > 0.813:
            return
        # Fast-forward the exact unforced modes over absence of geometric overlap.
        if pipe.time - self.last > 1e-12:
            self.advance(pipe.time - self.last, np.zeros_like(self.q))
        self.last = pipe.time + dt
        ring = guide.annular_points(pipe.a, g.sectors, g.radial_bins)
        load = np.zeros_like(self.q)
        forces = np.zeros(2)
        centers = np.zeros(2)
        for j, side in enumerate((-1, 1)):
            arms = ring + side * guide.feed.HALF * pipe.a
            x, y, z = (pipe.p + arms).T
            inside = (
                (z >= 0.625)
                & (z <= 0.813)
                & (
                    abs(x - guide.x_center(z) - g.x_offset_mm / 1000)
                    < 0.0005 * g.x_width_mm
                )
            )
            if not np.any(inside):
                continue
            z, y, r = z[inside], y[inside], arms[inside]
            phi, slope = self.beam.at(z, j) if self.paired else self.beam.at(z)
            travel = phi @ self.q[j]
            taper = np.where(z > 0.770, 0.038, 0.0) + slope @ self.q[j]
            norm = np.sqrt(1 + taper * taper)
            penetration = (side * y - guide.feed.HALF - guide.gap(z, g) - travel) / norm
            active = penetration > 0
            if not np.any(active):
                continue
            phi, r = phi[active], r[active]
            n = np.column_stack(
                (np.zeros(len(r)), side / norm[active], -taper[active] / norm[active])
            )
            actual = pipe.v + np.cross(pipe.angular(pipe.l), r)
            relative = actual.copy()
            relative[:, 1] -= side * (phi @ self.v[j])
            closing = np.sum(relative * n, axis=1)
            compression = penetration[active]
            pressure = np.maximum(
                0, g.stiffness_n_mm * 1000 * compression + g.damping_n_s_m * closing
            ) / len(ring)
            tangent = relative - closing[:, None] * n
            slide = (
                -pressure[:, None]
                * g.friction
                * tangent
                / np.maximum(0.02, np.linalg.norm(tangent, axis=1))[:, None]
            )
            f = -pressure[:, None] * n + slide
            force = f.sum(axis=0)
            pipe.l += np.sum(np.cross(r, f), axis=0) * dt
            pipe.v += force / guide.feed.M * dt
            load[j] = phi.T @ (-side * f[:, 1])
            if self.capture is not None:
                self.capture(pipe.time + dt, j, z[active] * 1000, -side * f[:, 1])
            self.pipe_work[j] += float(np.sum(f * actual)) * dt
            self.friction[j] -= float(np.sum(slide * tangent)) * dt
            self.impulse[j] += force * dt
            forces[j] = np.sum(pressure)
            if forces[j] > 0:
                centers[j] = pressure @ z[active] / forces[j]
                if self.first_loaded is None:
                    self.first_loaded = pipe.time
                self.last_loaded = pipe.time
            pipe.peak_guiding_n = max(pipe.peak_guiding_n, float(forces[j]))
            pipe.max_local_compression_mm = max(
                pipe.max_local_compression_mm, 1000 * float(compression.max())
            )
        self.advance(dt, load)
        self.peak_n = np.maximum(self.peak_n, forces)
        if self.first_loaded is not None:
            # A 1 mm output grid along the face, every integrator step in the guide window.
            if self.paired:
                disp = np.einsum("sj,skj->sk", self.q, self.beam.phi[:, ::4, :])
            else:
                disp = self.q @ self.beam.phi[::4].T
            self.peak_mm = np.maximum(self.peak_mm, 1000 * np.max(abs(disp), axis=1))
            self.trace.append(
                [
                    pipe.time + dt,
                    *forces,
                    *centers,
                    *disp[:, 0],
                    *disp[:, -1],
                    *np.max(abs(disp), axis=1),
                    *self.energy(),
                    *self.input,
                ]
            )
