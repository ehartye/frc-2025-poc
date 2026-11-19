"""Couple a local cylinder contact law to moving plate modes and moving aperture rays."""

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np
from response import HERE
from response import basis, plate

ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "shelf-contact"))
from experiment import CompliantPipe, GUIDE, Shelf, feed, source
from contact import quadrature


@lru_cache(maxsize=4)
def tables(hybrid, cutoff, seat_stiffness, installed):
    nx, ny = 13, 20
    modes, w, inertia = plate(hybrid, nx, ny, 2.2, seat_stiffness, installed)
    keep = w <= cutoff * 2 * np.pi
    selected = modes[:, keep]
    # Interpolate the modal displacement and both analytical slopes. Outside
    # this table the cylinder is beyond the supported strip.
    x = np.linspace(-0.100, 0.165, 107)
    y = np.linspace(-0.180, 0.180, 145)
    gx, gy = np.meshgrid(x, y)
    shape = []
    for dx, dy in ((0, 0), (1, 0), (0, 1)):
        vals = basis(gx.ravel(), gy.ravel(), nx, ny, dx, dy) @ selected
        shape.append(vals.reshape((len(y), len(x), -1)))
    return np.stack(shape, axis=2), w[keep], inertia[:, keep]


class CoupledPipe(CompliantPipe):
    def __init__(
        self,
        delta,
        shift,
        g,
        cfg,
        hybrid,
        cutoff,
        modal_damping,
        seat_stiffness,
        damper,
        damper_y=(-0.066, 0.066),
        running_gap=0.0,
        installed=False,
    ):
        super().__init__(delta, shift, g, cfg)
        self.table, self.freq, self.inertia = tables(
            hybrid, cutoff, seat_stiffness, installed
        )
        self.z = np.zeros_like(self.freq)
        self.zdot = self.z.copy()
        self.clock_cache = {}
        self.modal_damping = modal_damping
        self.datum = self.sample(np.array([0.1164, 0.100, 0.040]), np.zeros(3))[:, 0]
        self.max_probe = np.zeros(3)
        self.min_probe = np.zeros(3)
        self.max_contact_slope = 0.0
        self.max_deformed_mm = 0.0
        self.c = damper
        self.damper_points = self.sample(
            np.array([0.118, 0.118]), np.asarray(damper_y)
        )[:, 0]
        self.running_gap = running_gap
        self.plunger = np.zeros(2)
        self.plunger_peak = np.zeros(2)
        self.plunger_energy = np.zeros(2)
        self.plunger_speed = np.zeros(2)
        self.damper_history = []
        self.force_history = []
        self.record_mount_load = installed
        self.input_wrench = np.zeros(7)
        self.applied_dampers = np.zeros(2)
        # #105 and the couple model do not include pressure-tail constitutive
        # behavior. Resolve nominal tube material relative to three moving tails.
        self.pad_table = self.sample(np.full(3, 0.1251), np.array([-0.070, 0.0, 0.070]))
        self.pad_overlap = np.zeros(3)
        self.flush_overlap = np.zeros(3)
        self.pad_base = np.array(
            [
                [0.1251, -0.070, 0.50065],
                [0.1251, 0.0, 0.50065],
                [0.1251, 0.070, 0.50065],
            ]
        )

    def sample(self, x, y):
        ix = np.clip((x + 0.100) / 0.0025, 0.0, 106.0 - 1e-10)
        iy = np.clip((y + 0.180) / 0.0025, 0.0, 144.0 - 1e-10)
        i, j = ix.astype(int), iy.astype(int)
        fx, fy = (ix - i)[:, None, None], (iy - j)[:, None, None]
        a = self.table[j, i] * (1 - fx) + self.table[j, i + 1] * fx
        b = self.table[j + 1, i] * (1 - fx) + self.table[j + 1, i + 1] * fx
        return a * (1 - fy) + b * fy

    def advance_modes(self, dt, q):
        # Exact damped modal evolution with the specified held generalized load.
        key = round(dt, 12)
        if key not in self.clock_cache:
            w = self.freq
            d = self.modal_damping
            wd = w * np.sqrt(1 - d * d)
            e = np.exp(-d * w * dt)
            sn = e * np.sin(wd * dt) / wd
            cs = e * np.cos(wd * dt)
            self.clock_cache[key] = (cs + d * w * sn, sn, cs - d * w * sn)
        aa, sn, bb = self.clock_cache[key]
        if self.c:
            travel = self.damper_points @ self.z - self.running_gap
            speed = self.damper_points @ self.zdot
            # Unilateral ideal accessory with a finite-speed free return.
            # Viscous slope and force cap are trial inputs, not catalog ratings.
            self.plunger = np.maximum(0.0, self.plunger - dt * 0.008 / 0.3)
            compress = np.maximum(0.0, speed) * (travel >= self.plunger)
            reaction = np.minimum(750.0, self.c * compress)
            self.applied_dampers = reaction
            if np.any(reaction) and self.touch and self.time - self.touch[0] < 0.20:
                self.damper_history.append([self.time, *reaction, *travel, *compress])
            self.plunger = np.maximum(self.plunger, travel)
            self.plunger_peak = np.maximum(self.plunger_peak, self.plunger)
            self.plunger_energy += reaction * compress * dt
            self.plunger_speed = np.maximum(self.plunger_speed, compress)
            q = q - self.damper_points.T @ reaction
        if self.record_mount_load and self.touch and self.time - self.touch[0] < 0.15:
            accel = (
                q
                - 2 * self.modal_damping * self.freq * self.zdot
                - self.freq**2 * self.z
            )
            down, first_y, pitch = self.input_wrench[:3] - self.inertia @ accel
            fx, fy, roll_xy, pitch_xy = self.input_wrench[3:]
            roll = -0.18 * (2 * first_y - down) + roll_xy
            self.force_history.append(
                [
                    self.time,
                    fx,
                    fy,
                    -down,
                    roll,
                    pitch - 0.042 * down + pitch_xy,
                    *self.applied_dampers,
                ]
            )
        p = q / self.freq**2
        old = self.z - p
        self.z = aa * old + sn * self.zdot + p
        self.zdot = bb * self.zdot - self.freq**2 * sn * old
        probes = self.datum @ self.z * 1000
        self.max_probe = np.maximum(self.max_probe, probes)
        self.min_probe = np.minimum(self.min_probe, probes)
        # Place the 0.65 mm witness on the local deformed surface normal.
        deflection = self.pad_table @ self.z
        normal = np.column_stack((deflection[:, 1], deflection[:, 2], np.ones(3)))
        normal /= np.linalg.norm(normal, axis=1)[:, None]
        pts = self.pad_base.copy()
        pts[:, 2] -= deflection[:, 0] + 0.00065
        pts += 0.00065 * normal
        d = pts - self.p
        axial = d @ self.a
        radius = np.linalg.norm(d - np.outer(axial, self.a), axis=1)
        inside = np.minimum.reduce(
            [feed.HALF - abs(axial), feed.R - radius, radius - feed.RI]
        )
        self.pad_overlap = np.maximum(self.pad_overlap, inside)
        d -= 0.0008 * normal
        axial = d @ self.a
        radius = np.linalg.norm(d - np.outer(axial, self.a), axis=1)
        self.flush_overlap = np.maximum(
            self.flush_overlap,
            np.minimum.reduce(
                [feed.HALF - abs(axial), feed.R - radius, radius - feed.RI]
            ),
        )

    def floor(self, dt):
        self.last_normal_n = 0.0
        self.input_wrench[:] = 0.0
        # This check skips only the unforced approach before any plate motion.
        if (
            self.touch is None
            and self.p[2] - feed.HALF * abs(self.a[2]) - feed.R > feed.TOP
        ):
            return
        nodes, weights = quadrature(self.shelf.stations)
        axis = self.a
        base = self.p + np.outer(nodes * feed.HALF, axis)
        radial = (
            -feed.R
            * (feed.UP - axis * axis[2])
            / max(1e-12, math.sqrt(1 - axis[2] ** 2))
        )
        contact = base + radial
        # Orient each circular generator to its own local surface normal.
        for _ in range(2):
            field = self.sample(contact[:, 0], contact[:, 1])
            surf = field @ self.z
            normal = np.column_stack((surf[:, 1], surf[:, 2], np.ones(len(surf))))
            normal /= np.linalg.norm(normal, axis=1)[:, None]
            radial = normal - np.outer(normal @ axis, axis)
            radial *= -feed.R / np.linalg.norm(radial, axis=1)[:, None]
            contact = base + radial
        field = self.sample(contact[:, 0], contact[:, 1])
        surf = field @ self.z
        normal = np.column_stack((surf[:, 1], surf[:, 2], np.ones(len(surf))))
        normal /= np.linalg.norm(normal, axis=1)[:, None]
        # Relative normal gap, local surface velocity and no-tension contact.
        arm = contact - self.p
        omega = self.angular(self.l)
        vel = self.v + np.cross(omega, arm)
        vel[:, 2] += field[:, 0] @ self.zdot
        vn = (vel * normal).sum(axis=1)
        depth = np.maximum(0.0, (feed.TOP - surf[:, 0] - contact[:, 2]) * normal[:, 2])
        active = (
            (contact[:, 0] >= -0.100)
            & (contact[:, 0] <= feed.EDGE)
            & (abs(contact[:, 1]) <= 0.175)
        )
        fn = (
            0.5
            * weights
            * self.shelf.stiffness_n_m
            * depth
            * np.maximum(0.0, 1 - self.shelf.dissipation_s_m * vn)
            * active
        )
        if fn.max() > 0:
            if self.touch is None:
                self.touch = [
                    self.time,
                    self.p.tolist(),
                    self.v.tolist(),
                    omega.tolist(),
                ]
                self.first_impact_speed = float(vn[fn > 0].min())
            tangent = vel - vn[:, None] * normal
            shear = (
                -self.shelf.friction
                * fn[:, None]
                * tangent
                / np.maximum(
                    self.shelf.slip_speed_m_s, np.linalg.norm(tangent, axis=1)
                )[:, None]
            )
            force = fn[:, None] * normal + shear
            self.v += dt * force.sum(axis=0) / feed.M
            self.l += dt * np.cross(arm, force).sum(axis=0)
            q = field[:, 0].T @ force[:, 2]
            if self.record_mount_load:
                rx, ry = -force[:, 0], -force[:, 1]
                x, y, z = contact.T
                down = force[:, 2]
                self.input_wrench = np.array(
                    [
                        sum(down),
                        down @ ((y + 0.18) / 0.36),
                        down @ (x + 0.087),
                        sum(rx),
                        sum(ry),
                        -ry @ (z - 0.154),
                        rx @ (z - 0.154),
                    ]
                )
            self.last_normal_n = float(sum(fn))
            self.peak_shelf_n = max(self.peak_shelf_n, self.last_normal_n)
            self.shelf_impulse_ns += dt * self.last_normal_n
            self.peak_shelf_compression = max(
                self.peak_shelf_compression, float(max(depth[fn > 0]))
            )
            self.max_deformed_mm = max(
                self.max_deformed_mm, float(max(surf[fn > 0, 0])) * 1000
            )
            self.max_contact_slope = max(
                self.max_contact_slope,
                float(np.max(np.linalg.norm(surf[fn > 0, 1:], axis=1))),
            )
        else:
            q = 0.0
        self.advance_modes(dt, q)

    def ports(self):
        # Use the same aperture samples and range gates as the issued receiver.
        # The complete aperture moves with the local tangent plane at its center.
        hits = []
        for x, h in ((0.100, 0.004), (0.040, 0.0175)):
            down, gx, gy = self.sample(np.array([x]), np.zeros(1))[0] @ self.z
            normal = np.array([gx, gy, 1.0])
            normal /= np.linalg.norm(normal)
            tx = np.array([1.0, 0.0, -gx])
            tx /= np.linalg.norm(tx)
            ty = np.cross(normal, tx)
            origin = np.array([x, 0.0, feed.TOP - down])
            good = False
            for dx in np.linspace(-0.006, 0.006, 13):
                span = math.sqrt(max(0.0, 0.006**2 - dx**2))
                for dy in (-span, 0.0, span):
                    d = origin + dx * tx + dy * ty - self.p
                    da = d @ self.a
                    na = normal @ self.a
                    b = d @ normal - na * da
                    k = 1 - na * na
                    dis = b * b - k * (d @ d - da * da - feed.R**2)
                    if dis < 0:
                        continue
                    t = (-b - math.sqrt(dis)) / k
                    if -0.0003 <= t <= h and abs(da + t * na) <= feed.HALF:
                        good = True
                        break
                if good:
                    break
            hits.append(good)
        return hits


def replay(job):
    (
        name,
        hybrid,
        k,
        d,
        dt,
        cutoff,
        stations,
        zeta,
        extra,
        dashpot,
        damper_y,
        gap,
        installed,
        deadline,
        pose,
        jdk,
        cls,
        cache,
    ) = job
    delta, shift = (pose, 0.0) if pose is not None else source(name)
    controller = replace(GUIDE, sectors=128, radial_bins=4)
    pipe = CoupledPipe(
        delta,
        shift,
        controller,
        Shelf(k, d, stations=stations),
        hybrid,
        cutoff,
        zeta,
        extra,
        dashpot,
        damper_y,
        gap,
        installed,
    )
    old, old_cfg = feed.Pipe, feed.cfg
    feed.Pipe = lambda *_: pipe
    feed.cfg = replace(feed.cfg, slices=10)
    label = f"{name}-{'insert' if hybrid else 'stock'}-{k:g}-{d:g}-{dt * 1e6:g}-{cutoff}-{stations}-{zeta:g}"
    if extra:
        label += f"-seat-{extra:g}"
    if dashpot:
        label += f"-oil-{dashpot:g}"
    if gap or damper_y != (-0.066, 0.066):
        label += f"-y{damper_y[0] * 1000:g}_{damper_y[1] * 1000:g}-gap{gap * 1000:g}"
    if installed:
        label += "-machined"
    if deadline < 5460:
        label += f"-through-{deadline}"
    try:
        with subprocess.Popen(
            [
                str(jdk / "bin/java"),
                "-cp",
                str(cls),
                "org.reefscape.controls.PortConsole",
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1,
        ) as proc:
            result = feed.trial(
                proc,
                delta,
                label,
                dt,
                0,
                cache,
                feed_side=shift,
                relief=True,
                until_ms=deadline,
            )
            proc.stdin.close()
            proc.wait()
    finally:
        feed.Pipe, feed.cfg = old, old_cfg
    changes = []
    for tick, state, entry, seat, speed in result["portTicks"]:
        data = [state, entry, seat, speed]
        if not changes or changes[-1][1:] != data:
            changes.append([tick, *data])
    wrench = cache / (label + "-dampers.csv")
    if pipe.damper_history:
        np.savetxt(
            wrench,
            np.asarray(pipe.damper_history),
            delimiter=",",
            fmt="%.11g",
            header="from_lip_s,right_down_n,left_down_n,right_displacement_m,left_displacement_m,right_speed_mps,left_speed_mps",
            comments="",
        )
    mount_load = cache / (label + "-support.csv")
    if pipe.force_history:
        np.savetxt(
            mount_load,
            np.asarray(pipe.force_history),
            delimiter=",",
            fmt="%.11g",
            header="from_lip_s,fx_n,fy_n,fz_n,mx_about_both_feet_nm,my_about_both_feet_nm,right_damper_down_n,left_damper_down_n",
            comments="",
        )
    row = {
        "id": label,
        "case": name,
        "hardware": "insert" if hybrid else "stock",
        "localStiffnessNPerM": k,
        "localDissipationSPerM": d,
        "stepS": dt,
        "modeCutoffHz": cutoff,
        "modes": len(pipe.z),
        "lineStations": stations,
        "modalDampingRatio": zeta,
        "additionalSeatStiffnessNPerM": abs(extra),
        "extraSeatCount": 2 if extra > 0 else 4 if extra < 0 else 0,
        "stowDamperSlopeNsPerM": dashpot,
        "damperDemandJ": pipe.plunger_energy.tolist(),
        "damperYmm": [v * 1000 for v in damper_y],
        "runningGapMm": gap * 1000,
        "damperHistorySha256": hashlib.sha256(wrench.read_bytes()).hexdigest()
        if pipe.damper_history
        else None,
        "supportHistorySha256": hashlib.sha256(mount_load.read_bytes()).hexdigest()
        if pipe.force_history
        else None,
        "damperStrokeMm": (1000 * pipe.plunger_peak).tolist(),
        "damperInputMps": pipe.plunger_speed.tolist(),
        "tailWitnessMaximumInsideTubeWallMm": (1000 * pipe.pad_overlap).tolist(),
        "flushWitnessMaximumInsideTubeWallMm": (1000 * pipe.flush_overlap).tolist(),
        "machinedTray": installed,
        "peakLocalCompressionMm": 1000 * pipe.peak_shelf_compression,
        "peakSurfaceNormalN": pipe.peak_shelf_n,
        "peakBendingAtLoadedStationsMm": pipe.max_deformed_mm,
        "probePeakDownMm": pipe.max_probe.tolist(),
        "probePeakUpMm": (-pipe.min_probe).tolist(),
        "peakLoadedSlope": pipe.max_contact_slope,
        "hardwareStop": pipe.hard,
        "loss": pipe.loss,
        "peakCartridgeMm": float(pipe.peak),
        "lastState": result["lastPortState"],
        "fault": result["reason"],
        "firstBandJavaMs": result["firstPortBandMs"],
        "lastJavaMs": result["portTicks"][-1][0],
        "portTransitions": changes,
        "sourceRobotMmDeg": delta,
        "trajectorySha256": result["trajectorySHA256"],
    }
    print(
        label,
        row["lastState"],
        row["firstBandJavaMs"],
        round(pipe.peak_shelf_n),
        row["lastJavaMs"],
        flush=True,
    )
    return row


def run(jdk, cache, jobs, workers, poses=None):
    cache.mkdir(parents=True, exist_ok=True)
    reg = json.loads((ROOT / "station-landmark/feed.json").read_bytes())
    with tempfile.TemporaryDirectory() as folder:
        cls = Path(folder)
        code = cls / "PortAcquisition.java"
        code.write_text(feed.port_program(True), encoding="utf8", newline="\n")
        if hashlib.sha256(code.read_bytes()).hexdigest() != reg["programSha256"]:
            raise ValueError("Port controller changed")
        subprocess.run(
            [
                str(jdk / "bin/javac"),
                "-d",
                str(cls),
                str(code),
                str(ROOT / "observable-acquisition/PortConsole.java"),
            ],
            check=True,
        )
        with ProcessPoolExecutor(max_workers=workers) as pool:
            tasks = []
            for job in jobs:
                if len(job) == 10:
                    job = (*job, (-0.066, 0.066), 0.0)
                if len(job) == 12:
                    job = (*job, False)
                if len(job) == 13:
                    job = (*job, 5460)
                tasks.append((*job, (poses or {}).get(job[0]), jdk, cls, cache))
            return list(pool.map(replay, tasks))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("jdk", type=Path)
    p.add_argument("cache", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("--workers", type=int, default=3)
    p.add_argument(
        "--group",
        choices=[
            "main",
            "refine",
            "controls",
            "damping",
            "seats",
            "anvil",
            "oil",
            "oil-fine",
            "installed",
            "machined",
            "mounts",
        ],
        default="main",
    )
    p.add_argument(
        "--poses",
        type=Path,
        help="JSON object mapping both case names to pipe-input [dx,dy,yaw] in mm,mm,degrees",
    )
    a = p.parse_args()
    cases = ("leftSmall", "rightSmall")
    jobs = [
        (name, hybrid, 400000, 4, 0.0000625, 1500, 32, 0.02)
        for name in cases
        for hybrid in (False, True)
    ]
    if a.group == "refine":
        jobs = [
            (name, True, 400000, 4, dt, 2300, 48, 0.02)
            for name in cases
            for dt in (0.00003125, 0.000015625)
        ]
    if a.group == "controls":
        jobs = [
            (name, True, k, 4, 0.0000625, cutoff, 32, 0.02)
            for name in cases
            for k, cutoff in ((400000, 0), (40000, 1500), (80000, 1500))
        ]
    if a.group == "damping":
        jobs = [
            (name, True, 400000, d, 0.0000625, 1500, 32, zeta)
            for name in cases
            for d, zeta in ((4, 0.08), (4, 0.15), (1, 0.02))
        ]
    jobs = [(*j, 0.0) for j in jobs]
    if a.group == "seats":
        jobs = [
            (name, True, 400000, 4, 0.0000625, 1800, 32, 0.02, k)
            for name in cases
            for k in (1e6, 10e6, 100e6)
        ]
    jobs = [(*j, 0.0) for j in jobs]
    if a.group == "oil":
        jobs = [
            (name, True, 400000, 4, 0.0000625, 1800, 32, 0.02, 0.0, c)
            for name in cases
            for c in (175.0, 500.0, 1500.0)
        ]
    if a.group == "anvil":
        jobs = [
            (name, True, 400000, 4, 0.0000625, 1800, 32, 0.02, -k, 0.0)
            for name in cases
            for k in (1e6, 10e6, 100e6)
        ]
    if a.group == "oil-fine":
        jobs = [
            (name, True, 400000, 4, dt, 2300, 48, 0.02, 0.0, 1500.0)
            for name in cases
            for dt in (0.00003125, 0.000015625)
        ]
    if a.group == "installed":
        jobs = [
            (
                name,
                True,
                400000,
                4,
                0.0000625,
                1800,
                32,
                0.02,
                0.0,
                1500.0,
                (-0.066, 0.042),
                gap,
            )
            for name in cases
            for gap in (0.0, 0.0001, 0.0002)
        ]
    if a.group == "machined":
        jobs = [
            (
                name,
                True,
                400000,
                4,
                dt,
                2300,
                48,
                0.02,
                0.0,
                1500.0,
                (-0.066, 0.042),
                0.0001,
                True,
            )
            for name in cases
            for dt in (0.0000625, 0.00003125)
        ]
    if a.group == "mounts":
        jobs = [
            (
                name,
                True,
                400000,
                4,
                0.00003125,
                2300,
                48,
                0.02,
                0.0,
                1500.0,
                (-0.066, 0.042),
                0.0001,
                True,
                500,
            )
            for name in cases
        ]
    poses = json.loads(a.poses.read_bytes()) if a.poses else None
    if poses is not None and (
        set(poses) != set(cases)
        or any(len(v) != 3 or not np.isfinite(v).all() for v in poses.values())
    ):
        raise ValueError("Both signed, finite pipe input poses are required")
    results = run(a.jdk, a.cache, jobs, a.workers, poses)
    a.out.write_text(
        json.dumps(results, indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
