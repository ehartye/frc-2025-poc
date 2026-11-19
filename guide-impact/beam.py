"""Two independent sloping guide backings on translational arm/root springs."""

import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "station-guide"))
import guide as guide

LAYOUTS = {
    "folded": (4.0, 14.0, True),
    "flat": (0.0, 14.0, True),
    "half-couple": (4.0, 7.0, True),
    "unribbed": (4.0, 14.0, False),
    "rigid": (4.0, 14.0, True),
}


def digest(p):
    with p.open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def support_source(kit):
    src = kit / "loads.py"
    spec = importlib.util.spec_from_file_location("fixed_guide_reference", src)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def bending_inertia(lip):
    width = 40 / math.hypot(1, 0.46)
    thick = 0.8
    web = width * thick
    legs = 2 * lip * thick
    centroid = (web * thick / 2 + legs * (thick + lip / 2)) / (web + legs)
    return (
        width * thick**3 / 12
        + web * (centroid - thick / 2) ** 2
        + 2 * thick * lip**3 / 12
        + legs * (centroid - thick - lip / 2) ** 2
    )


def basis_at(z, nodes, slope, modes):
    i = np.clip(np.searchsorted(nodes, z, side="right") - 1, 0, len(nodes) - 2)
    dz = nodes[i + 1] - nodes[i]
    t = (z - nodes[i]) / dz
    l = dz * slope
    h = np.stack(
        [
            1 - 3 * t * t + 2 * t**3,
            l * t * (1 - t) ** 2,
            3 * t * t - 2 * t**3,
            l * t * t * (t - 1),
        ],
        axis=-1,
    )
    dh = (
        np.stack(
            [
                6 * t * (t - 1),
                l * (1 - t) * (1 - 3 * t),
                6 * t * (1 - t),
                l * t * (3 * t - 2),
            ],
            axis=-1,
        )
        / dz[:, None]
    )
    values = modes[2 * i[:, None] + np.arange(4)]
    return np.einsum("ni,nim->nm", h, values), np.einsum("ni,nim->nm", dh, values)


class Beam:
    def __init__(
        self, kit, layout, cutoff=1800, damping=0.03, arm_fraction=0.236, segments=8
    ):
        fit = json.loads((kit / "fit.json").read_bytes())
        if (
            digest(kit / "guide-mounts.step") != fit["stepSha256"]
            or len(fit["volumesMm3"]) != 58
        ):
            raise ValueError("Unexpected guide assembly")
        self.layout = layout
        self.cutoff = cutoff
        self.damping = damping
        self.arm_fraction = arm_fraction
        ref = support_source(kit)
        lip, pivot, ribs = LAYOUTS[layout]
        self.slope = math.hypot(1, 0.46)
        ea = ref.E * 1e6
        iy = bending_inertia(lip) * 1e-12
        self.nodes = np.concatenate(
            [
                np.linspace(a, b, segments + 1)[:-1]
                for a, b in ((0.625, 0.687), (0.687, 0.787), (0.787, 0.813))
            ]
            + [np.array([0.813])]
        )
        size = 2 * len(self.nodes)
        k = np.zeros((size, size))
        m = np.zeros_like(k)
        volume = fit["volumesMm3"]
        sheet_volume = volume["guide-L-backing"]
        # Retain folded mass on the flat control to isolate its stiffness change.
        self.distributed = (
            volume["guide-L-liner"] * 0.00000050 + sheet_volume * 0.00000270
        )
        density = self.distributed / ((self.nodes[-1] - self.nodes[0]) * self.slope)
        for i, dz in enumerate(np.diff(self.nodes)):
            l = dz * self.slope
            ix = np.s_[2 * i : 2 * i + 4, 2 * i : 2 * i + 4]
            k[ix] += (
                ea
                * iy
                / l**3
                * np.array(
                    [
                        [12, 6 * l, -12, 6 * l],
                        [6 * l, 4 * l * l, -6 * l, 2 * l * l],
                        [-12, -6 * l, 12, -6 * l],
                        [6 * l, 2 * l * l, -6 * l, 4 * l * l],
                    ]
                )
            )
            m[ix] += (
                density
                * l
                / 420
                * np.array(
                    [
                        [156, 22 * l, 54, -13 * l],
                        [22 * l, 4 * l * l, 13 * l, -3 * l * l],
                        [54, 13 * l, 156, -22 * l],
                        [-13 * l, -3 * l * l, -22 * l, 4 * l * l],
                    ]
                )
            )
        self.attachment = []
        for mm in ref.STATIONS:
            root = 2 * int(np.argmin(abs(self.nodes - mm / 1000)))
            prefix = f"guide-L-{int(mm)}-"
            nose = sum(
                volume[prefix + t] * rho
                for t, rho in [
                    ("mount-arm", 0.0000027 * arm_fraction),
                    ("tapped-bar", 0.0000027),
                    ("wedge", 0.0000027),
                    ("face-screw-0", 0.00000785),
                    ("face-screw-1", 0.00000785),
                ]
            )
            stiffness = 1000 / ref.arm(mm, pivot, ribs)["totalMmPerN"]
            self.attachment.append([mm, nose, stiffness])
            m[root, root] += nose
            k[root, root] += stiffness
        transform = np.linalg.cholesky(m)
        left = np.linalg.solve(transform, k)
        sym = np.linalg.solve(transform, left.T).T
        ev, normalized = np.linalg.eigh((sym + sym.T) / 2)
        modes = np.linalg.solve(transform.T, normalized)
        if ev[0] <= 0:
            raise ValueError("Unsupported guide mode")
        self.w = np.sqrt(ev[ev < (2 * np.pi * cutoff) ** 2])
        modes = modes[:, : len(self.w)]
        self.grid = np.linspace(0.625, 0.813, 753)
        self.phi, self.slope_phi = basis_at(self.grid, self.nodes, self.slope, modes)
        self.check = {}
        for mm in (625, 637.5, 687, 737, 787, 813):
            shape = basis_at(
                np.array([mm / 1000]), self.nodes, self.slope, np.eye(size)
            )[0][0]
            reference = float(ref.travel(np.array([mm]), 1.0, lip, pivot, ribs)[0])
            full = 1000 * float(shape @ np.linalg.solve(k, shape))
            res = 1000 * float(np.sum((shape @ modes / self.w) ** 2))
            self.check[str(mm)] = [reference, full, res]
        self.source = {
            "step": fit["stepSha256"],
            "fit": digest(kit / "fit.json"),
            "rootEquations": digest(kit / "loads.py"),
            "staticReceipt": digest(kit / "loads.json"),
        }
        self.cache = {}

    def at(self, z):
        s = np.clip((z - 0.625) * 4000, 0, len(self.grid) - 1.00000001)
        i = s.astype(int)
        t = (s - i)[:, None]
        return self.phi[i] * (1 - t) + self.phi[i + 1] * t, self.slope_phi[i] * (
            1 - t
        ) + self.slope_phi[i + 1] * t

    def advance(self, dt, q, v, f):
        key = round(dt, 12)
        if key not in self.cache:
            w = self.w
            z = self.damping
            wd = w * np.sqrt(1 - z * z)
            e = np.exp(-z * w * dt)
            s = e * np.sin(wd * dt) / wd
            c = e * np.cos(wd * dt)
            self.cache[key] = (c + z * w * s, s, c - z * w * s)
        a, b, c = self.cache[key]
        eq = f / self.w**2
        return a * (q - eq) + b * v + eq, c * v - self.w**2 * b * (q - eq)

    def receipt(self):
        return {
            "source": self.source,
            "layout": self.layout,
            "cutoffHz": self.cutoff,
            "frequenciesHz": (self.w / (2 * np.pi)).tolist(),
            "modalDamping": self.damping,
            "distributedMassPerSideKg": self.distributed,
            "movingArmMassFraction": self.arm_fraction,
            "rootsMmKgNPerM": self.attachment,
            "staticMmPerN_referenceFullRetained": self.check,
        }
