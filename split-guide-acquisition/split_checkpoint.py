"""Snapshot all generalized states and the original port clock at camera exposures and departure."""

from pathlib import Path
import sys, hashlib
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "compliant-carriage"))
from model import FlexiblePipe, coupled

EXPOSURES = (5372, 5389, 5406, 5423, 5440, 5457, 5460)
PIVOT_EPOCH = 21.571672635314284


def vec(a):
    return np.asarray(a).tolist()


def ah(a):
    return hashlib.sha256(np.asarray(a, dtype="<f8").tobytes()).hexdigest()


class Station(FlexiblePipe):
    settings = (1.0, 5e6, 1.0, 0.002, 0.0001, 1.0, 2)

    def __init__(self, *args):
        super().__init__(*args)
        self.exposure = {}
        self.pending = iter(EXPOSURES)
        self.next_exposure = next(self.pending, None)

    def moves(self, target, dt):
        super().moves(target, dt)
        if self.next_exposure is None:
            return
        java_ms = self.time * 1000 + 100
        if java_ms + 1e-6 >= self.next_exposure:
            if abs(java_ms - self.next_exposure) > 1e-5:
                raise ValueError("Missing exact exposure")
            self.exposure[str(self.next_exposure)] = self.capture(target)
            self.next_exposure = next(self.pending, None)

    def capture(self, target):
        m = self.model
        z = self.z
        dz = self.zdot
        grips = []
        for i, side in enumerate((-1, 1)):
            pts = self.footprint(side)
            area = sum(a for _, a in pts)
            loads = []
            for arm, cell in pts:
                travel = 1000 * (side * (self.p[1] + arm[1]) - coupled.feed.HALF)
                if travel < -self.inward:
                    continue
                vel = self.v + np.cross(self.angular(self.l), arm)
                n = (
                    max(0, self.preload + 2 * travel + 37.52 * side * vel[1])
                    * cell
                    / max(coupled.feed.cfg.area_for_preload, area)
                )
                loads.append((travel, n))
            grips.append(
                {
                    "side": side,
                    "contactCells": len(loads),
                    "normalN": sum(n for _, n in loads),
                    "compressionRangeMm": [
                        min((t for t, _ in loads), default=None),
                        max((t for t, _ in loads), default=None),
                    ],
                }
            )
        return {
            "javaMs": round(self.time * 1000 + 100),
            "wallS": PIVOT_EPOCH + self.time + 0.100,
            "secondsFromLip": self.time,
            "p": vec(self.p),
            "a": vec(self.a),
            "v": vec(self.v),
            "l": vec(self.l),
            "omega": vec(self.angular(self.l)),
            "z": vec(z),
            "zdot": vec(dz),
            "landsM": vec(m["site"] @ z),
            "landSpeedMps": vec(m["site"] @ dz),
            "guideStagesM": vec(m["relative"] @ z),
            "guideStageSpeedMps": vec(m["relative"] @ dz),
            "damperBaseM": vec(m["damperBase"] @ z),
            "damperBaseSpeedMps": vec(m["damperBase"] @ dz),
            "plungerM": vec(self.plunger),
            "lastDamperForceN": vec(self.applied_dampers),
            "nextBeltMps": target,
            "actualBeltMps": self.belt,
            "apertures": list(map(bool, self.ports())),
            "grips": grips,
            "touch": self.touch,
            "firstBeltsS": self.first_belts,
            "hard": self.hard,
            "loss": self.loss,
        }


class ClosedLoopDisturbance(Station):
    """Introduce a bench fault at the first positive challenge following the held brake."""

    disturbance = ""

    def __init__(self, *args):
        super().__init__(*args)
        self.applied_fault = False

    def moves(self, target, dt):
        if self.time >= 0.48 - 1e-8 and target > 0 and not self.applied_fault:
            self.applied_fault = True
            if self.disturbance == "lost-traction":
                from dataclasses import replace

                coupled.feed.cfg = replace(coupled.feed.cfg, belt_mu=0.0)
        if self.applied_fault and self.disturbance == "locked-belts":
            # The actuator can no longer turn the end belts. Continue integrating the
            # pipe, both guide stages, gripper normal force and plunger return.
            super().moves(0.0, dt)
        else:
            super().moves(target, dt)
