"""Compose two distinct fitted backings with the registered #118 station replay."""

import tempfile
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

import project_inputs

ROOT = project_inputs.project
sys.path.insert(0, str(ROOT / "guide-impact"))
import beam as prior
from arm_contact import ContactResponse
import importlib.util

_spec = importlib.util.spec_from_file_location(
    "registered_station_replay", ROOT / "guide-impact/replay.py"
)
replay = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(replay)
import screen


def read(p):
    return json.loads(p.read_bytes())


class Pair:
    def __init__(self, kit, layout, cutoff, arm_fraction, configuration):
        reg = read(configuration / "revision.json")
        if (
            prior.digest(configuration / "revision.json")
            != project_inputs.registry["selectedExchangeRecord"]
            or prior.digest(configuration / "route.step")
            != project_inputs.routes["front-return"]
            or reg["stepSha256"] != project_inputs.routes["front-return"]
            or reg["layout"] != "front-return"
        ):
            raise ValueError("Different optical route")
        settings = read(ROOT / "guide-setting/fit.json")
        if (
            prior.digest(ROOT / "guide-setting/setting.step") != reg["settingSha256"]
            or settings["stepSha256"] != reg["settingSha256"]
        ):
            raise ValueError("Different installed guide datum")
        old_fit = read(kit / "fit.json")
        if old_fit["stepSha256"] != reg["guideSha256"]:
            raise ValueError("Different guide")
        full = prior.Beam(kit, "folded", 1.0e8, arm_fraction=arm_fraction)
        if len(full.w) != len(full.nodes) * 2:
            raise ValueError("Unresolved parent basis")
        self.layout = layout
        self.source = {
            "step": reg["stepSha256"],
            "rootKit": reg["guideSha256"],
            "setting": reg["settingSha256"],
        }
        self.sides = []
        self.fits = []
        mounted = np.array([0.687, 0.787])
        at = full.at(mounted)[0]
        c = screen.segments("front-return")["compliance"]
        for side in ("R", "L"):
            model = prior.Beam(kit, "folded", cutoff, arm_fraction=arm_fraction)
            # Project installed attachment stiffness and inertial changes into the
            # parent's complete 50-mode mass-normalized FE basis before truncation.
            k = np.diag(full.w**2)
            m = np.eye(len(k))
            for j, z in enumerate((687, 787)):
                key = f"guide-{side}-{z}-mount-arm"
                volume = settings["volumesMm3"][key]
                stiff = model.attachment[j][2]
                if side == "L" and z == 687:
                    volume = reg["volumesMm3"][key]
                    stiff = 1000 / c
                dm = (volume - old_fit["volumesMm3"][key]) * 0.00000270 * arm_fraction
                v = np.outer(at[j], at[j])
                m += dm * v
                k += (stiff - model.attachment[j][2]) * v
                model.attachment[j][1] += dm
                model.attachment[j][2] = stiff
            chol = np.linalg.cholesky(m)
            left = np.linalg.solve(chol, k)
            sym = np.linalg.solve(chol, left.T).T
            ev, vec = np.linalg.eigh((sym + sym.T) / 2)
            if ev.min() <= 0:
                raise ValueError("Nonpositive mode")
            keep = ev < (2 * np.pi * cutoff) ** 2
            transform = np.linalg.solve(chol.T, vec[:, keep])
            model.w = ev[keep] ** 0.5
            model.phi = full.phi @ transform
            model.slope_phi = full.slope_phi @ transform
            model.cache.clear()
            self.sides.append(model)
            checks = {}
            for z in (625, 637.5, 687, 737, 787, 813):
                shape = full.at(np.array([z * 0.001]))[0][0]
                expected = float(screen.ref.travel(np.array([z]), 1.0)[0])
                if side == "L":
                    expected += ((787 - z) / 100) ** 2 * (
                        c - screen.ref.arm(687)["totalMmPerN"]
                    )
                solved = 1000 * float(shape @ np.linalg.solve(k, shape))
                retained = 1000 * float(np.sum((shape @ transform / model.w) ** 2))
                if abs(solved - expected) > 0.00005:
                    raise ValueError((z, solved, expected))
                checks[str(z)] = [expected, solved, retained]
            self.fits.append(
                dict(
                    side=side,
                    attachment=model.attachment,
                    modesHz=(model.w / (2 * np.pi)).tolist(),
                    analyticFullRetainedMmPerN=checks,
                )
            )
        size = max(len(s.w) for s in self.sides)

        def pad(a, fill=0.0):
            return np.pad(
                a,
                [(0, 0)] * (a.ndim - 1) + [(0, size - a.shape[-1])],
                constant_values=fill,
            )

        self.w = np.stack([pad(s.w, 1.0) for s in self.sides])
        self.phi = np.stack([pad(s.phi) for s in self.sides])
        self.slope_phi = np.stack([pad(s.slope_phi) for s in self.sides])
        self.grid = full.grid
        self.damping = full.damping
        self.cache = {}
        self.cutoff = cutoff
        self.moving = arm_fraction

    def at(self, z, j):
        s = np.clip((z - 0.625) * 4000, 0, len(self.grid) - 1.00000001)
        i = s.astype(int)
        t = (s - i)[:, None]
        return tuple(
            a[j, i] * (1 - t) + a[j, i + 1] * t for a in (self.phi, self.slope_phi)
        )

    advance = prior.Beam.advance

    def receipt(self):
        return dict(
            source=self.source,
            layout=self.layout,
            cutoffHz=self.cutoff,
            movingArmMassFraction=self.moving,
            sides=self.fits,
        )


def run(args):
    reg = read(ROOT / "guide-impact/results.json")
    orig = next(
        r
        for r in reg["cases"]
        if r["case"] == args.case
        and r["stepUs"] == args.step
        and r["id"].endswith("serial-carriage-recover")
        and r["layout"] == "folded"
    )
    replay.Beam = lambda *a, **kw: Pair(*a, **kw, configuration=args.configuration)

    class RegisteredResponse(ContactResponse):
        def __init__(self, *args):
            super().__init__(*args)
            self.tracked = []

        def step(self, pipe, dt):
            prior_count = len(self.trace)
            super().step(pipe, dt)
            if len(self.trace) != prior_count:
                self.tracked.append((self.last, *self.q.ravel(), *self.v.ravel()))

        def finish(self, time):
            result = super().finish(time)
            np.save(folder / "guide-modes.npy", np.asarray(self.tracked))
            result["guideModesSha256"] = prior.digest(folder / "guide-modes.npy")
            return result

    replay.GuideResponse = RegisteredResponse

    spec = importlib.util.spec_from_file_location(
        "recovery_source", ROOT / "split-guide-acquisition/split_controller.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with tempfile.TemporaryDirectory() as tmp:
        classes = Path(tmp)
        path = classes / "PortAcquisition.java"
        path.write_text(module.candidate(replay.feed), encoding="utf8", newline="\n")
        if prior.digest(path) != orig["programSha256"]:
            raise ValueError("Different Java")
        subprocess.run(
            [
                str(args.jdk / "bin/javac"),
                "-d",
                str(classes),
                str(path),
                str(ROOT / "observable-acquisition/PortConsole.java"),
            ],
            check=True,
        )
        job = (
            args.case,
            orig["poseMmDeg"],
            "front-return",
            args.step,
            ROOT / "guide-mounts",
            args.jdk,
            classes,
            args.output,
            5460,
            1800.0,
            0.236,
            ROOT / "compliant-carriage",
            0.0,
            False,
            True,
            orig["programSha256"],
        )
        folder = (
            args.output
            / f"{args.case}-front-return-{args.step:g}us-1800Hz-0.236-5460-serial-carriage-recover"
        )
        replay.trial(job)
