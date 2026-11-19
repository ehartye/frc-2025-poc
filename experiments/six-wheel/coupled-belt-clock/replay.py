"""Drive the pinned annular-contact integrator from desktop held speeds."""
import argparse
import csv
import hashlib
import json
import math
import sys
from bisect import bisect_right
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "axial-spin"))
import pipe_spin_route as route
from pipe_axial_contact import AxialContact, M, clip
from spin_cases import CANDIDATE


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Trace:
    def __init__(self, profile):
        with (HERE / "controller.csv").open(encoding="ascii", newline="") as file:
            rows = [r for r in csv.DictReader(file) if r["profile"] == profile]
        self.rows = rows
        self.ticks = [int(r["tick_ms"]) for r in rows]
        if self.ticks != list(range(20, 901, 20)):
            raise ValueError("missing or duplicate scheduler tick")
        self.values = [float(r["issued_mps"]) for r in rows]
        self.intake_start = next(int(r["tick_ms"]) for r in rows if r["state"] == "WAIT_ENTRY")
        self.release_start = next(int(r["tick_ms"]) for r in rows if r["state"] == "RELEASE_SEAT")
        self.confirm = next(int(r["tick_ms"]) for r in rows if r["state"] == "CONFIRM")
        if any(not math.isfinite(v) for v in self.values):
            raise ValueError("invalid belt command")

    def held(self, elapsed, acquisition=False):
        origin = self.intake_start if acquisition else self.release_start
        # Align an integration time within floating-point round-off to the exact scheduler tick.
        time_ms = origin + elapsed * 1000 + 1e-7
        if acquisition:
            # Remain in the observed zero-speed custody demand after the scripted seat edge.
            time_ms = min(time_ms, self.confirm)
        index = bisect_right(self.ticks, time_ms) - 1
        if index < 0 or time_ms > self.ticks[-1] + 1e-5:
            raise ValueError("held command outside recorded domain")
        return self.values[index]


class HeldContact(AxialContact):
    def __init__(self, cfg, spin, trace, acquisition):
        super().__init__(cfg, spin)
        self.trace = trace
        self.acquisition = acquisition
        self.target = None

    def normals(self, u, v, yaw, phase, time, zrel=0):
        left, right, friction, limit, floor = super().normals(u, v, yaw, phase, time, zrel)
        self.target = None
        if phase == "place":
            self.target = self.trace.held(time - self.start)
        elif self.acquisition and phase == "seat":
            self.target = self.trace.held(time, acquisition=True)
        return left, right, friction, limit if self.target is None else abs(self.target), floor

    def forces(self, speed, vertical, normals, requests, limit, dt, required_normal):
        if self.target is None:
            return super().forces(speed, vertical, normals, requests, limit, dt, required_normal)
        # Invert #43's velocity-command input (desired = speed + 2*request/(M*20)).
        # Both sides receive the same direct setpoint independently of simulated pipe position.
        request = (self.target - speed) * M * 10
        prev = self.belt_speed.copy()
        result = super().forces(speed, vertical, normals, (request, request), abs(self.target), dt, required_normal)
        for side, (patches, normal) in enumerate(zip(self.points, normals)):
            if not patches or not normal:
                # Powered belts approach the setpoint even in air; zero pipe force on unloaded side.
                increment = clip(self.target - prev[side], self.cfg.belt_accel * dt)
                self.belt_speed[side] = prev[side] + increment
                self.max_belt_accel = max(self.max_belt_accel, abs(increment)/dt)
                self.max_belt_speed = max(self.max_belt_speed, abs(self.belt_speed[side]))
        return result


def run_case(job):
    profile, piece, step = job
    trace = Trace(profile)
    make = lambda cfg, spin: HeldContact(cfg, spin, trace, acquisition=False)
    with patch.object(route, "AxialContact", make):
        row, through, _ = route.trial(CANDIDATE, piece, step)
    row.pop("release_trace", None)
    if through["outcome"] == "paired-trough-first-contact":
        fixed = route.settle.continue_contact(
            sector=route.SECTORS[piece["id"]], upstream=through, piece_id=piece["id"],
            withdrawal="fixed", initial_roll=row["release_spin_rad_s"], dt=.0002, inertia_ratio=route.K)
        row["desktop_fixed_slide"] = {"status": fixed["stop"], "minimum_gap_mm": fixed["min_shelf_clearance_mm"],
                                      "elapsed_from_target_contact_s": fixed["elapsed_s"]}
    row["events_from_station_s"] = {e["event"]: e["time_s"] for e in through["events"]}
    return profile, piece["id"], str(step), row


def study():
    pickup = {}
    for step in (.0005, .00025):
        trace = Trace("RAMP_090")
        make = lambda cfg, spin: HeldContact(cfg, spin, trace, acquisition=True)
        with patch.object(route, "AxialContact", make):
            result, _, path = route.acquire(CANDIDATE, step)
        positions = [point[0] for point in path]
        impossible = []
        for row in trace.rows:
            if row["state"] not in ("WAIT_ENTRY", "WAIT_SEAT", "CONFIRM"):
                continue
            elapsed = (int(row["tick_ms"]) - trace.intake_start) / 1000
            index = max(0, bisect_right(positions, elapsed) - 1)
            sampled_t, center, _, _ = path[index]
            for port, u in (("entry", .100), ("seat", .040)):
                if row[port] == "true" and abs(u - center) > route.R:
                    impossible.append({"state": row["state"], "scripted_ms_after_lip": round(elapsed * 1000),
                                       "sample_from_lip_s": sampled_t, "port": port, "center_m": center,
                                       "projected_gap_mm": (abs(u-center) - route.R) * 1000})
        result["impossible_scripted_blockages"] = impossible
        pickup[str(step)] = result
    jobs = [(profile, piece, step) for profile in ("HISTORICAL", "RAMP_090")
            for piece in route.DATA["identified_transfers"] for step in (.0005, .00025)]
    cases = {}
    with ProcessPoolExecutor(max_workers=4) as workers:
        for profile, identity, step, row in workers.map(run_case, jobs):
            cases.setdefault(profile, {}).setdefault(identity, {})[step] = row
    sources = [HERE / n for n in ("DesktopReleaseIO.java", "DesktopClockTest.java", "init.gradle",
                                  "controller.csv", "capture.py", "replay.py")]
    sources += sorted((ROOT / "competition-controls/src/main").rglob("*.java"))
    sources += [ROOT / n for n in ("axial-spin/pipe_axial_contact.py", "axial-spin/pipe_spin_route.py",
                                   "axial-spin/spin_cases.py", "axial-spin/study.json",
                                   "scripts/station_l1_mission.py", "scripts/station_l1_settle.py",
                                   "shared/examples/java-2026-sim/gradle.lockfile",
                                   "sourced-match/source-545.json")]
    return {"schema": "station-held-belt-clock/1",
            "sourceSha256": {p.relative_to(ROOT).as_posix(): digest(p) for p in sources},
            "approach": "Conditional release from #43 modeled seating and carry. Scripted IO edges.",
            "acquisition": pickup, "releases": cases, "officialScore": None, "measuredHardware": False}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=HERE / "study.json")
    args = parser.parse_args()
    args.output.write_text(json.dumps(study(), indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
