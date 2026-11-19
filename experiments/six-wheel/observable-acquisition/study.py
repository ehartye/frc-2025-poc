"""Close the centered receiver contact plant around the Java controller's two optical returns."""
import argparse
import csv
import hashlib
import json
import math
import os
from dataclasses import asdict, dataclass, replace
from pathlib import Path
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "coupled-belt-clock"))
from replay import HeldContact, CANDIDATE, route
from pipe_axial_contact import AxialContact, G, I, M, R, clip, ramp


@dataclass(frozen=True)
class Case:
    name: str = "nominal"
    phase_ms: int = 0
    depth_mm: float = 0
    heights_mm: tuple = (4.0, 17.5)
    port_offset_mm: tuple = (0, 0)
    grip_mu: float = .4
    preload: float = 10
    ramp_mu: float = .4
    fault: str = ""
    counterfeed: bool = True


def port_returns(u, z, case):
    # Vertical rays anywhere across each 12 mm bore, directed through its centerline in Y.
    # The closest surface is the opaque, transverse pipe sidewall at Y=0.
    reach = []
    for port, offset, height in zip((100, 40), case.port_offset_mm, case.heights_mm):
        distance = max(0.0, abs(u * 1000 - port - offset) - 6)
        radius = R * 1000
        if distance > radius:
            reach.append(False)
            continue
        gap = 1000 * (z - route.station.SHELF_TOP) - math.sqrt(radius**2 - distance**2)
        reach.append(gap <= height)
    return tuple(reach)


def max_return_radius(height_mm):
    radius = R * 1000
    return 6 + math.sqrt(2 * radius * height_mm - height_mm**2)


def in_band_bound():
    # Upper activation ranges include the entire opening and an assumed ±0.25 mm port registration.
    return [100 - .25 - max_return_radius(4.2), 40 + .25 + max_return_radius(17.8)]


class DirectBelt(HeldContact):
    def __init__(self, cfg, spin):
        super().__init__(cfg, spin, trace=None, acquisition=False)
        self.target = 0.0


def run(driver, case, dt, events):
    driver.stdin.write("COUNTERFEED\n" if case.counterfeed else "STATIONARY\n")
    driver.stdin.flush()
    cfg = replace(CANDIDATE, belt_mu=case.grip_mu, preload=case.preload, ramp_mu=case.ramp_mu)
    initial = ramp(route.DATA["feed"], cfg.ramp_mu)
    lip = route.station.departure(initial["speed"])
    if lip["mode"] != "immediate-separation":
        raise ValueError("unmodeled edge contact")
    x, z = route.station.flight_center(lip, 0)
    u = route.U_ORIGIN + case.depth_mm / 1000 - x
    speed, vertical = -lip["velocity_x_mps"], lip["velocity_z_mps"]
    contact = DirectBelt(cfg, initial["spin"])
    supported = False
    impact = None
    escaped = None
    lip_ms = 100 + case.phase_ms
    first_qualified = None
    first_fault = None
    lowest = u
    time = 0.0
    measurement = {}
    for ms in range(0, 2701, 20):
        now = ms / 1000
        entry, seat = (False, False) if ms < lip_ms or escaped else port_returns(u, z, case)
        t = ms - lip_ms
        if case.fault == "aliased-pair-stuck" and t >= 40: entry = seat = True
        if case.fault == "entry-stuck-low": entry = False
        if case.fault == "entry-high-at-wake": entry = True
        if case.fault == "seat-stuck-high" and t >= 140: seat = True
        if case.fault == "drop-seat-in-dwell" and measurement.get("state") == "DWELL": seat = False
        at = ms - 20 if case.fault == "stale" and t >= 160 else ms
        valid = case.fault != "missing-frame" or t < 180
        calibrated = case.fault != "uncalibrated"
        docked = case.fault != "out-of-plane" and (case.fault != "lost-dock" or t < 200)
        enabled = case.fault != "disabled" or t < 220
        sensor_row = [ms, at, int(valid), int(calibrated), int(docked), int(enabled), int(entry), int(seat)]
        driver.stdin.write(",".join(map(str, sensor_row)) + "\n")
        driver.stdin.flush()
        state, active, value, reason = driver.stdout.readline().rstrip("\n").split(",")
        demand = float(value)
        events.append([case.name, int(round(dt * 1e6)), *sensor_row, state, active, value, reason])
        measurement = {"state": state, "from_lip_ms": t, "reason": reason, "active": active == "true",
                       "u_mm": u * 1000, "speed_mps": speed, "spin_rad_s": contact.spin}
        if state == "FAULT":
            first_fault = measurement
            break
        if state == "IN_BAND":
            if first_qualified is None:
                first_qualified = dict(measurement)
            if t >= first_qualified["from_lip_ms"] + 240: break
        if escaped:
            continue
        end = now + .020
        if time < lip_ms / 1000:
            unloaded = min(end, lip_ms / 1000) - time
            contact.belt_speed = [b + clip(demand-b, cfg.belt_accel*unloaded) for b in contact.belt_speed]
            time += unloaded
        if end <= lip_ms / 1000 + 1e-12:
            time = end
            continue
        while end - time > 1e-11:
            step = min(dt, end - time)
            if not supported and vertical < 0:
                step = min(step, max(1e-10, (z - route.SHELF) / -vertical))
            # Keep #43 finite patch forces and the exact #47 held-speed actuator.
            norms = AxialContact.normals(contact, u, 0, 0, "seat", time, z - route.SHELF)
            contact.target = demand
            forces, floor, up = contact.forces(speed, vertical, norms[:2], (0, 0), abs(demand), step,
                                               M*G if supported else None)
            acc = (sum(forces) + floor) / M
            az = 0.0 if supported else up / M - G
            u += speed * step + .5 * acc * step**2
            speed += acc * step
            z += vertical * step + .5 * az * step**2
            vertical += az * step
            time += step
            lowest = min(u, lowest)
            if not supported and z <= route.SHELF + 1e-10:
                if not -.100 <= u <= .159:
                    return {"outcome": "touchdown-outside-shelf", "u_mm": u*1000}
                impulse = clip(-(speed + contact.spin * R)/(1/M + R*R/I), cfg.shelf_mu*M*max(0, -vertical))
                speed += impulse/M
                contact.spin += R*impulse/I
                supported = True
                z, vertical = route.SHELF, 0.
                impact = (time*1000-lip_ms, u*1000)
            if supported and (not -.100 <= u <= .165 or contact.lifted):
                escaped = {"u_mm": u*1000, "from_lip_ms": time*1000-lip_ms, "last_command_state": state}
                break
    outcome = "physical-custody-lost" if escaped else ("neutral-fault" if first_fault else measurement["state"])
    return {"outcome": outcome, "physical_loss": escaped,
            "first_in_band": first_qualified, "last": measurement, "first_contact_ms_mm": impact,
            "minimum_u_mm": lowest*1000, "minimum_shelf_N": contact.min_shelf_normal}


def cases():
    base = Case()
    result = [base] + [replace(base, name=f"stationary-{d}", depth_mm=d, counterfeed=False) for d in (-5,0,5)]
    result.extend(replace(base, name=f"phase-{p}", phase_ms=p) for p in range(1, 20))
    for depth in (-5, 5):
        for phase in range(20):
            result.append(replace(base, name=f"depth-{depth}-phase-{phase}", depth_mm=depth, phase_ms=phase))
    for thresholds in ((3.7,17.2), (4.2,17.8)):
        for offsets in ((-.25,+.25), (+.25,-.25)):
            name = "optics-" + "-".join(map(str, (*thresholds, *offsets)))
            result.append(replace(base, name=name, heights_mm=thresholds, port_offset_mm=offsets))
    result.extend([replace(base, name="low-grip", preload=2, grip_mu=.08),
                   replace(base, name="sliding-ramp", ramp_mu=.15)])
    result.extend(replace(base, name=fault, fault=fault) for fault in ("aliased-pair-stuck", "entry-stuck-low", "entry-high-at-wake",
                  "seat-stuck-high", "drop-seat-in-dwell", "stale", "missing-frame", "uncalibrated", "lost-dock", "out-of-plane", "disabled"))
    return result


def indistinguishable_empty_receiver(driver, nominal):
    # Even perfectly ordered false electronic returns are observationally identical.
    driver.stdin.write("COUNTERFEED\n")
    verified = 0
    terminal = ""
    for row in nominal:
        inputs = row[2:10]
        driver.stdin.write(",".join(map(str, inputs)) + "\n")
        driver.stdin.flush()
        regenerated = driver.stdout.readline().rstrip("\n").split(",")
        if regenerated != row[10:]: raise ValueError("identical observations produced different commands")
        terminal = regenerated[0]
        verified += 1
    return {"physical_truth": "empty receiver; replayed nominal sensor stream", "identical_output_ticks": verified,
            "terminal_state": terminal, "independent_physical_proof": False}


def generate(jdk):
    rows, outcomes = [], {}
    suffix = ".exe" if os.name == "nt" else ""
    with tempfile.TemporaryDirectory() as temp:
        subprocess.run([str(jdk/"bin"/("javac" + suffix)), "-d", temp, str(HERE/"PortConsole.java"),
                        str(HERE/"PortAcquisition.java")], check=True)
        with subprocess.Popen([str(jdk/"bin"/("java" + suffix)), "-cp", temp, "org.reefscape.controls.PortConsole"],
                              stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1) as driver:
            try:
                for case in cases():
                    outcomes[case.name] = {}
                    for step in (.0005, .00025):
                        outcomes[case.name][str(step)] = run(driver, case, step, rows)
                spoof = indistinguishable_empty_receiver(driver, [r for r in rows if r[:2] == ["nominal", 500]])
            finally:
                driver.stdin.close()
            if driver.wait() != 0: raise RuntimeError("Java controller exited unsuccessfully")
    with (HERE/"ticks.csv").open("w", newline="", encoding="ascii") as file:
        writer = csv.writer(file, lineterminator="\n")
        writer.writerow(["case", "step_us", "ms", "sample_ms", "valid", "calibrated", "docked", "enabled",
                         "entry", "seat", "state", "active", "belt_mps", "reason"])
        writer.writerows(rows)
    inputs = [HERE/"PortAcquisition.java", HERE/"PortConsole.java", HERE/"study.py", HERE/"ticks.csv",
              ROOT/"coupled-belt-clock/replay.py", ROOT/"axial-spin/pipe_axial_contact.py",
              ROOT/"axial-spin/pipe_spin_route.py", ROOT/"sourced-match/source-545.json",
              HERE/"ports.json", HERE/"native_ports.py", ROOT/"design/station-l1/mechanical.py",
              ROOT/"competition-controls/io-contract.json", ROOT/"design/electrical-inspection/package.py",
              ROOT/"design/electrical-inspection/circuits.json"]
    data = {"schema": "centered-digital-acquisition/1", "source_sha256": {
                 p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
            "two_true_center_bound_mm": in_band_bound(), "cases": outcomes,
            "conditions": {case.name: asdict(case) for case in cases()},
            "empty_receiver_identical_electronics": spoof,
            "calibrated_on_hardware": False, "carry_permitted": False}
    (HERE/"results.json").write_text(json.dumps(data, indent=2, allow_nan=False)+"\n", encoding="utf-8", newline="\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--java-home", required=True, type=Path)
    generate(parser.parse_args().java_home.resolve())
