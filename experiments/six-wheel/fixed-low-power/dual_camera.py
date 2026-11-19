"""Screen three independently powered USB loads on the fixed-low match clock."""
import argparse
import csv
import hashlib
import json
import math
from functools import lru_cache
from pathlib import Path

import power

ROOT = power.ROOT
HERE = Path(__file__).resolve().parent
SUPPLIES = {
    # channel, input lead m, output AWG, output lead m, fuse/connectors/USB ohms,
    # connected output current including the host's 1 W auxiliary allowance.
    "Host": (6, .80, 16, .20, .025, 3.20),
    "FixedCamera": (8, .95, 14, 1.30, .055, 1.20),
    "MovingCamera": (9, 1.10, 14, 1.50, .055, 1.20),
}
RESISTANCE = {k: v for k, v in power.electric.RESIST.items() if k != "Observer"}
for name, (_, length, *_) in SUPPLIES.items():
    RESISTANCE[name] = 2 * 1.25 * power.electric.AWG[18] * length + .003


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inlet(name, extra_ohm=0):
    _, _, awg, length, interfaces, amps = SUPPLIES[name]
    resistance = 2 * 1.25 * power.electric.AWG[awg] * length + interfaces + extra_ohm
    # Bench acceptance limits at converter screws include ripple and transients.
    # A further 10 mV bounds instrumentation error at either extreme.
    return {"loopOhm": resistance, "conditionalMinV": 5.13 - .010 - amps * resistance,
            "conditionalMaxV": 5.20 + .010, "loadA": amps}


def solve(voc, common, watts):
    """High-voltage fixed point approached monotonically from open circuit."""
    bus = voc
    for _ in range(150):
        volts = {}
        for key, value in watts.items():
            discriminant = bus**2 - 4 * RESISTANCE[key] * value
            if discriminant <= 0:
                raise ValueError("branch collapse")
            volts[key] = (bus + math.sqrt(discriminant)) / 2
        currents = {key: value / volts[key] for key, value in watts.items()}
        residual = bus + common * sum(currents.values()) - voc
        if abs(residual) < 1e-9:
            return volts, currents
        bus -= residual
        if bus <= 0:
            raise ValueError("main supply collapse")
    raise ValueError("high-voltage root unconfirmed")


def rack_windows():
    optics = read(ROOT / "low-observation/results.json")["pieces"]
    feedback = read(ROOT / "slide-feedback/results.json")["pieces"]
    with (ROOT / "slide-feedback/phases.csv").open() as stream:
        phases = list(csv.DictReader(stream))
    windows = []
    for plan in power.plans:
        name = plan["id"]
        rows = [r for r in phases if r["piece"] == name]
        if {int(r["exposure_phase_ms"]) for r in rows} != set(range(17)) or len(rows) != 17:
            raise ValueError("incomplete exposure phases")
        origin = 18 + optics[name]["teleop_minus_local_seconds"]
        start = origin + min(int(r["earliest_command_ms"]) for r in rows) / 1000
        end = origin + max(int(r["latest_verified_stow_ms"]) for r in rows) / 1000
        piece = feedback[name]
        windows.append(dict(piece=name, startWallS=start, endWallS=end,
                            phaseA=piece["peakPhaseA"], motorV=piece["peakMotorV"]))
    return windows


def integrated_rack(folder):
    expected_events = "c225b4ade4fdfaea9eb5fae6ffbc76af067838f9ec28284a09e45a9ea2ef49b6"
    expected_path = "62c69e83995e02563d2bd39f554fd1906665a121e140901dcd35d1980f9558e0"
    if sha(folder / "events.csv") != expected_events:
        raise ValueError("unreviewed integrated event clock")
    with (folder / "events.csv").open() as stream:
        events = [r for r in csv.DictReader(stream) if r["scenario"] == "nominal"]
    starts = {r["piece"]: int(r["wall_ms"]) for r in events if r["event"] == "withdraw"}
    stops = {r["piece"]: int(r["wall_ms"]) for r in events if r["event"] == "stow"}
    if set(starts) != {"coral-A", "coral-B"} or starts.keys() != stops.keys():
        raise ValueError("both withdrawals required")
    bins = {}
    for piece, start in starts.items():
        file = folder / f"path-nominal-{piece}.csv"
        if sha(file) != expected_path:
            raise ValueError("integrated actuator samples changed")
        with file.open() as stream:
            for row in csv.DictReader(stream):
                tick = (start + int(row["ms"])) // 20
                amps = abs(float(row["phase_A"]))
                volts = abs(float(row["motor_V"])) + amps * power.electric.PHASE_R["Slide"]
                demand = .7 + amps * volts / .90
                previous = bins.get(tick, (0, 0, 0))
                bins[tick] = max(previous[0], demand), max(previous[1], volts), previous[2] + 1
    return bins, dict(commandWallMs=starts, stowWallMs=stops, eventsSha256=expected_events, motionSha256=expected_path,
                      peakSlideInputW=max(value[0] for value in bins.values()),
                      recordedMillisPerWithdrawal=3001)


def demands(grips, windows, actual=None):
    # Recompute a conservative extension envelope including the 0.440 kg camera
    # allowance. Independent maxima bound the full 3 s quintic and its endpoints.
    extension_v = .370 / 3 * 1.875
    extension_a = .370 / 9 * 10 / math.sqrt(3)
    moving = 2 * (power.rack["loaded_outer_mass_kg"] + .440) + .5 * power.rack["moving_middle_mass_kg"]
    extension = power.motor(moving * extension_a + 54, extension_v / 2, .010,
                            power.UP5 * power.UP4, .70, extension_a / 2, 3e-6)
    starts = [18 + s["drive"] + power.match.corrected().routes(s["sector"])[1].duration + 1.5
              for s in power.plans]
    for wall, _, phase, original, needed in power.make_samples(grips, False, 25):
        watts = original.copy()
        del watts["Observer"]
        amps, motor_v = phase["Slide"]
        if any(wall <= start + 3 and start < wall + power.DT for start in starts):
            amps = max(amps, extension["phase_A"])
            motor_v = max(motor_v, extension["terminal_V"])
        for w in windows:
            if wall <= w["endWallS"] and w["startWallS"] < wall + power.DT:
                amps = max(amps, w["phaseA"])
                motor_v = max(motor_v, w["motorV"])
        needed = dict(needed, Slide=motor_v + amps * power.electric.PHASE_R["Slide"])
        watts["Slide"] = .7 + amps * needed["Slide"] / .90
        tick = round(wall / power.DT)
        if actual is not None and tick in actual:
            new_watts, new_voltage, covered_ms = actual[tick]
            # If any part of this 20 ms bin precedes command or follows the raw
            # trace, retain its original hold/drive requirement as well.
            watts["Slide"] = new_watts
            needed["Slide"] = new_voltage
            if covered_ms < 20:
                watts["Slide"] = max(new_watts, original["Slide"])
                before = phase["Slide"][1] + phase["Slide"][0] * power.electric.PHASE_R["Slide"]
                needed["Slide"] = max(new_voltage, before)
        yield wall, watts, needed


def replay(samples, voc, common, efficiency, withdrawals):
    @lru_cache(maxsize=4096)
    def operating_point(items):
        watts = dict(items)
        for name in SUPPLIES:
            watts[name] = 5.21 * SUPPLIES[name][-1] / efficiency
        return solve(voc, common, watts)

    weakest = headroom = math.inf
    converters = {key: {"minInputV": math.inf, "peakInputA": 0.} for key in SUPPLIES}
    peak = failed = 0
    withdrawal_min = math.inf
    for wall, watts, needed in samples:
        term, current = operating_point(tuple(watts.items()))
        key = min(term, key=term.get)
        if term[key] < weakest:
            weakest, where = term[key], [wall, key]
        if any(wall <= end and start < wall + power.DT for start, end in withdrawals):
            withdrawal_min = min(withdrawal_min, term["Slide"])
        failed += term[key] < 9.5
        headroom = min(headroom, min(term[key] - v for key, v in needed.items()))
        peak = max(peak, sum(current.values()))
        for key, row in converters.items():
            row["minInputV"] = min(row["minInputV"], term[key])
            row["peakInputA"] = max(row["peakInputA"], current[key])
    return dict(openCircuitV=voc, commonOhm=common, assumedEfficiency=efficiency,
                ticks=len(samples), peakA=peak, weakestV=weakest, weakestAt=where,
                belowFloorS=failed * power.DT, minMotorHeadroomV=headroom, converters=converters,
                minimumSlideTerminalInsideNominalPullsV=withdrawal_min)


def run(grip_path, hal_folder):
    if sha(grip_path) != read(HERE / "results.json")["gripTraceSha256"]:
        raise ValueError("grip trace differs from the registered two-pipe match")
    if sha(ROOT / "design/fixed-no-air/whole-robot.json") != read(ROOT / "slide-feedback/no-air.json")["registerSha256"]:
        raise ValueError("two-camera installation changed")
    occupied = {row["channel"] for row in power.electric.CIRCUITS if row["load"] != "Observer"}
    channels = [row[0] for row in SUPPLIES.values()]
    if len(set(channels)) != len(channels) or occupied.intersection(channels):
        raise ValueError("duplicate PDH channel")
    windows = rack_windows()
    samples = list(demands(read(grip_path), windows))
    actual, register = integrated_rack(hal_folder)
    pulls = [(t / 1000, register["stowWallMs"][name] / 1000)
             for name, t in register["commandWallMs"].items()]
    issued = list(demands(read(grip_path), [], actual))
    cases = {name: replay(samples, v, r, efficiency, pulls) for name, v, r, efficiency in (
        ("nominal", 12.4, .025, .85), ("screened", 11.8, .027, .85),
        ("weak", 11.8, .032, .85), ("conversionLoss", 11.8, .027, .75))}
    files = ["fixed-low-power/power.py", "fixed-low-power/retreat.py",
             "design/electrical-inspection/circuits.json", "design/electrical-inspection/replay.py",
             "design/receiver-drives/sizing.json", "slide-feedback/no-air.json",
             "design/fixed-no-air/whole-robot.json", "fixed-height-match/placements.json",
             "slide-feedback/results.json", "slide-feedback/phases.csv", "low-observation/results.json"]
    return {"sourceSha256": {s: sha(ROOT / s) for s in files}, "gripSha256": sha(grip_path),
            "integratedHal": register, "integratedScreenedPack": replay(issued, 11.8, .027, .85, pulls),
            "rackWindows": windows, "inlets": {k: inlet(k) for k in SUPPLIES},
            "extra50mOhmMovingLoop": inlet("MovingCamera", .050), "cases": cases}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("grips", type=Path)
    parser.add_argument("hal_folder", type=Path)
    args = parser.parse_args()
    report = power.electric.round_values(run(args.grips, args.hal_folder))
    (HERE / "dual-camera.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n",
                                          encoding="utf-8", newline="\n")
