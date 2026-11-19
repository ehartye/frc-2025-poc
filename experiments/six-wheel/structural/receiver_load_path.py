"""Screen lateral loads on the provisional receiver. All calculated loads are scenario demands."""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "cad/station-receiver/design.json"
CONFIG = Path(__file__).with_name("study.json")


def fastener_shear(points_mm, load_xy_n, application_xy_mm):
    """Rigid, in-plane, equal-stiffness elastic fastener-group load distribution."""
    if len(points_mm) < 2 or len(set(map(tuple, points_mm))) != len(points_mm):
        raise ValueError("Distinct mounting points required")
    vectors = [*points_mm, load_xy_n, application_xy_mm]
    if any(len(v) != 2 or not all(math.isfinite(x) for x in v) for v in vectors):
        raise ValueError("Finite planar vectors required")
    center = [sum(p[i] for p in points_mm) / len(points_mm) for i in range(2)]
    offsets = [(p[0] - center[0], p[1] - center[1]) for p in points_mm]
    rx, ry = [application_xy_mm[i] - center[i] for i in range(2)]
    fx, fy = load_xy_n
    moment_nmm = rx * fy - ry * fx
    polar = sum(x*x + y*y for x, y in offsets)
    loads = [[fx / len(points_mm) - moment_nmm*y/polar,
              fy / len(points_mm) + moment_nmm*x/polar] for x, y in offsets]
    return {"centroidMm": center, "momentNm": moment_nmm/1000,
            "transmittedLoadsN": loads, "maxShearN": max(math.hypot(*f) for f in loads)}


def load_plugin(directory):
    source = directory / "scripts/impact-load-envelope.py"
    spec = importlib.util.spec_from_file_location("frc_impact_screen", source)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    version = json.loads((directory / "package.json").read_text())["version"]
    return mod.scenario, {"version": version, "scriptSha256": hashlib.sha256(source.read_bytes()).hexdigest()}


def calculate(config, fixture, impact):
    if fixture["units"] != "mm" or not math.isclose(fixture["width_mm"], 420):
        raise ValueError("Re-register the project fixture before changing the shelf geometry")
    front = config["shelfRearXmm"] + fixture["length_mm"]
    half_width = fixture["width_mm"] / 2
    mounts = config["shelfMountsMm"]
    cases = []
    for name, pose in config["poses"].items():
        stroke = pose["strokeMm"]
        if not 0 <= stroke <= fixture["slide_travel_mm"]:
            raise ValueError("Pose exceeds registered slide travel")
        x = front + stroke
        shelf_z = pose["liftMm"] - config["shelfTopBelowLiftMm"]
        for study in config["contacts"]:
            demand = impact(study["robotMassKg"], study["peerMassKg"], study["closingSpeedMps"],
                            study["combinedCompressionM"], 0)
            force = demand["averageCompressionForceN"]
            # Side contact at the leading corner. The translating fastener pattern
            # follows the shelf. The carriage-to-chassis support stays fixed in X.
            group = fastener_shear(mounts, [0, -force], [front, half_width])
            carriage_arm = (x - config["carriageReferenceXmm"]) / 1000
            cases.append({"pose": name, "scenario": study["id"], "tipXmm": x,
                          "tipZmm": shelf_z, "frontBumperSetbackMm": config["bumperFrontXmm"]-x,
                          "aboveBumperTopMm": shelf_z-config["bumperTopZmm"],
                          "energyJ": demand["relativeEnergyJ"], "averageSideForceN": force,
                          "shelfGroup": group, "carriageYawMomentNm": -force*carriage_arm,
                          "peakForceN": None, "structuralPass": None})
    return cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin", required=True, type=Path)
    parser.add_argument("--output", type=Path, help="New receipt file; defaults to stdout")
    args = parser.parse_args()
    impact, plugin = load_plugin(args.plugin)
    config = json.loads(CONFIG.read_text())
    result = {"schema": "reefscape-receiver-side-load/1", "plugin": plugin,
              "inputSha256": {p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in (FIXTURE, CONFIG, Path(__file__).resolve())},
              "cases": calculate(config, json.loads(FIXTURE.read_text()), impact)}
    value = json.dumps(result, indent=2, allow_nan=False) + "\n"
    if args.output:
        with args.output.open("x", encoding="utf-8", newline="\n") as stream:
            stream.write(value)
    else:
        print(value, end="")


if __name__ == "__main__":
    main()
