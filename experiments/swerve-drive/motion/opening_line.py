"""Score the primary station route's start and AUTO end against numbered R4 geometry."""

import argparse
import csv
import json
import math
import subprocess
import sys
from pathlib import Path

from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec

from park_endpoint import HERE, R4, ROOT, SIX, assembly, park, read

CASES = ("plus", "minus", "heavy", "upper-minus")
# The disabled plant requests a zero voltage cap. Reconstructed motor-terminal
# voltage includes floating-point cancellation of winding drop and back EMF.
DISABLED_RESIDUAL_V = 1e-10


def start_cloth(covers, pose, width_mm, inset_mm):
    """Intersect material at its actual heading with a vertical field-tape prism."""
    west, east = [inch * 25.4 for inch in park.TAPE]
    prism = BRepPrimAPI_MakeBox(
        gp_Pnt(west + inset_mm, inset_mm, -10),
        gp_Pnt(east - inset_mm, width_mm - inset_mm, 1200),
    ).Shape()
    placement = gp_Trsf()
    placement.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), pose[2])
    placement.SetTranslationPart(gp_Vec(pose[0] * 1000, pose[1] * 1000, 0))
    result = {}
    for name, shape in covers.items():
        installed = BRepBuilderAPI_Transform(shape, placement, False).Shape()
        overlap = BRepAlgoAPI_Common(installed, prism)
        if not overlap.IsDone():
            raise ValueError("Tape intersection failed for " + name)
        result[name] = assembly.cad.volume(overlap.Shape())[0]
    return result


def west_gap(pose, bounds, rounding_mm):
    x, _, yaw = pose
    west = min(x * 1000 + u * math.cos(yaw) - v * math.sin(yaw)
               for u in (bounds[0], bounds[3]) for v in (bounds[1], bounds[4]))
    return west - park.TAPE[1] * 25.4 - rounding_mm


def disabled_voltage(wheels):
    maximum = 0.0
    seen = set()
    with wheels.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            # The wheel record is stamped at the END of its 5 ms command interval.
            ms = round(1000 * float(row["wall_s"]))
            if 15000 < ms <= 18000:
                key = (ms, row["id"])
                if key in seen:
                    raise ValueError("Repeated disabled module observation")
                seen.add(key)
                for channel in ("drive_voltage_v", "steer_voltage_v"):
                    value = float(row[channel])
                    if not math.isfinite(value):
                        raise ValueError("Invalid disabled voltage")
                    maximum = max(maximum, abs(value))
    required = {(ms, corner) for ms in range(15005, 18001, 5)
                for corner in ("FL", "FR", "RL", "RR")}
    if seen != required:
        raise ValueError("Incomplete disabled wheel clock")
    return maximum


def review(directory, recordings):
    issued = read(HERE / "energy.json")
    for rel, expected in issued["sourceSha256"].items():
        if assembly.sha(HERE / rel) != expected:
            raise ValueError("Opening input changed: " + rel)
    endpoint = read(HERE / "park-endpoint.json")
    for relative in ("cad/full-swerve-r4/full-robot.step", "experiments/six-wheel/leave-park/rehearse.py",
                     "experiments/swerve-drive/packaging/envelopes.json", "cad/full-swerve-r4/appearances.json"):
        if assembly.sha(ROOT / relative) != endpoint["pins"][relative]:
            raise ValueError("Field or robot configuration changed: " + relative)
    bodies = assembly.named(R4 / "full-robot.step")
    covers = {name: shape for name, shape in bodies.items() if name.startswith("bumper-cover-")}
    if set(covers) != {"bumper-cover-front-red", "bumper-cover-rear-red"}:
        raise ValueError("Numbered assembly cover identities changed")
    hull = endpoint["hullMm"]
    radius = max(math.hypot(x, y) for x in (hull[0], hull[3]) for y in (hull[1], hull[4]))
    # The solver rounds the three pose columns to five decimal places (metres, metres, radians).
    rounding_mm = 0.005 + 2 * radius * math.sin(0.000005 / 2)
    field_size = park.corrected().PROFILE["field"]
    width_mm = field_size["width"] * 25.4
    field_record = SIX / "leave-park/native-park.json"
    registered = read(field_record)
    if registered["field_step_sha256"] != endpoint["fieldSha256"]:
        raise ValueError("Starting-line occurrence belongs to a different field")
    box = registered["support_surfaces"]["NAUO31:tape_AutoLine"]
    field_center_x = field_size["length"] * 25.4 / 2
    native_edges = [field_center_x + box[i] for i in (0, 3)]
    line_error = max(abs(a - 25.4 * b) for a, b in zip(native_edges, park.TAPE))
    if abs(box[2] + width_mm / 2) > 0.001 or abs(box[5] - width_mm / 2) > 0.001:
        raise ValueError("Registered tape no longer spans the field")
    allowance = rounding_mm + line_error
    cases = {}
    for name in CASES:
        data = recordings[name]
        wheels = directory / (name + ".csv")
        expected = issued["cases"][name]
        if (data["case"] != name or assembly.sha(wheels) != expected["csvSha256"]
                or data["csvSha256"] != expected["csvSha256"]
                or data["tapeGapMmAt15s"] != expected["tapeGapMmAt15s"]):
            raise ValueError("Replayed opening differs from the issued wheel case: " + name)
        poses = {round(row[0] * 1000): row[1:4] for row in data["samples"]}
        placed = poses[0]
        cloth = start_cloth(covers, placed, width_mm, allowance)
        displaced = [placed[0] + 1, *placed[1:]]
        outside = sum(start_cloth(covers, displaced, width_mm, 0).values())
        gap = west_gap(poses[15000], hull, allowance)
        stop_voltage = disabled_voltage(wheels)
        if outside > 1e-4:
            raise ValueError("Off-line control reached the starting tape")
        cases[name] = dict(
            massKg=data["assumedGrossMassKg"], wheelCsvSha256=expected["csvSha256"],
            startPoseMAndRad=placed, autoEndPoseMAndRad=poses[15000],
            startClothLowerMm3=cloth, shiftedStartClothMm3=outside,
            autoEndWholeRobotGapLowerMm=gap, disabledMaxLeadV=stop_voltage,
            modeledLeavePoints=3 if sum(cloth.values()) > 1e-4 and gap > 0 and stop_voltage < DISABLED_RESIDUAL_V else 0,
        )
    return dict(configuration=endpoint["configuration"], manualEdition="archive-compiled",
                openingRegisterSha256=assembly.sha(HERE / "energy.json"),
                geometryRegisterSha256=assembly.sha(HERE / "park-endpoint.json"),
                nativeTapeRegisterSha256=assembly.sha(field_record), nativeTapeFieldEdgesMm=native_edges,
                tapeRegistrationAllowanceMm=line_error, coordinateRoundingAllowanceMm=rounding_mm,
                disabledResidualLimitV=DISABLED_RESIDUAL_V,
                tapeEdgesIn=list(park.TAPE), cases=cases)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts", type=Path, help="project station and optics inputs")
    parser.add_argument("directory", type=Path, help="output directory for four new wheel traces")
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    runs = {}
    for case in CASES:
        subprocess.run([sys.executable, HERE / "opening.py", args.artifacts, args.directory,
                        "--case", case], check=True)
        runs[case] = read(args.directory / (case + ".json"))
    result = review(args.directory, runs)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
