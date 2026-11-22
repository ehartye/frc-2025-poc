"""Cross-check the actual cold port cartridge, its head seats, and both independently exported STEP payloads."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def exchange_data(path):
    contents = path.read_bytes()
    if contents.count(b"DATA;") != 1:
        raise ValueError("Expected one complete STEP DATA section")
    section, close = contents.split(b"DATA;", 1)[1].split(b"ENDSEC;", 1)
    if b"END-ISO-10303-21;" not in close:
        raise ValueError("STEP exchange incomplete")
    return hashlib.sha256(section).hexdigest()


def check(package, station):
    sys.path.insert(0, str(package / "scripts"))
    from fusion_authority import digest, register_path, verify
    from mechanical_motion import bearing_area, bounds, distance
    from showcase_contacts import load_solids

    register = register_path(ROOT)
    canonical = register.parent / "issue-004/assembly.step"
    auth = verify(ROOT, register, canonical.with_name("lineage.json"))
    issued = json.loads((station / "report.json").read_bytes())
    if issued["verifiedAuthority"] != auth or issued["stepSha256"] != digest(station / "port-joint.step"):
        raise ValueError("Native artifact binding changed")
    data = exchange_data(canonical)
    if data != exchange_data(station / "port-joint.step"):
        raise ValueError("Independent geometry exports differ")
    solids, sheets = load_solids(canonical)
    if len(solids) != 504 or sheets:
        raise ValueError("Incomplete installed chassis")
    placed = {name: np.asarray(bounds(solid)) for name, solid in solids.items()}
    phase = json.loads((HERE / "port-504-seated-phase-inputs.json").read_bytes())
    trial = json.loads((HERE / "port-504-seated-comparison.json").read_bytes())
    seats = {}
    for name, row in trial["cases"].items():
        source = next(r for r in issued["coldCartridge"] if r["path"] == row["nativeOccurrence"])
        errors = sorted((max(abs(box - source["boundsMm"])), label) for label, box in placed.items())
        if errors[0][0] > .1 or errors[1][0] < .5 or errors[0][1] != row["placedSolid"]:
            raise ValueError(f"Ambiguous fitted screw {name}")
        screw = solids[errors[0][1]]
        seat = solids[row["bearingHost"]]
        area = bearing_area(screw, seat, phase["cases"][name]["intoScrewAxis"])
        gap = distance(screw, seat)
        if abs(area - row["measured"][0]["underheadAreaMm2"]) > .001 or gap > .001:
            raise ValueError(f"Head failed to seat at {name}")
        seats[name] = dict(solid=errors[0][1], seat=row["bearingHost"], areaMm2=area, gapMm=gap)
    if len(seats) != 6:
        raise ValueError("Expected four primary anchors and two side keepers")
    return dict(authority=auth, installedSolids=len(solids), diagnosticStepSha256=issued["stepSha256"],
                stepDataSha256=data, allSixHeadSeats=seats,
                scope="Parked, unloaded geometry. The native issue contains separate dual-method interference checks. Actual service preload and the operating assembly are separate.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("package", type=Path)
    parser.add_argument("station", type=Path)
    args = parser.parse_args()
    result = check(args.package, args.station)
    (HERE / "port-504-independent.json").write_text(json.dumps(result, indent=2) + "\n")
    print(result["stepDataSha256"], len(result["allSixHeadSeats"]))
