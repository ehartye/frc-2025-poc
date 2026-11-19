"""Check the revised withdrawal against both finite pipe trials and the shared electrical load."""
import argparse
import csv
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "selected-drive"))
from route import read, samples, sha
from electrical import screen
import landing


def run(images, output, jdk):
    results = read(output / "results.json")
    paths = results["hal"]["rackPaths"]
    if sha(output / "events.csv") != results["hal"]["events"]:
        raise ValueError("Changed scheduler events")
    t, _, arcs, labels, reference, _, _ = samples(images, "nominal")
    with (output / "events.csv").open() as stream:
        events = list(csv.DictReader(stream))
    weights = read(ROOT / "selected-match/mass.json")["representedKg"]
    mass = weights[1] + 6.17 + 7.50  # operating battery and upper bumper allowance from the drive study
    bounds = {}
    for mode, pieces in results["cases"].items():
        if not mode.startswith("ramp") or not all("stow" in case for case in pieces.values()):
            continue
        target = output / mode
        target.mkdir(exist_ok=True)
        motion = output / f"path-{mode}-coral-A.csv"
        landing.run(output, jdk, motion, output / "events.csv", scenario=mode, registration=output, output=target)
        with (target / "landing-observed.csv").open() as stream:
            contacts = [{k: row[k] for k in ("case", "outcome", "min_front_m", "min_riser_m", "max_compression_m")}
                        for row in csv.DictReader(stream)]
        clocks = {piece: {r["event"]: int(r["wall_ms"]) / 1000 for r in events
                          if r["scenario"] == mode and r["piece"] == piece} for piece in pieces}
        for piece, clock in clocks.items():
            if any(clock[event] != reference[piece][event] for event in ("feed", "carry", "release", "withdraw", "stow")):
                raise ValueError("New trial requires a different wheel demand schedule")
        packs = {}
        for resistance in (.027, .032):
            electrical = screen(t, arcs, labels, clocks, mass, "nominal", (11.8, resistance), output,
                                rack_case=mode, rack_paths=paths)
            packs[str(resistance)] = {k: electrical[k] for k in ("minTerminalV", "firstReferenceInhibitWallS", "limitingBranch")}
            packs[str(resistance)]["slide"] = electrical["branchBounds"]["Slide"]
        bounds[mode] = {"contacts": contacts, "power": packs}
    receipt = {"r103HighKg": weights[1], "operatingHighWithoutPipeKg": mass, "massSource": sha(ROOT / "selected-match/mass.json"),
               "hal": sha(output / "hal.json"), "cases": bounds}
    (output / "qualification.json").write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf8", newline="\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for parameter in ("images", "output", "jdk"):
        parser.add_argument(parameter, type=Path)
    args = parser.parse_args()
    run(args.images.resolve(), args.output.resolve(), args.jdk.resolve())
