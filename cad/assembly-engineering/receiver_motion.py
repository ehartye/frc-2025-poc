"""Sweep the new receiver material against every other slide frame of the shop robot."""

import csv
import json
from collections import Counter

from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut

from attachments import base, bounds, volume
from clearance import evaluate
from installed_audit import HERE, reopen, sha
from issue import document
from receiver_frames import corrections

OUT = HERE / "receiver-service"
FRAMES = {
    "Outer slide": 1.0,
    "Left capture yoke": 1.0,
    "Right capture yoke": 1.0,
    "Middle slide": 0.5,
}


def assembly():
    issued = HERE / "issue"
    original = {k: shape for k, *_, shape in reopen()}
    original.update(base.named(issued / "station-and-jack-interfaces.step"))
    for k in json.loads((issued / "interfaces.json").read_bytes())["fixedHeight"][
        "removed"
    ]:
        original.pop(k)
    with (issued / "cold-comparison.csv").open(encoding="utf8", newline="") as src:
        frame = {
            r["id"]: FRAMES.get(r["occurrence"].split(":")[0], 0.0)
            for r in csv.DictReader(src)
        }
    frame.update({k: 1.0 for k in corrections()})
    battery = base.named(HERE / "battery-fit/battery-retention.step")
    grip = base.named(OUT / "grip-carriers.step")
    switches = base.named(OUT / "switch-mounts.step")
    installed = original | battery | grip | switches
    removed = json.loads((OUT / "grip-fit.json").read_bytes())["removed"] + [
        "slide-middle-stop-dog-leg"
    ]
    for k in removed:
        installed.pop(k)
    for k in battery:
        frame.setdefault(k, 0.0)
    for k in grip:
        frame.setdefault(k, 1.0)
    for k in switches:
        frame.setdefault(k, 0.5 if k.startswith("slide-middle-dog-") else 0.0)
    assert set(installed) <= set(frame)
    return original, installed, frame, grip | switches


def material(k, shape, original):
    # Holes remove material. Only positive material newly entering a distinct
    # frame can introduce a fresh obstruction. Switch envelopes use full bodies.
    if k not in original or "-switch" in k:
        return shape
    source = original[k]
    if k == "slide-middle-stop-dog":
        from attachments import fuse

        source = fuse(source, original["slide-middle-stop-dog-leg"])
    diff = BRepAlgoAPI_Cut(shape, source).Shape()
    if diff.IsNull() or volume(diff) < 0.001:
        return None
    return diff


def run():
    original, installed, frame, revisions = assembly()
    triples = [(k, s, bounds(s)) for k, s in installed.items()]
    cases = {}
    for k, changed in revisions.items():
        new = material(k, changed, original)
        if new is None:
            continue
        cases[k] = {}
        for fraction in (0.0, 0.5, 1.0):
            dx = frame[k] - fraction
            if not dx:
                continue
            # Spring levers contact the cam on purpose. Their permitted
            # displacement is assessed independently against Omron's catalog.
            peers = [
                (j, s, b)
                for j, s, b in triples
                if frame[j] == fraction
                and not (
                    {k, j} & {"slide-middle-stop-dog"}
                    and any(t.endswith("-switch-lever") for t in (j, k))
                )
            ]
            cases[k][str(fraction)] = evaluate(
                new, peers, (1 if dx > 0 else -1, 0, 0), 0, abs(dx) * 370
            )
    # Both yokes inherit the outer-stage X travel. Their physical closing
    # stroke is an independent Y translation relative to the grip.
    yokes = json.loads(
        (
            HERE.parents[1] / "experiments/swerve-drive/retention/open-yokes.json"
        ).read_bytes()
    )
    for row in yokes["parts"]:
        if row["frame"] != "yoke":
            continue
        k = row["id"]
        peers = [
            (j, s, b) for j, s, b in triples if j in revisions and j.startswith("grip-")
        ]
        cases.setdefault(k, {})["jaw-closing"] = evaluate(
            installed[k], peers, (0, -1 if "-left-" in k else 1, 0), 0, 35
        )
    unresolved = [
        (k, f, r)
        for k, ff in cases.items()
        for f, d in ff.items()
        for r in d["adverse"]
    ]
    receipt = dict(
        source={
            name: sha(path)
            for name, path in (
                ("parent", HERE / "issue/station-and-jack-interfaces.step"),
                ("battery", HERE / "battery-fit/battery-retention.step"),
                ("grip", OUT / "grip-carriers.step"),
                ("switches", OUT / "switch-mounts.step"),
            )
        },
        installed=len(installed),
        frameCounts=dict(Counter(frame[k] for k in installed)),
        rackTravelMm=[0, 370],
        checked=cases,
        adverse=unresolved,
    )
    document(OUT / "travel.json", receipt)
    print("Frame counts", receipt["frameCounts"], "adverse", unresolved, flush=True)


if __name__ == "__main__":
    run()
