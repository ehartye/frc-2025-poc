"""Exercise the known signed approach poses with finite stowed objects and exposure delays."""

import argparse
import json
from pathlib import Path

from pipe_scene import HERE, ROOT, nav, bound_machined_shelf
from features import VisibleFasteners
from coarse_vision import coarse_observation


def run(assets, machined=False, shelf_guard=False):
    machined = machined or shelf_guard
    variant = "machined-guard" if shelf_guard else "machined" if machined else ""
    root = assets / "observed-reef-a"
    root.mkdir(exist_ok=True)
    baseline = nav.read(ROOT / "conditioned-match/slow-navigation.json")
    if machined:
        register = nav.read(HERE / "station-bridge.json")
        raster = assets / "observed-reef-a/machined-fixed.npz"
        envelope = nav.checked_scene(
            assets,
            registered_background=(
                raster,
                register["leftSmall"]["machinedFieldBackgroundSha256"],
            ),
        )
    else:
        envelope = nav.checked_scene(assets)
    guard = bound_machined_shelf(envelope) if shelf_guard else None
    rows = []
    for side, entry in baseline.items():
        config = nav.read(ROOT / "station-landmark/routes.json")["cases"][side]
        for mode in (
            "nominal",
            "wrong-face",
            "dropout",
            "stale",
            "ambiguous",
            "shared-survey-shift",
        ):
            features = VisibleFasteners(envelope)
            folder = root / variant / side
            folder.mkdir(exist_ok=True, parents=True)
            r = nav.sector.tracecase(
                "coral-A",
                entry["initial"],
                mode,
                envelope,
                folder,
                wheel_scale=config["wheelScale"],
                physical_track_mm=config["effectiveTrackMm"],
                observer=coarse_observation,
                features=features,
            )
            r.pop("path")
            r.update(
                wheelCase=side,
                raysUsed={
                    str(k): sum(n == k for n, _ in features.audit) for k in (2, 3, 4)
                },
            )
            if mode in ("nominal", "ambiguous", "dropout"):
                r["studentFrames"] = []
                for clock in (630, 1190, 1680, 2170, 2660, 3150):
                    candidates = [
                        obs
                        for obs in features.observations
                        if obs["captureMs"] <= clock
                    ]
                    if candidates and (
                        not r["studentFrames"]
                        or candidates[-1]["captureMs"]
                        != r["studentFrames"][-1]["captureMs"]
                    ):
                        r["studentFrames"].append(candidates[-1])
            if machined:
                r["cameraBackgroundSha256"] = nav.sha(raster)
            if guard is not None:
                r["expandedShelfBoxMm"] = guard
            rows.append(r)
            print(
                side,
                mode,
                r["accepted"],
                r["reason"],
                r["decodedTagFrames"],
                r["pairedHeadFrames"],
                r["latestInterval"],
                r["finalBodyErrorMmDeg"],
                flush=True,
            )
    (HERE / ((variant + "-" if variant else "") + "field-trials.json")).write_text(
        json.dumps(rows, indent=2) + "\n", encoding="utf8", newline="\n"
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets", type=Path)
    p.add_argument("--machined", action="store_true")
    p.add_argument("--shelf-guard", action="store_true")
    a = p.parse_args()
    run(a.assets, a.machined, a.shelf_guard)
