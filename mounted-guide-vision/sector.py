"""Repeat signed closed-loop sector registration using the actual mounted guide masks."""

import argparse
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "observed-reef-a"))
from pipe_scene import nav
from features import VisibleFasteners
from coarse_vision import coarse_observation


def run(assets, source=None, modes=None):
    if source is None:
        source = nav.read(HERE / "station.json")
    if modes is None:
        modes = (
            "nominal",
            "wrong-face",
            "dropout",
            "stale",
            "ambiguous",
            "shared-survey-shift",
        )
    home = assets / "mounted-guide-vision"
    baseline = nav.read(ROOT / "conditioned-match/slow-navigation.json")
    old = {
        (r["wheelCase"], r["mode"]): r
        for r in nav.read(ROOT / "observed-reef-a/machined-field-trials.json")
    }
    scenarios = []
    for installation, study in source.items():
        background = home / (installation + "-fixed.npz")
        ctx = nav.checked_scene(
            assets,
            use_guide=False,
            registered_background=(
                background,
                study["register"]["fixedBackgroundSha256"],
            ),
        )
        for side, entry in baseline.items():
            config = nav.read(ROOT / "station-landmark/routes.json")["cases"][side]
            for mode in modes:
                destination = home / installation / side
                destination.mkdir(parents=True, exist_ok=True)
                feature = VisibleFasteners(ctx)
                row = nav.sector.tracecase(
                    "coral-A",
                    entry["initial"],
                    mode,
                    ctx,
                    destination,
                    wheel_scale=config["wheelScale"],
                    physical_track_mm=config["effectiveTrackMm"],
                    observer=coarse_observation,
                    features=feature,
                )
                row.pop("path")
                prior = old[side, mode]
                row.update(
                    installation=installation,
                    wheelCase=side,
                    previousPathSha256=prior["motionSHA256"],
                    sameWheelMotion=row["motionSHA256"] == prior["motionSHA256"],
                    fixedBackgroundSha256=study["register"]["fixedBackgroundSha256"],
                    measuredFastenerRays={
                        str(i): sum(n == i for n, _ in feature.audit) for i in (2, 3, 4)
                    },
                )
                scenarios.append(row)
                print(
                    installation,
                    side,
                    mode,
                    row["accepted"],
                    row["reason"],
                    row["sameWheelMotion"],
                    flush=True,
                )
    return scenarios


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets", type=Path)
    p.add_argument("--setting", action="store_true")
    p.add_argument("--ablation", action="store_true")
    a = p.parse_args()
    label = "ablation" if a.ablation else "setting" if a.setting else ""
    if a.ablation:
        records = nav.read(HERE / "ablation-sources.json")
    elif a.setting:
        records = nav.read(HERE / "setting-station.json")
    else:
        records = None
    (HERE / ((label + "-" if label else "") + "sector.json")).write_text(
        json.dumps(
            run(a.assets, records, ("nominal",) if a.ablation else None), indent=2
        )
        + "\n",
        encoding="utf8",
        newline="\n",
    )
