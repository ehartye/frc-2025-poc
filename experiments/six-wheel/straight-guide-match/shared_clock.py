"""Join the modeled supply, wheel service, traffic and opening on one field clock."""

import argparse
import json
import sys
from pathlib import Path
from plant import HERE, ROOT, Selection, read

sys.path.insert(0, str(ROOT / "observed-reef-a"))
from field_screen import reconcile


def run_case(assets, side, step, mode="nominal", stop=None):
    camera = read(HERE / "revised-opening.json")[side]
    source = read(HERE / f"station-{side}-{step:g}.json")
    if not source["guardedStationHandoff"]:
        raise ValueError("No observed pipe for loaded path")
    seat_time = 21.571672635314284 + source["firstInBandMs"] / 1000
    r = reconcile(
        assets,
        side,
        mode,
        stop,
        selected=Selection(step, True),
        source_opening=(
            assets / "straight-guide-match" / (side + "-revised-opening.npz"),
            camera["pathSha256"],
        ),
        seat_wall_s=seat_time,
    )
    r.update(feedSha256=source["pipeSha256"], stockId="coral-A", seatWallS=seat_time)
    return r


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("assets", type=Path)
    p.add_argument("side")
    p.add_argument("step", type=float)
    p.add_argument("--mode", default="nominal")
    p.add_argument("--stop", type=float)
    p.add_argument("--output", required=True, type=Path)
    a = p.parse_args()
    a.output.write_text(
        json.dumps(run_case(a.assets, a.side, a.step, a.mode, a.stop), indent=2) + "\n",
        encoding="utf8",
        newline="\n",
    )
