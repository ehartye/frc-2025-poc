"""Recompute the signed swerve arrival with a specified PE density and shelf friction."""

import argparse
from concurrent.futures import ProcessPoolExecutor
from contextlib import ExitStack
from dataclasses import replace
import importlib.util
import json
import os
from pathlib import Path
import sys
from unittest.mock import patch

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
HERE = Path(__file__).resolve().parent
MOTION = HERE.parent / "motion"
source = importlib.util.spec_from_file_location(
    "numbered_swerve_feed", MOTION / "feed.py"
)
feed = importlib.util.module_from_spec(source)
source.loader.exec_module(feed)
plate = sys.modules[feed.engine.coupled.plate.__module__]
SHELF = feed.engine.coupled.Shelf
NOMINAL_MODULUS = 150000 * 6894.757293168


def one(item):
    jdk, clocks, folder, sign, density, friction, step = item
    root = folder / f"rho-{density:g}-mu-{friction:g}-{sign}-{step:g}us"
    root.mkdir(exist_ok=True, parents=True)

    # The original replay constructs a fresh frozen Shelf and two-sided carriage in this process.
    # Only its floor friction is changed. The guide, belts, pipe and field retain their own coefficients.
    def material(*args, **kwargs):
        return replace(SHELF(*args, **kwargs), friction=friction)

    feed.engine.coupled.tables.cache_clear()
    with ExitStack() as stack:
        stack.enter_context(patch.object(feed.engine.coupled, "Shelf", material))
        stack.enter_context(patch.object(plate, "RHO_PE", density))
        stack.enter_context(patch.object(plate, "E_PE", NOMINAL_MODULUS))
        record = feed.run(
            jdk, clocks, root, sign, step, False, opening=MOTION / "energy.json"
        )
    evidence = {
        "densityKgM3": density,
        "shelfFriction": friction,
        "modulusPa": NOMINAL_MODULUS,
        "feed": record,
    }
    (root / "result.json").write_text(
        json.dumps(evidence, indent=2) + "\n", encoding="utf8", newline="\n"
    )
    print(
        sign,
        density,
        friction,
        step,
        record["state"],
        record["javaFirstBandMs"],
        flush=True,
    )
    return evidence


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("jdk", type=Path)
    p.add_argument("clocks", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument(
        "--condition",
        nargs=3,
        action="append",
        metavar=("DENSITY", "FRICTION", "STEP_US"),
        required=True,
    )
    p.add_argument("--workers", type=int, default=2)
    a = p.parse_args()
    jobs = [
        (a.jdk, a.clocks, a.output, sgn, *map(float, c))
        for c in a.condition
        for sgn in ("plus", "minus")
    ]
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        list(pool.map(one, jobs))
