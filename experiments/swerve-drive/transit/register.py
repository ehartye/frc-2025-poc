"""Match the source-bound loaded wheel and cargo inputs to the current sector trial."""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
MOTION = HERE.parent / "motion"
sys.path.insert(0, str(MOTION))
from loaded_departure import read, sha, write as write


def prior(raw, case, step):
    record = read(MOTION / "loaded-departure.json")
    row = next(r for r in record["cases"] if (r["case"], r["stepUs"]) == (case, step))
    route = row["routes"]["600"]
    folder = raw / f"{case}-{step:g}us"
    state = folder / (case + "-600-wheels.npz")
    pipe = (folder / route["pipe"]["file"]).with_suffix(".npz")
    if (
        sha(state) != route["wheel"]["wheelStateAndEncoderTrace"]
        or sha(pipe) != route["pipe"]["terminalStateSha256"]
        or route["pipe"]["piece"] != "coral-A"
        or not all(r["accepted"] for r in route["vision"]["frames"][-5:])
        or row["options"]["observed"] != 600
    ):
        raise ValueError("Terminal shaft, contact or custody source differs")
    return row, route, state, pipe
