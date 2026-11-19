"""Reopen the original achieved navigation input and verify its outbound receiver motion."""

import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "relieved-match"))
import carry


def run(raw):
    row = next(
        v
        for v in carry.read(carry.HERE / "navigation.json")["cases"]
        if v["pipe"] == "coral-A" and v["mode"] == "nominal"
    )
    t, poses, _, _, _, release = carry.frames("coral-A", row, raw, 0.001)
    phase = t - (release - 3)
    u = np.clip(phase / 3, 0, 1)
    expected = 0.370 * u**3 * (10 - 15 * u + 6 * u * u)
    error = float(np.max(np.abs(expected - poses[:, 3])))
    if error > 1e-12 or not row["accepted"] or not np.isclose(poses[:, 3].max(), 0.370):
        raise ValueError("Historical trajectory does not match the stipulated extension")
    files = [
        "relieved-match/carry.py",
        "relieved-match/navigation.py",
        "relieved-match/navigation.json",
        "relieved-match/first-feed.json",
        "design/receiver-drives/size_drives.py",
        "design/receiver-drives/sizing.json",
    ]
    feed = carry.read(carry.HERE / "first-feed.json")
    return dict(
        piece="coral-A",
        mode="nominal",
        nativeNavigationCsvSha256=row["motionSHA256"],
        previousNavigationAccepted=row["accepted"],
        previousDriveWallS=feed["scheduledDriveWallS"],
        previousExtensionStartFromDriveS=release - 3,
        previousReleaseFromDriveS=release,
        nativeFrameCount=len(t),
        nativeFrameStepS=0.001,
        maximumStrokeDifferenceM=error,
        sources={k: hashlib.sha256((ROOT / k).read_bytes()).hexdigest() for k in files},
        scope="Prescribed extension on the previously executed route. Its wall origin and placement permission do not transfer to a different attained arrival. No independent forward rack encoder trajectory.",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("relieved_match_raw", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    a.output.write_bytes(
        (json.dumps(run(a.relieved_match_raw), indent=2, allow_nan=False) + "\n").encode()
    )
