"""Check the modeled station aperture ray bundle against the newly installed swerve geometry."""

import argparse
import json
import math
from pathlib import Path
import numpy as np
from opening import HERE, ROOT, sha, read, n
import panel


def qualify(directory):
    data = read(ROOT / "swerve-package/envelopes.json")
    power=read(ROOT/"swerve-power/envelopes.json")
    groups = {
        "rotating_and_fixed_modules": data["cornerBoundsMm"],
        "changed_structure": data["frameBoundsMm"],
        "controllers_trays_and_wiring":power["stationaryBoundsMm"],
    }
    f, _, _, _ = n.route.route.field.scenario(
        read(ROOT / "fixed-height-match/placements.json")["placements"], inboard=True
    )
    distances = []
    for file in sorted(directory.glob("*.json")):
        record = read(file)
        if not record.get("imagesTaken"):
            continue
        start = next(
            t for t, label in record["events"] if label == "station acquisition range"
        )
        for t, x, y, *_ in record["samples"]:
            if t >= start - 0.11:
                distances.append(math.dist((x, y), f.DOCK) * 1000)
    # Include the furthest ring in the panel, both camera pupils, robot pose variation
    # between 100 ms viewer samples, and the full 6.35 mm panel thickness.
    reach = (
        max(distances)
        + math.hypot(444.525, max(abs(panel.CORNERS[:, 1])) + panel.RADIUS)
        + math.hypot(315.0, 37.5)
        + abs(970.0 - panel.CORNERS[0, 2])
        + 150.0
        + 6.35
    )
    lowest = (
        min(970.0, float(panel.CORNERS[0, 2]) - panel.RADIUS)
        - (11.0 + 3.0) * reach / panel.FOCAL_PX
    )
    register = {}
    for name, values in groups.items():
        if not values:
            raise ValueError("Missing geometry group " + name)
        arr = np.array(list(values.values()))
        bounds = np.r_[arr[:, :3].min(axis=0), arr[:, 3:].max(axis=0)]
        register[name] = dict(
            items=len(arr),
            boundsMm=bounds.tolist(),
            upperRayGapMm=lowest - bounds[5],
            insetFromAssumedBumperMm=425.0 - max(abs(bounds[[0, 1, 3, 4]])),
        )
    return dict(
        scope="Material added or changed in these maps, including local modeled wires. Earlier robot and station registration retained. Unmodeled connector tails, glare, extrinsic drift, wear and physical sightlines remain open.",
        inputs={
            p: sha(ROOT / p)
            for p in (
                "swerve-package/envelopes.json",
                "swerve-power/envelopes.json",
                "station-landmark/native.json",
            )
        },
        sourceBackground=directory.joinpath("plus.json").is_file()
        and read(directory / "plus.json")["backgroundSha256"],
        paddedRayLengthMm=reach,
        lowestUpperHoleRayIncluding14PixelAllowanceMm=lowest,
        groups=register,

    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("raw", type=Path)
    p.add_argument("out", type=Path)
    a = p.parse_args()
    a.out.write_text(
        json.dumps(qualify(a.raw), indent=2) + "\n", encoding="utf8", newline="\n"
    )
