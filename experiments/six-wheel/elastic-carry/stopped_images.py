"""Read the independent stereo views at the end of each modeled one-second arrested hold."""

import argparse
import json
from pathlib import Path
import numpy as np
from carry_images import camera_views, fit_row, silhouette, outcome


def run(assets, folder, rows):
    result = []
    for head in ("roof", "roof-front", "roof-rear", "fixed-native"):
        actual, reference = camera_views(assets, folder, head)
        for r in rows:
            xyz = np.asarray(r["endPipeXyzMm"])
            axis = np.asarray(r["lastAxis"])
            pair = silhouette(actual, xyz, axis / np.linalg.norm(axis))
            fit = fit_row(pair, reference, head.startswith("roof"), 0)
            result.append(
                {
                    "case": r["case"],
                    "monitor": r["monitor"],
                    "head": head,
                    "endWallS": r["endWallS"],
                    "gravityRelativeXYZmm": xyz.tolist(),
                    "readout": fit,
                    "bounded": outcome(fit, [xyz[0], xyz[1], xyz[2] - 557.15]),
                }
            )
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for k in ("artifacts", "folder", "source", "out"):
        p.add_argument(k, type=Path)
    a = p.parse_args()
    rows = json.loads(a.source.read_bytes())
    a.out.write_text(
        json.dumps(run(a.artifacts, a.folder, rows), indent=2) + "\n",
        encoding="utf8",
        newline="\n",
    )
