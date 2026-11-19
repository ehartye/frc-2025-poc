"""Build a portable field replay from the recorded simulations and official field registration."""

import argparse
import json
import math
from pathlib import Path
from shared_clock import HERE, ROOT, read
from field_screen import nav


def make(files):
    rows = [read(path) for path in files]
    plans = read(ROOT / "fixed-height-match/placements.json")["placements"]
    field, _, team, _ = nav.route.route.field.scenario(plans, inboard=True)
    stock = []
    for name, r in read(ROOT / "full-match/installed-stock.json")["stock"].items():
        x, z, y = r["native_center_mm"]
        fx, fy = field.field_from_native(x / 25.4, y / 25.4)
        stock.append([r["group"], round(fx, 4), round(fy, 4), round(z, 1)])
    peers = {}
    for identity, driver in team.items():
        if identity != "red2":
            peers[identity] = [
                [round(float(k), 5) for k in driver.at(max(0, t - 18))[0]]
                for t in range(157)
            ]
    cases = []
    for row in rows:
        cases.append(
            {
                k: row[k]
                for k in (
                    "side",
                    "stepUs",
                    "condition",
                    "outcome",
                    "points",
                    "events",
                    "pose",
                )
            }
        )
    length = field.PROFILE["field"]["length"] * field.IN
    width = field.PROFILE["field"]["width"] * field.IN
    walls = []
    for plane in field.PROFILE["rehearsal"]["planes"]:
        x, y = [v * field.IN for v in plane["at"]]
        h = math.radians(plane["inwardDeg"])
        nx, ny = math.cos(h), math.sin(h)
        candidates = []
        if abs(ny) > 1e-6:
            candidates += [(cx, y - (cx - x) * nx / ny) for cx in (0, length)]
        if abs(nx) > 1e-6:
            candidates += [(x - (cy - y) * ny / nx, cy) for cy in (0, width)]
        vertices = [
            point
            for point in candidates
            if 0 <= point[0] <= length and 0 <= point[1] <= width
        ]
        if len(vertices) == 2:
            walls.append(vertices)
    package = dict(
        walls=walls,
        tapeX=(field.PROFILE["field"]["length"] - 144 - 65.5 - 88) * field.IN,
        parkRect=[length / 2 - 23 * field.IN, 0, 46 * field.IN, 146.5 * field.IN],
        field=[
            field.PROFILE["field"]["length"] * field.IN,
            field.PROFILE["field"]["width"] * field.IN,
        ],
        obstacles=[[k, list(v)] for k, v in field.obstacles()],
        stock=stock,
        peers=peers,
        cases=cases,
    )
    html = (HERE / "replay-template.html").read_text(encoding="utf8")
    if html.count("/*MATCH_DATA*/") != 1:
        raise ValueError("Missing data insertion")
    return html.replace(
        "/*MATCH_DATA*/", json.dumps(package, separators=(",", ":"), allow_nan=False)
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("files", type=Path, nargs="+")
    a = p.parse_args()
    (HERE / "replay.html").write_text(make(a.files), encoding="utf8", newline="\n")
