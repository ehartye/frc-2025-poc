"""Draw the actual refined pipe trajectory at the candidate dock at a common axis scale."""

import argparse
import csv
import json
from pathlib import Path


def draw(trace, target):
    with trace.open() as source:
        rows = [
            r
            for r in csv.DictReader(source)
            if r["case"] == "plus/coral-A/3.15/refined"
        ]

    def xy(x, z):
        return 200 + 1650 * x, 382 - 1650 * (z - 0.38)

    def line(points):
        return " ".join(f"{x:.2f},{y:.2f}" for x, y in points)

    t = json.loads(target.read_bytes())
    path = [xy(float(r["x_m"]), float(r["z_m"])) for r in rows]
    floor = [
        (x, t["floor_intercept_m"] + t["floor_slope"] * x) for x in (0.0127, 0.2457)
    ]
    outlines = []
    for index, color in ((0, "#517a96"), (2, "#b9791f"), (-1, "#165b84")):
        x, y = path[index]
        outlines.append(
            f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{1650 * 0.05715:.2f}" stroke="{color}" fill="{color}" fill-opacity=".07" stroke-width="2"/>'
        )
        outlines.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="3.5" fill="{color}"/>')
    wall = [
        xy(0, 0.38),
        xy(0, t["lip_top_m"]),
        xy(t["lip_depth_m"], t["lip_top_m"]),
        xy(t["lip_depth_m"], 0.38),
    ]
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="800" height="490" viewBox="0 -85 800 490" role="img" aria-labelledby="title desc">
<title id="title">One simulated pipe enters the L1 trough</title><desc id="desc">Side view at a 54 mm greater setback. Pipe outlines show release, 0.10 s and its settled position, with the traced center in gold.</desc>
<rect x="0" y="-85" width="800" height="490" fill="#f7fafb"/>
<g font-family="Arial,sans-serif" fill="#183547">
<text x="25" y="-54" font-size="22" font-weight="bold">From the loaded rim into the trough</text>
<text x="25" y="-30" font-size="14">Sector A · plus wheel case · 54 mm greater setback · side view</text>
<polygon points="{line(wall)}" fill="#8496a2"/>
<polyline points="{line([xy(x, z) for x, z in floor])}" fill="none" stroke="#364b56" stroke-width="5"/>
{"".join(outlines)}
<polyline points="{line(path)}" fill="none" stroke="#ad6813" stroke-width="3"/>
<text x="625" y="25" font-size="15" fill="#517a96">Release · 0 s</text>
<text x="625" y="53" font-size="15" fill="#ad6813">Falling · 0.10 s</text>
<text x="625" y="81" font-size="15" fill="#165b84">Supported · 3 s</text>
<text x="81" y="331" font-size="14">Outer wall</text>
<text x="487" y="291" font-size="14">Trough floor</text>
<path d="M628 262h82.5" stroke="#364b56" stroke-width="3"/><text x="645" y="285" font-size="13">50 mm</text>
</g></svg>
'''


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for key in ("trace", "target", "output"):
        p.add_argument(key, type=Path)
    args = p.parse_args()
    args.output.write_text(draw(args.trace, args.target), encoding="utf8", newline="\n")
