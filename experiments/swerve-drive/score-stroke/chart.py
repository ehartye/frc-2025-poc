"""Plot the actual signed R4 cap intersection and compression-only pipe trace."""
import argparse
import json
from pathlib import Path
import numpy as np


def polyline(points, color, width=3):
    coords = " ".join(f"{x:.2f},{y:.2f}" for x,y in points)
    return f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="{width}"/>'


def chart(result, traces):
    blue, gold = "#23638e", "#ad6411"
    paths, notes = [], []
    for sign,color in (("plus",blue),("minus",gold)):
        rows = [row for row in result["caps"][sign] if row["side"] == 0]
        peak = max(row["issuedCommonMm3"] for row in result["caps"]["plus"] if row["side"] == 0)
        y_limit = max(5,5*np.ceil(peak/5))
        paths.append(polyline(((66+100*(r["strokeMm"]-21),217-137*r["issuedCommonMm3"]/y_limit) for r in rows),color))
        with np.load(traces/f"{sign}-free-31.25us-3s-400000.npz") as data:
            samples = data["samples"]
        # Preserve the minima and maxima of each 25 ms interval at their actual times.
        envelope = []
        for indices in np.array_split(np.arange(len(samples)),125):
            subset = samples[indices,3]
            for index in sorted({indices[np.argmin(subset)], indices[np.argmax(subset)]}):
                t,force = samples[index,[0,3]]
                envelope.append((492+90*t,217-6.3*force))
        paths.append(polyline(envelope,color,1.6))
    for x,y,label in ((66,70,f"{int(y_limit)} mm³"),(66,238,"21 mm"),(324,238,"24 mm"),(492,70,"22 N"),
                      (492,238,"0 s"),(744,238,"3 s"),(48,221,"0"),(474,221,"0")):
        notes.append(f'<text x="{x}" y="{y}" font-size="13">{label}</text>')
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="856" height="285" viewBox="0 0 856 285" role="img" aria-labelledby="title desc">
<title id="title">R4 scoring stroke evidence</title>
<desc id="desc">Left shows the installed rear cap entering the moving rib. Right shows normal support on the independent pipe if the buttons are withdrawn.</desc>
<rect width="856" height="285" fill="#f8fafb"/>
<g font-family="Arial,sans-serif" fill="#192f40">
<text x="40" y="38" font-size="20" font-weight="bold">Installed cap and shelf overlap</text>
<text x="473" y="38" font-size="20" font-weight="bold">Pipe support with buttons withdrawn</text>
<path d="M66 80V217H366 M492 80V217H775" fill="none" stroke="#778894"/>
{''.join(paths)}{''.join(notes)}
<text x="40" y="270" font-size="14">Both corrected wheel cases</text>
<text x="277" y="270" font-size="14" fill="{blue}">Blue · plus</text>
<text x="385" y="270" font-size="14" fill="{gold}">Gold · minus</text></g></svg>
'''


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ("native","traces","output"):
        parser.add_argument(key,type=Path)
    args=parser.parse_args()
    args.output.write_text(chart(json.loads(args.native.read_bytes()),args.traces),encoding="utf8",newline="\n")
