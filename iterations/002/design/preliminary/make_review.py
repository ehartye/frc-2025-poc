"""Assemble the LAN review bundle: the generated player plus its local retention figure. No STEP or CAD source.

    python -B make_review.py PROJECT_ROOT
"""
import html
import json
import math
import shutil
import sys
from pathlib import Path

ROOT = Path(sys.argv[1]).resolve()
PRE = ROOT / "iterations/002/design/preliminary"
OUT = PRE / "review"
OUT.mkdir(exist_ok=False)
shutil.copyfile(PRE / "package-motion.html", OUT / "package-motion.html")

geo = json.loads((ROOT / "iterations/002/design/pan/geometry.json").read_text(encoding="utf-8"))
ret = json.loads((PRE / "retention-review.json").read_text(encoding="utf-8"))
blocks = json.loads((PRE / "layout-blocks.json").read_text(encoding="utf-8"))
skin = [(p[0], p[2]) for p in next(x for x in geo["parts"] if x["id"] == "Port cheek skin")["sections"][0]]
px, pz = 205.0, 580.0
r = blocks["poses"]["pipe"]["apothemMm"]
seat = blocks["poses"]["pipe"]["seatFlatXZ"]


def pose(points, theta):
    t = math.radians(theta)
    return [(px + (x - px) * math.cos(t) - (z - pz) * math.sin(t), pz + (x - px) * math.sin(t) + (z - pz) * math.cos(t))
            for x, z in points]


def svg_panel(theta, ox, title, pipe_flat, note, arrow=None):
    W, H, s = 420, 360, 0.55
    tx = lambda x: ox + 30 + (x + 60) * s
    ty = lambda z: H - 20 - (z - 380) * s
    poly = " ".join(f"{tx(x):.1f},{ty(z):.1f}" for x, z in pose(skin, theta))
    (cx, cz), = pose([pipe_flat], theta)
    out = [f'<polygon points="{poly}" fill="#007F78" fill-opacity=".35" stroke="#007F78"/>',
           f'<circle cx="{tx(cx):.1f}" cy="{ty(cz):.1f}" r="{r * s:.1f}" fill="#E6B450" stroke="#7a5a10"/>',
           f'<circle cx="{tx(px):.1f}" cy="{ty(pz):.1f}" r="4" fill="#BCBDCA" stroke="#333"/>',
           f'<line x1="{ox + 20}" y1="{ty(0) if False else H - 20}" x2="{ox + W - 20}" y2="{H - 20}" stroke="#666"/>',
           f'<text x="{ox + 20}" y="24" font-size="15" font-weight="bold">{html.escape(title)}</text>',
           f'<text x="{ox + 20}" y="44" font-size="12">{html.escape(note)}</text>',
           f'<line x1="{tx(cx):.1f}" y1="{ty(cz):.1f}" x2="{tx(cx):.1f}" y2="{ty(cz) + 50:.1f}" stroke="#222" marker-end="url(#a)"/>',
           f'<text x="{tx(cx) + 6:.1f}" y="{ty(cz) + 50:.1f}" font-size="11">g</text>']
    if arrow:
        (ex, ez), = pose([arrow], theta)
        out.append(f'<line x1="{tx(cx):.1f}" y1="{ty(cz):.1f}" x2="{tx(ex):.1f}" y2="{ty(ez):.1f}" stroke="#D55E00" '
                   f'stroke-width="2.5" stroke-dasharray="6 4" marker-end="url(#a)"/>')
    return out


finding = {f["id"]: f["text"] for f in ret["findings"]}
parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="860" height="470" font-family="sans-serif">',
         '<defs><marker id="a" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">'
         '<path d="M0,0L10,5L0,10z"/></marker></defs><rect width="860" height="470" fill="#fff"/>']
parts += svg_panel(70.0, 0, "Loaded pose: stow 70 deg, powered", seat,
                   f"Lip and floor carry the pipe; lift-off above {ret['stowLiftoffForwardAccelMps2']:.2f} m/s2 forward (R1)",
                   arrow=(seat[0], seat[1] + 120))
parts += svg_panel(-5.0, 430, "Power-off escape control", (seat[0] + 330, seat[1]),
                   "Unpowered 3 g opening: pan falls to -5 deg soft stop, pipe exits the front (R2)",
                   arrow=(seat[0] + 460, seat[1]))
y = 400
for k in ("R1", "R2"):
    parts.append(f'<text x="20" y="{y}" font-size="11">{html.escape(k + ": " + finding[k][:150])}</text>')
    y += 18
parts.append(f'<text x="20" y="{y}" font-size="11">Side view, X forward, Z up; pan section is the port cheek skin outline (r3). '
             f'Dashed arrow: escape direction. Source: retention-review.json.</text></svg>')
(OUT / "retention.svg").write_text("\n".join(parts) + "\n", encoding="utf-8")
(OUT / "index.html").write_text(
    '<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
    '<title>Red 2 baseline A review</title><body style="font-family:sans-serif;max-width:900px;margin:24px auto;padding:0 16px">'
    '<h1>Red 2 iteration 002: baseline architecture A (guarded pan)</h1><p><strong>Superseded pending bench gates.</strong> The evaluation recommends a fixed pocket and roller nip; this guarded-pan package is kept as comparator A. Receipt status: provisional (two open narrative objectives); all 14 cases pass and the displaced control is rejected.</p><ul><li><a href="package-motion.html">Package motion player</a></li>'
    '<li><a href="retention.svg">Pan retention: loaded pose and power-off escape control</a></li></ul>'
    '<img src="retention.svg" alt="Pan retention figure" style="max-width:100%"></body>\n', encoding="utf-8")
print(sorted(p.name for p in OUT.iterdir()))
