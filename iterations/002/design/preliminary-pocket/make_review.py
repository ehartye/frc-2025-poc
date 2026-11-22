"""Assemble the LAN review bundle: the two generated players, a retention figure and an index. No STEP, F3D or CAD source.

    python -I -B make_review.py PROJECT_ROOT
"""
import hashlib
import html
import json
import math
import shutil
import sys
from pathlib import Path

ROOT = Path(sys.argv[1]).resolve()
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pocket_geometry as pg  # noqa: E402

PRE = ROOT / "iterations/002/design/preliminary-pocket"
OUT = PRE / "review"
OUT.mkdir(exist_ok=False)
for name in ("package-motion.html", "field-interaction.html"):
    shutil.copyfile(PRE / name, OUT / name)
receipt = json.loads((PRE / "evidence" / "preliminary-check.json").read_text(encoding="utf-8"))
ret = json.loads((PRE / "retention-review.json").read_text(encoding="utf-8"))
find = json.loads((PRE / "design-findings.json").read_text(encoding="utf-8"))
gates = json.loads((PRE / "bench-gates.json").read_text(encoding="utf-8"))
rul = json.loads((PRE / "rules-rulings.json").read_text(encoding="utf-8"))
blocks = json.loads((PRE / "layout-blocks.json").read_text(encoding="utf-8"))
seat = blocks["poses"]["seatMm"]

# ---- retention figure: section at the seat with the push-back contacts, and the power-off escape control
W, H, s = 900, 430, 0.9
def panel(ox, title, note, flap=True, arrow=None):
    tx = lambda x: ox + 20 + (x - 60) * s * 0.75
    tz = lambda z: H - 40 - (z - 470) * s * 0.75
    out = [f'<text x="{ox + 20}" y="24" font-size="15" font-weight="bold">{html.escape(title)}</text>',
           f'<text x="{ox + 20}" y="42" font-size="12">{html.escape(note)}</text>']
    pts = " ".join(f"{tx(x):.1f},{tz(z):.1f}" for x, z in pg.INLET if x > 40)
    out.append(f'<polyline points="{tx(40):.1f},{tz(pg.floor_z(40)):.1f} {pts}" fill="none" stroke="#1a5fd0" stroke-width="3"/>')
    out.append(f'<polyline points="{" ".join(f"{tx(x):.1f},{tz(z):.1f}" for x, z in pg.HOOD)}" fill="none" stroke="#1a5fd0" stroke-width="2"/>')
    if flap:
        h = pg.FLAP_HINGE
        out.append(f'<line x1="{tx(h[0]):.1f}" y1="{tz(h[1]):.1f}" x2="{tx(h[0]):.1f}" y2="{tz(h[1] - pg.FLAP_LENGTH):.1f}" stroke="#f2b705" stroke-width="4"/>')
    for c in (pg.LOWER_ROLLER, pg.UPPER_ROLLER):
        out.append(f'<circle cx="{tx(c[0]):.1f}" cy="{tz(c[1]):.1f}" r="{pg.ROLLER_R * s * 0.75:.1f}" fill="none" stroke="#1a5fd0" stroke-dasharray="5 4"/>')
        out.append(f'<circle cx="{tx(c[0]):.1f}" cy="{tz(c[1]):.1f}" r="{pg.CORE_R * s * 0.75:.1f}" fill="#1a5fd0" fill-opacity=".25" stroke="#1a5fd0"/>')
    out.append(f'<circle cx="{tx(seat[0]):.1f}" cy="{tz(seat[1]):.1f}" r="{pg.R * s * 0.75:.1f}" fill="#e6b450" fill-opacity=".7" stroke="#7a5a10"/>')
    out.append(f'<line x1="{tx(seat[0]):.1f}" y1="{tz(seat[1]):.1f}" x2="{tx(seat[0]):.1f}" y2="{tz(seat[1]) + 45:.1f}" stroke="#222" marker-end="url(#a)"/>')
    out.append(f'<text x="{tx(seat[0]) + 5:.1f}" y="{tz(seat[1]) + 45:.1f}" font-size="11">g</text>')
    if arrow:
        out.append(f'<line x1="{tx(seat[0]):.1f}" y1="{tz(seat[1]):.1f}" x2="{tx(seat[0] + arrow[0]):.1f}" y2="{tz(seat[1] + arrow[1]):.1f}" '
                   'stroke="#d55e00" stroke-width="2.5" stroke-dasharray="6 4" marker-end="url(#a)"/>')
    return out


parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H + 60}" font-family="sans-serif" viewBox="0 0 {W} {H + 60}">',
         '<defs><marker id="a" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path d="M0,0L10,5L0,10z"/></marker></defs>',
         f'<rect width="{W}" height="{H + 60}" fill="#fff"/>']
parts += panel(0, "Loaded pose: carry seat, nip locked", f"Floor pushes the pipe toward the nip at {ret['seat']['gravityAlongFloorTowardNipG']:.3f} g; hood, treads and flap push back")
parts += panel(450, "Power-off escape control: flap removed", "Unwedged pipe, 8 g toward the inlet: lost rearward up the floor", flap=False, arrow=(-200, 56))
parts.append(f'<text x="20" y="{H + 30}" font-size="11">Side view, +X toward the reef, Z up. Nip: tread (dashed) and rigid core (filled). '
             'Source: retention-review.json and evidence/field-trials.json (sled-isolation-no-flap).</text></svg>')
(OUT / "retention.svg").write_text("\n".join(parts) + "\n", encoding="utf-8")

status = receipt["status"]
missing = receipt.get("missingInputs", [])
li = lambda items: "".join(f"<li>{html.escape(x)}</li>" for x in items)
fx = "".join(f"<li><b>{html.escape(f['id'])}</b> ({html.escape(f['status'])}): {html.escape(f['text'])}</li>" for f in find["findings"])
gx = "".join(f"<tr><td>{g['id']} {html.escape(g['name'])}</td><td>{html.escape(g['pass_'])}</td><td>{html.escape(g['kill'] or '-')}</td>"
             f"<td>{html.escape(json.dumps(g['prediction'])[:220])}</td><td>{g['status']}</td></tr>" for g in gates["gates"])
rx = "".join(f"<li><b>{html.escape(r['rule'])}</b>: {html.escape(r['verdict'])}. {html.escape(r['basis'])}</li>" for r in rul["rulings"])
(OUT / "index.html").write_text(f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Red 2 B'x review</title><style>
:root{{--blue:#0b3d91;--gold:#f2b705;--ink:#111827;--bg:#f7f8fb;--card:#fff;--line:#d6dbe6}}
@media (prefers-color-scheme:dark){{:root:not([data-theme=light]){{--ink:#e5e7eb;--bg:#0b1220;--card:#111a2e;--line:#24314f}}}}
body{{margin:0;font:14px/1.5 system-ui,sans-serif;background:var(--bg);color:var(--ink)}}header{{background:var(--blue);color:#fff;padding:14px 16px;border-bottom:4px solid var(--gold)}}
main{{max-width:1000px;margin:0 auto;padding:12px 16px}}section{{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:10px 14px;margin:10px 0}}
table{{border-collapse:collapse;width:100%;font-size:12px}}td,th{{border:1px solid var(--line);padding:4px;vertical-align:top}}img{{max-width:100%;height:auto;background:#fff}}a{{color:#1a5fd0}}
</style></head><body><header><h1 style="margin:0;font-size:19px">Red 2 iteration 002: B'x pocket preliminary layout</h1>
<div>Issue {html.escape(receipt['issue'])} | receipt status <b>{html.escape(status)}</b> | robot datum: chassis centre on the tire floor, +X toward the reef, +Y port, +Z up, mm</div></header><main>
<section><h2>Open the evidence</h2><ul><li><a href="package-motion.html">Package motion player</a>: solid blocks, cycle cases, collisions, inventory, mass and voltage budgets.</li>
<li><a href="field-interaction.html">Field-interaction playback</a>: solved receive, carry-sled and eject trials against the trough, with disabled and isolation controls.</li>
<li><a href="retention.svg">Retention: loaded pose and power-off escape control</a></li></ul>
<p>Disposition: selected for preliminary review; bench gates G1-G4 open; the r3 pan package (A) remains the fallback until they pass.</p></section>
<section><h2>Receipt</h2><p>Plan sha256 {receipt['planSha256'][:16]}..., STEP sha256 {receipt['step']['sha256'][:16]}..., design revision {receipt['designRevision']}.</p>
<p>Open items that keep it from 'layout-checked':</p><ul>{li(missing)}</ul></section>
<section><h2>Rules</h2><ul>{rx}</ul></section>
<section><h2>Findings</h2><ul>{fx}</ul></section>
<section><h2>Bench gates (open verification)</h2><table><tr><th>Gate</th><th>Pass</th><th>Kill</th><th>Model prediction</th><th>Status</th></tr>{gx}</table></section>
<section><img src="retention.svg" alt="Retention figure: seated pipe and the no-flap escape control"></section>
<p style="font-size:12px">This directory holds only review pages and their embedded data: no STEP, F3D, supplier solids or other CAD source.</p>
</main></body></html>
""", encoding="utf-8")
manifest = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(OUT.iterdir())}
(OUT / "MANIFEST.json").write_text(json.dumps(dict(issue=receipt["issue"], files=manifest), indent=1) + "\n", encoding="utf-8")
print(sorted(manifest))
