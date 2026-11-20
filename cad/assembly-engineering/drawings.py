"""Generate printable engineering traveler sheets from exact fitted STEP edges, in mm."""

import json, math
from pathlib import Path
import pymupdf as pdf
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.GCPnts import GCPnts_QuasiUniformDeflection
from OCP.HLRAlgo import HLRAlgo_Projector
from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Ax2, gp_Pnt, gp_Dir
from attachments import base
from installed_audit import HERE, sha
from cold_issue import precise

PLATES = [
    (
        "SW-01",
        "Module saddle and tab screws",
        ["FL-A-tab-saddle", "FL-A-tab-screw-57.15", "FL-A-tab-locknut-57.15"],
        [
            "Machine 8 saddles from 6061-T6. Each gets two opposed pairs of integral raised rings, OD11/ID5.30 x 0.05 high.",
            "Bench datum: lower tab top Z69.850, upper tab underside Z120.650. Match both receiving tab faces without forcing the module.",
            "16 screws: Bolt Depot 8232, 10-32 x 2-3/4. 16 prevailing nuts: 2623. Assembled minimum extension above nut: 3.675 under the receiving limits.",
            "Support the cartridge; remove bumpers and nearest deck tie; lower 195 for bench access. The 16 on-robot wrench sweeps are obstructed.",
        ],
    ),
    (
        "CH-02",
        "Through-bolted jack foot",
        [
            "Left rail saddle",
            "L-270-sleeve",
            "L-270-screw",
            "L-270-washer",
            "L-270-nut",
        ],
        [
            "Source stack shown at X270/Y341. Repeat X317 and mirror both to Y-341. Maintain chassis rail alignment on a fixture.",
            "Rail holes: 10.10 through both walls. Sleeve: 9.98 OD / 6.60 ID x 99.95 long. Saddle: 6.60 through, 13.45 x 90 degree seat.",
            "M6 x 120 class 10.9 countersunk screw. 4 mm key with 4.8 OD shank; 10 AF socket with OD15 below. Verify torque and rail crushing.",
            "A 0.01 mm downward head shift intersects the receiving cone; a 0.15 mm raise leaves measurable daylight.",
        ],
    ),
    (
        "EL-03",
        "Electrical platform and controller",
        [
            "front-electronics-platform",
            "roborio-0-0-backing-spacer",
            "roborio-0-0-washer",
            "roborio-0-0-4-40-screw",
        ],
        [
            "Platform: 3 mm 6061-T6 sheet; retain 1.40 pocket skin and the 16-wide rails at the controller inserts. CNC contours in the companion STEP.",
            "roboRIO centers: X194.20 and 295.80; Y-183 and -56, relative to robot origin. Four 3.40 clearance holes.",
            "Below skin: 1.60 spacer with OD10 and 0.40 washer. Four 4-40 x 1/4 socket screws from underside. Nominal insert penetration 2.95.",
            "Fit four M3 countersunk base screws and support posts before device installation. Check actual case landing, connector clearance, grounding and harness strain.",
        ],
    ),
    (
        "EL-04",
        "PDH bridge and main deck",
        [
            "distribution-bridge",
            "pdh-base-0-0-M4-flush-screw",
            "pdh-base-0-0-M4-bottom-washer",
            "pdh-base-0-0-M4-locknut",
        ],
        [
            "Install four bridge ties at X-275/-129, Y116/199. Machine 4.50 through, countersink 8.30 x 90 degrees at the upper face.",
            "Top from Z161; host underside at Z150. Fit M4 flat screws, finished length 18.20 including head, and M4 washer/prevailing nut beneath deck.",
            "PDH housing centers X-320.2375/-104.3375; Y104.7625/206.3625. Four factory 5.00 bores in 6.35 mm thick ears.",
            "Install four M4 x 16 low-head screws, heads below bridge, 0.8 washers and M4 nuts above tabs. Use 7 AF socket OD9.4 max; OD10 hits the case.",
        ],
    ),
    (
        "PW-05",
        "Removable battery restraint",
        [
            "battery-hold-down",
            "battery-front-tie-rod",
            "battery-front-upper-washer",
            "battery-front-upper-locknut",
        ],
        [
            "Battery gauge: 76 x 181 x 167. Secure end and side stops to tray and deck with eight positively retained M4 screws.",
            "2 x M5 threaded rods from 200 stock, finish 193.8. Tie axes: X-212, Y-116 / +86. Hold-down bar 5 thick; add top and bottom washers/nuts.",
            "Insulate terminal area. Disconnect and retain the power lead before lifting. Support the removed 76 x 181 battery.",
            "Remove upper nuts/washers and bar. Vertical 172 mm shell-only clearance is separate from terminal and cable service.",
        ],
    ),
    (
        "RC-06",
        "Stowed receiver and shock lands",
        [
            "support-machined-bridge",
            "receiver-absorber-0-KM8-lower",
            "receiver-absorber-0-KM8-upper",
        ],
        [
            "Two 8.30 bores at X118, Y-66 and +42. Fit ACE MA30EUM and four KM8 locknuts, two per installed unit.",
            "KM8: M8x1, 10 AF, 3 thick. Lower bearing z433, upper z438. Hold one nut and torque the other; initially set 0.10 free plunger gap.",
            "Each nut retains 32.496 square mm pressure witness. Two 2.6-thick open jaw gauges enter horizontally; blind socket install is unnecessary.",
            "Require loaded relative tray/bridge displacement; 18 underside pockets reduce midspan stiffness. Prior stationary absorption cases do not qualify this bridge.",
        ],
    ),
    (
        "CL-07",
        "Captive shoe guide and stem",
        [
            "Shoe dovetail carrier",
            "Positive bearing shoe",
            "Positive bearing shoe stop pin",
            "Positive bearing shoe M2.5 stop locknut",
            "Stem antirotation spring pin",
        ],
        [
            "Machine two 11.60 x 12.10 through guide tunnels along Y, X293/307. Add R0.50 corner reliefs; grind shoe section 11.50 x 12.00.",
            "Slide shoes 36 from retracted. Pin axes X293/Y+12 and X307/Y-12; shoulder 3.00 x 25, head OD4.8 and M2.5 tip with locking nut.",
            "Stem M12x1.5 shoulder at Z459, engage 7 into the guide body, transverse 2.00 hole at Z462.35. Fit spring pin to arrest loosening.",
            "Nominal guide roof overlap 22 at full travel. Gauge finished backlash and debris clearance. Position actuators and closure witness remain under development.",
        ],
    ),
    (
        "CL-08",
        "Twin jack screw and catch",
        [
            "Left screw core",
            "Left ratchet wheel",
            "Left pawl",
            "Left pawl pivot",
            "Left over AXK1024 bearing",
            "Left shoulder thrust disk",
        ],
        [
            "Two Roton 59285 5/8-8 RH screws; 90234 bronze nuts. Lead 3.175. Turn journals 9.98 from the 11.6078 mm minor diameter.",
            "Opposed AXK1024 thrust elements with matching hardened washers and shaft shoulder. Rotor and planetary output deliver torque only.",
            "Fit REV NEO 550/REV-41-1607, pinion 1608, REV-41-1603 stage, REV-41-1604 output. Align pilot and six original M3 mount positions.",
            "24-tooth wheel limits nominal reverse travel to 0.1323 mm. Proof-load housing, shaft and pawl; release requires unloading plus positive independent acknowledgement.",
        ],
    ),
]
VIEWS = [
    ("TOP", ((0, 0, 1), (1, 0, 0))),
    ("FRONT", ((0, -1, 0), (1, 0, 0))),
    ("ISOMETRIC", ((1, -1, 0.8), (1, 1, 0))),
]


def curves(shapes, direction, right):
    algo = HLRBRep_Algo()
    for s in shapes:
        algo.Add(s)
    algo.Projector(
        HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(*direction), gp_Dir(*right)))
    )
    algo.Update()
    algo.Hide()
    drawn = HLRBRep_HLRToShape(algo)
    results = []
    for output in (drawn.VCompound(), drawn.OutLineVCompound()):
        if output.IsNull():
            continue
        it = TopExp_Explorer(output, TopAbs_EDGE)
        while it.More():
            c = BRepAdaptor_Curve(TopoDS.Edge(it.Current()))
            it.Next()
            sample = GCPnts_QuasiUniformDeflection(c, 0.08)
            if sample.IsDone():
                pts = [sample.Value(i) for i in range(1, sample.NbPoints() + 1)]
                if len(pts) > 1:
                    results.append([(p.X(), p.Y()) for p in pts])
    return results


def sheet(doc, title, items, text, source, idx, code):
    page = doc.new_page(width=842, height=595)

    def label(x, y, t, size=10, bold=False, color=(0.06, 0.12, 0.19)):
        page.insert_text(
            (x, y), t, fontsize=size, fontname="hebo" if bold else "helv", color=color
        )

    label(32, 36, title, 22, True)
    label(
        32, 54, "TEAM 1780   /   CAPTIVE SWERVE + FRONT SHALLOW JACK   /   UNITS mm", 9
    )
    label(720, 38, code, 17, True)
    for col, (view, (axis, right)) in enumerate(VIEWS):
        x = 32 + col * 263
        y = 85
        paths = curves(items, axis, right)
        pts = [v for path in paths for v in path]
        xmin, ymin = min(p[0] for p in pts), min(p[1] for p in pts)
        xmax, ymax = max(p[0] for p in pts), max(p[1] for p in pts)
        scale = min(245 / (xmax - xmin), 210 / (ymax - ymin))
        off = (
            x + (245 - (xmax - xmin) * scale) / 2,
            y + (210 - (ymax - ymin) * scale) / 2,
        )
        shape = page.new_shape()
        for path in paths:
            shape.draw_polyline(
                [
                    (off[0] + (px - xmin) * scale, off[1] + (ymax - py) * scale)
                    for px, py in path
                ]
            )
        shape.finish(color=(0.09, 0.22, 0.30), width=0.6)
        shape.commit()
        label(x, 319, view, 9, True)
        if col < 2:
            label(
                x,
                335,
                f"Projected extremes  {xmax - xmin:.2f}  x  {ymax - ymin:.2f}",
                9,
            )
    page.draw_line((32, 351), (810, 351), color=(0.35, 0.48, 0.56), width=0.6)
    for j, line in enumerate(text):
        result = page.insert_textbox(
            pdf.Rect(32, 366 + j * 36, 808, 402 + j * 36),
            line,
            fontsize=10.5,
            fontname="helv",
            color=(0.09, 0.14, 0.21),
        )
        if result < 0:
            raise ValueError((code, j, "annotation overflow"))
    label(
        32,
        539,
        "Mating dimensions and names govern. Do not scale the page. Read the STEP and sequence together.",
        10,
        True,
    )
    label(
        32,
        565,
        f"Selected STEP {source[:16]}   /   Parked pose   /   Sheet {idx + 1} of {len(PLATES)}",
        9,
    )
    label(610, 565, "DESIGN REVIEW   /   SIMULATION", 9, True)


def build():
    folder = HERE / "issue"
    step = folder / "station-and-jack-interfaces.step"
    parts = base.named(step)
    digest = sha(step)
    doc = pdf.open()
    for idx, (code, name, items, notes) in enumerate(PLATES):
        sheet(doc, name, [parts[k] for k in items], notes, digest, idx, code)
    doc.set_metadata(
        {
            "title": "Captive swerve engineering assembly drawings",
            "author": "Team 1780 design study",
        }
    )
    (folder / "assembly-drawings.pdf").write_bytes(
        doc.tobytes(no_new_id=True, deflate=True, garbage=4)
    )


if __name__ == "__main__":
    build()
