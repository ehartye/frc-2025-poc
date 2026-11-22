"""Build the iteration 002 preliminary package: labeled planar reserves in the robot datum, written as one STEP.

Run from anywhere with the CAD Python and the plugin scripts on the path:
    python -B build_layout.py PROJECT_ROOT OUTPUT_DIR PLUGIN_SCRIPTS

Every reserve is a planar polyhedron. Mated reserves are partitioned at their mating plane: a
lower-priority reserve is trimmed by the higher-priority reserve inflated by PARTITION_MM, so every
pair of distinct reserves is separated by at least that gap. The gap shell at a mating face is the
interface allowance; seated contact, fastening and fit inside it are detailed-design checks.
"""
import hashlib
import json
import math
import sys
from pathlib import Path

ROOT = Path(sys.argv[1]).resolve()
OUT = Path(sys.argv[2]).resolve()
sys.path.insert(0, str(Path(sys.argv[3]).resolve()))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from showcase_contacts import load_solids  # noqa: E402  (plugin STEP reader, same naming as the checker)
from layout_occ import (box, aabb, volume, solids, fuse, cut, common, revolved, distance, valid, loft_solid, prism_xy,  # noqa: E402
                        ngon_prism, ngon_prism_y, cylinder, nodes, obb_box, rotated, moved, offset_polygon)

G = 2.1                      # partition reserve between distinct reserves (mm), above the 2.0 mm case gaps
GM = 3.1                     # wider partition beside a moving reserve, so the interval checker certifies the 2.0 mm gap
                             # within its subdivision budget (rigid-motion speed bound times the half interval)
GT = 4.0                     # partition between each steer sweep and its module housing
PIVOT = (205.0, 0.0, 580.0)  # pan axis, +Y (pan/geometry.json)
STOW_FROM_FLAT = -70.0       # rotation about +Y from the authored flat pose (sim theta +70 deg)
PIPE_APOTHEM = 57.15         # 4.5 in OD CORAL (2025 manual; not yet registered in rules-v0.json)
PIPE_LENGTH = 301.625        # project constant (experiments/six-wheel/scripts/receiver_dock.py)
CHASSIS_STEP = "artifacts/resume-20261121/screws-place-01/four-seated-screws.step"
PAN_GEOMETRY = "iterations/002/design/pan/geometry.json"
ELECTRICAL = "iterations/002/design/chassis/inset-electrical-package.json"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pin(rel):
    return {"file": rel, "sha256": sha(ROOT / rel)}


blocks = {}      # name -> dict(shape, group, function, parent, motion, bounds, missing, priority)
order = []


def add(name, shape, **meta):
    if name in blocks:
        raise ValueError("duplicate block " + name)
    blocks[name] = dict(shape=shape, **meta)
    order.append(name)


def inflated_boxes(shapes_list):
    return [obb_box(s, G)[0] for s in shapes_list]


# ---------------------------------------------------------------- fixed chassis issue
ids = {}
chassis, non_solid = load_solids(ROOT / CHASSIS_STEP, ids)
assert len(chassis) == 658 and not non_solid, (len(chassis), len(non_solid))
structure, modules = [], {k: {"turret": [], "housing": []} for k in (1, 2, 3, 4)}
for name in chassis:
    path = ids[name]["occurrencePath"]
    if "Matched SDS L2 module" in path[1]:
        k = int(path[1].split(":")[1])
        modules[k]["turret" if any("MK4 Turret Assembly" in p for p in path) else "housing"].append(name)
    else:
        structure.append(name)
assert len(structure) == 150

struct_boxes = [obb_box(chassis[n])[0] for n in structure]
# The actual frame plan hull of the 658-solid issue. The upper perimeter chamfer tubes cut the rear corners by 42 mm
# (outer face (-370, +/-323) to (-328, +/-365)), but the lower cassette corners at (-361.375, +/-356.375) lie on
# |y| - x = 717.75, so the frame perimeter that bounds every body has a 17.25 mm rear cutback. The project record
# (pan/operating-intent rules.startPerimeter) states two 57.8 mm chamfers; neither chamfer bounds the cassettes.
CUTBACK = 370.0 + 365.0 - max(abs(y) - x for n in chassis for x, y, z in nodes(chassis[n]))
OUTLINE = [(370.0, -365.0), (370.0, 365.0), (-370.0 + CUTBACK, 365.0), (-370.0, 365.0 - CUTBACK),
           (-370.0, -365.0 + CUTBACK), (-370.0 + CUTBACK, -365.0)]
# Box conservatism at the mitred rear tube ends would otherwise reach outside the frame outline that bounds every chassis body.
S = common(fuse(struct_boxes), prism_xy(OUTLINE, -1.0, 600.0))
S_infl = fuse(inflated_boxes([chassis[n] for n in structure]))
assert len(solids(S)) == 1
# Split the fixed chassis into spatially compact reserves so the interval checker only compares nearby bodies.
# Same per-body boxes and outline clip as the single fused block; groups are partitioned at their mating faces
# in priority order (perimeter first), exactly like every other mated reserve.
CHASSIS_GROUPS = [
    ("Chassis perimeter", ("Structural perimeter", "Frame support", "Sleeve"),
     "Structural perimeter tubes, the two rear chamfer tubes, frame supports and wheel-station sleeves"),
    ("Chassis cassettes and socket rails", ("Cassette", "socket rail"),
     "Lower module cassettes and the eight socket-rail ends the module plates clamp"),
    ("Chassis pan trusses and stringers", ("Pan truss", "Upper pan stringer"),
     "Port and starboard pan trusses (members 0-5) and the upper pan stringers"),
    ("Chassis sills, decks and crossmembers", None,
     "Inboard sills, pan crossmembers, truss landings and nutbars, equipment sheets, tabs, perimeter keys, keyed pedestals "
     "and nutbars, and the four seated 91251A542 clamp screws"),
]
clip = prism_xy(OUTLINE, -1.0, 600.0)
assigned = {}
for title, keys, _ in CHASSIS_GROUPS:
    assigned[title] = [n for n in structure if (keys is None or any(k in n for k in keys))
                       and not any(n in v for v in assigned.values())]
leftover = [n for n in structure if not any(n in v for v in assigned.values())]
assert not leftover, leftover
higher = []
chassis_members = []
for title, keys, text in CHASSIS_GROUPS:
    names_ = assigned[title]
    raw = common(fuse([obb_box(chassis[n])[0] for n in names_]), clip)
    pieces = sorted(solids(cut(raw, higher)) if higher else solids(raw), key=volume, reverse=True)
    for i, piece in enumerate(pieces):
        name = title + ("" if i == 0 else f" part {i + 1}")
        chassis_members.append(name)
        add(name, piece, group="structure", role="structure",
            function=text + ("" if i == 0 else " (separated piece of the same reserve)"),
            parent="fixed (robot datum)", motion="fixed",
            bounds=f"Union of per-body boxes of {len(names_)} chassis bodies in " + CHASSIS_STEP +
                   " clipped to the frame plan hull and partitioned 2.1 mm from the higher-priority chassis reserves; holes and pockets filled",
            missing=["Bolted and riveted joints between chassis reserves lie inside the 2.1 mm partition shells"])
    higher.append(fuse(inflated_boxes([chassis[n] for n in names_])))

n32 = math.cos(math.pi / 32)
turret_bands = {}
for k, mod in modules.items():
    ring = [aabb(chassis[n]) for n in mod["turret"] if "A035XPO" in "".join(ids[n]["occurrencePath"])]
    assert len(ring) == 1
    cx, cy = (ring[0][0] + ring[0][3]) / 2, (ring[0][1] + ring[0][4]) / 2
    pts = [p for n in mod["turret"] for p in nodes(chassis[n])]
    ztop = max(p[2] for p in pts)
    cuts = [(0.0, 100.0), (100.0, 120.0), (120.0, ztop)]
    bands = []
    for z0, z1 in cuts:
        r = max(math.hypot(x - cx, y - cy) for x, y, z in pts if z0 - 1e-6 <= z <= z1 + 1e-6)
        bands.append((z0, z1, r + 0.3))
    turret_bands[k] = dict(axis=[cx, cy], bands=bands)
    # Representative turret: the wheel as its own cylinder on its axle, plus coaxial steer cylinders above the tread
    # for the wheel mounts, steering pulley and turret gears. The housing is partitioned from the full steer sweep.
    wheel = [p for n in mod["turret"] if "MK4 Billet Wheel Assembly" in "".join(ids[n]["occurrencePath"]) for p in nodes(chassis[n])]
    other = [p for n in mod["turret"] if "MK4 Billet Wheel Assembly" not in "".join(ids[n]["occurrencePath"]) for p in nodes(chassis[n])]
    wz0, wz1 = min(p[2] for p in wheel), max(p[2] for p in wheel)
    best = None
    for ang in [math.radians(a10 / 10) for a10 in range(0, 1800, 5)]:
        dx, dy = math.cos(ang), math.sin(ang)
        span = [(x - cx) * dx + (y - cy) * dy for x, y, z in wheel]
        if best is None or max(span) - min(span) < best[0]:
            best = (max(span) - min(span), ang, min(span), max(span))
    _, ang, s0, s1 = best
    dx, dy = math.cos(ang), math.sin(ang); wzc = (wz0 + wz1) / 2
    wc = (cx + dx * (s0 + s1) / 2, cy + dy * (s0 + s1) / 2)
    wr = max(math.hypot(-(x - wc[0]) * dy + (y - wc[1]) * dx, z - wzc) for x, y, z in wheel) + 0.2
    zm = min(p[2] for p in other)
    # True steer envelope: revolve the turret's radial profile 360 deg about the steer axis. The radius of every
    # 4 mm slab is the largest meshed-node distance of any turret constituent in that slab, plus 0.5 mm.
    ztop_t = max(p[2] for p in pts)
    slabs = []
    z = 0.0
    while z < ztop_t - 1e-9:
        z1 = min(z + 4.0, ztop_t)
        r = max((math.hypot(x - cx, y - cy) for x, y, zz in pts if z - 1e-6 <= zz <= z1 + 1e-6), default=0.0)
        slabs.append([z, z1, r + 0.5])
        z = z1
    for i in range(len(slabs)):           # no slab narrower than a neighbour's shared face would leave a pocket below a lip
        slabs[i][2] = max(slabs[i][2], 1.0)
    T = revolved((cx, cy), [tuple(sl) for sl in slabs])
    T_infl = revolved((cx, cy), [(-GT, 0.0, slabs[0][2] + GT)] + [(a_, b_, r_ + GT) for a_, b_, r_ in slabs] + [(ztop_t, ztop_t + GT, slabs[-1][2] + GT)])
    T_infl = fuse([T_infl] + [revolved((cx, cy), [(max(a_ - GT, -GT), b_ + GT, r_ + GT)]) for a_, b_, r_ in slabs])
    reach = max(sl[2] for sl in slabs)
    turret_bands[k].update(wheel=dict(centreMm=[wc[0], wc[1], wzc], axle=[dx, dy, 0.0], radiusMm=wr, widthMm=s1 - s0 + 0.4),
                           revolvedProfile=slabs, envelopeRadiusMm=reach)
    corner = {1: "front-left", 2: "front-right", 3: "rear-right", 4: "rear-left"}[k]
    add(f"Module {k} steer sweep", T, group="steer sweeps", role="drive",
        function=f"{corner} SDS MK4i L2 turret: wheel, wheel mounts, steering pulley and turret gears, full 360 deg steer swept volume",
        parent=f"static envelope about the module {k} steer axis ({cx:.2f}, {cy:.2f}) mm, +Z; covers every azimuth",
        motion="none (the revolved envelope already contains every steer angle)",
        bounds=f"Solid of revolution of the 43 turret constituents' radial profile in 4 mm slabs (max meshed radius + 0.5 mm, "
               f"largest {reach:.1f} mm) about the steer axis, Z 0-{ztop_t:.1f} mm",
        missing=["Loaded tire radius and tread compression (factory tire floor Z = 0)",
                 "Module internal gear mesh and belt wrap are supplier interfaces inside the partition shell"])
    H = fuse([obb_box(chassis[n])[0] for n in mod["housing"]])
    pieces = sorted(solids(cut(H, [T_infl, S_infl])), key=volume, reverse=True)
    for i, piece in enumerate(pieces):
        add(f"Module {k} housing" + ("" if i == 0 else f" part {i + 1}"), piece, group="module housings", role="drive",
            function=f"{corner} module fixed housing: plates, drive and steer NEOs, CANcoder, gears, belt and 10-32 frame screws"
                     + ("" if i == 0 else " (separated piece of the same housing after partition)"),
            parent="fixed: clamped to the chassis cassette and socket rails", motion="fixed",
            bounds="Per-body boxes of the 84 non-turret module solids, partitioned from the chassis and the steer sweep",
            missing=["Frame-screw seating and clamp are inside the 2.1 mm partition shell; verified separately by the cassette checks"])

# ---------------------------------------------------------------- pan package r3 (proposal) from its own sections
geometry = json.loads((ROOT / PAN_GEOMETRY).read_text(encoding="utf-8"))
parts = {p["id"]: p for p in geometry["parts"]}
solid_of = {k: loft_solid(*p["sections"]) for k, p in parts.items()}
ROTATING_SUPPLIER = {"Idler headed stub hex shank", "Port reverse-limit cam"}
pan_ids = [k for k, p in parts.items() if p["group"] == "Guarded pan" and k not in ROTATING_SUPPLIER]
P_flat = fuse([solid_of[k] for k in pan_ids])
P_stow = rotated(P_flat, PIVOT, (0, 1, 0), STOW_FROM_FLAT)
assert len(solids(P_stow)) == 1

support_ids = [k for k, p in parts.items() if p["group"] == "Pan upper support"]
UPPER_KEYS = ("stringer extension", "stringer splice", "inlet truss", "Truss node plug")
upper_ids = [k for k in support_ids if any(s in k for s in UPPER_KEYS)]
cassette_ids = [k for k in support_ids if k not in upper_ids]
inlet_ids = [k for k, p in parts.items() if p["group"] == "Fixed inlet"]
panel_ids = [k for k in inlet_ids if k.startswith("Fixed feed panel")]
guide_ids = [k for k in inlet_ids if k not in panel_ids]

U = fuse([solid_of[k] for k in upper_ids])
U = sorted(solids(cut(U, [S_infl])), key=volume, reverse=True)
U_infl = fuse(inflated_boxes([solid_of[k] for k in upper_ids]))
U_infl_m = fuse([obb_box(solid_of[k], GM)[0] for k in upper_ids])
S_infl_m = fuse([obb_box(chassis[n], GM)[0] for n in structure])
for i, piece in enumerate(U):
    add("Upper support" + ("" if i == 0 else f" part {i + 1}"), piece, group="upper support", role="structure",
        function="Fixed upper support: stringer extensions and splices to Y = +/-242 mm, inlet truss legs and landing pads"
                 + ("" if i == 0 else " (separated piece)"),
        parent="fixed: spliced to the upper pan stringers at X 100.475 and landed on structural perimeter 0, 2 and 4",
        motion="fixed",
        bounds="Outer section prisms of the r3 'Pan upper support' stringer, splice and inlet-truss parts; truss node plugs fall inside the chassis truss reserve",
        missing=["Splice rows, plugged butt nodes and landing-pad fasteners are inside the partition shell (pan README open item 6)"])

# rotating supplier volumes and stop interfaces folded into the removable cassette as swept reserves
px, _, pz = PIVOT
sq = lambda h, y0, y1: cylinder((px, y0, pz), (0, 1, 0), h, y1 - y0)   # coaxial cylinder on the pan axis
extra = {
    "port hex journal": sq(7.4, 188.0 + GM, 191.85),
    "port REV-21-1915 bearing": sq(14.2875 + 0.1, 191.85, 198.15),
    "starboard REV-21-1915 bearing": sq(14.2875 + 0.1, -198.15, -191.85),
    "port limit cam sweep, collar and switch": fuse([sq(31.0, 198.15, 212.0), box([195, 198.15, 535], [215, 212.0, 557])]),
    "pan gearbox and NEO 550 envelope": sq(32.0, 216.0, 350.0),
    "starboard idler journal": sq(7.4, -191.85, -188.0 - GM),
    "starboard thrust, spring and collar stack": sq(28.0, -229.5, -198.15),
}
# stow stop pads: pad plus shims on the backing top face, partitioned from the cheek lug at stow
pads = {}
for side, s in (("Port", 1), ("Starboard", -1)):
    back = parts[f"{side} stow stop pad backing"]["sections"][0]
    p0, p1 = back[0], back[1]                      # top edge (X, Z) of the backing profile
    nrm = (0.2397140055132868, 0.9708435484468008)  # operating-intent padPushDirection
    ylo, yhi = sorted((138.5 * s, 160.5 * s))
    thick = 6.5 - G - 0.05
    while True:
        a = [p0, p1, [p1[0] + nrm[0] * thick, 0, p1[2] + nrm[1] * thick], [p0[0] + nrm[0] * thick, 0, p0[2] + nrm[1] * thick]]
        pad = loft_solid([(q[0], ylo, q[2]) for q in a], [(q[0], yhi, q[2]) for q in a])
        if distance(pad, P_stow) >= GM + 0.02:
            break
        thick -= 0.05
    pads[side] = (pad, thick)
    extra[f"{side.lower()} stow stop pad (partitioned)"] = pad
for side, s in (("Port", 1), ("Starboard", -1)):
    capsec = parts[f"{side} soft stop cap"]["sections"][0]
    p0, p1 = capsec[3], capsec[2]                    # sloped cap top edge (X, Z)
    nrm = (0.08715574274765817, 0.9961946980917455)  # geometry.json softStopGeometry padPushDirection
    ylo, yhi = sorted((168.0 * s, 187.05 * s))
    a = [p0, p1, [p1[0] + nrm[0] * 18.5, 0, p1[2] + nrm[1] * 18.5], [p0[0] + nrm[0] * 18.5, 0, p0[2] + nrm[1] * 18.5]]
    extra[f"{side.lower()} soft stop pad"] = loft_solid([(q[0], ylo, q[2]) for q in a], [(q[0], yhi, q[2]) for q in a])
C = fuse([solid_of[k] for k in cassette_ids] + list(extra.values()))
C = sorted(solids(cut(C, [S_infl_m, U_infl_m])), key=volume, reverse=True)
assert len(C) == 1, [volume(c) for c in C]
C_infl = fuse(inflated_boxes([solid_of[k] for k in cassette_ids] + list(extra.values())))
add("Pan cassette", C[0], group="pan cassette", role="mechanism",
    function="Removable cassette: X110 and X250 cradles, end clips, bearing carriers, gearbox plate, landing clips, "
             "starboard knee struts, stow-stop brackets and pads, soft-stop beams and pads, pan gearbox and NEO 550, "
             "limit cam sweep and switch, both journal stacks",
    parent="fixed in operation: cradle end clips to the upper pan stringers; extracted with the pan for service",
    motion="fixed (service: vertical withdrawal)",
    bounds="Outer section prisms of the r3 cassette parts plus coaxial cylinder journal, bearing, cam-sweep, spring-stack and gearbox reserves about the pan axis; "
           f"stow pad reserve {pads['Port'][1]:.2f} mm of the 6.5 mm pad and shim stack",
    missing=["UltraPlanetary stage stack length and NEO 550 length are unsourced (gearbox reserve ends at Y 350)",
             "Stow pad seating against the cheek lug at 70 deg is the qualified stop interface (pan-stop-study-05), represented by the partition",
             "Clutch envelope and the drive hex inside the 2.1 mm journal partition"])

add("Guarded pan", P_stow, group="pan", role="mechanism",
    function="Rotating guarded pan r3: 430 x 322 mm floor, slotted rear lips, cheeks, stop lugs, socket posts, bulkhead and clips",
    parent=f"moving: revolute about the pan axis through ({px}, 0, {pz}) mm, +Y; parent Pan cassette bearings",
    motion="stow 70 deg (installed pose) to dump -2 deg",
    bounds="Outer section prisms of the 52 r3 'Guarded pan' parts (bores, windows and lightening filled), authored flat and rotated 70 deg; "
           "the idler stub and limit cam are swept reserves in the cassette",
    missing=["Moving pan joining hardware heads (rivets, #10-32) are inside section bounds only where flush; protruding heads unbounded",
             "Soft-stop and fault excursions below -2 deg (to -8.5 deg) are outside the operating cycle"])

xmember_ids = [k for k in guide_ids if "crossmember" in k]
guide_ids = [k for k in guide_ids if k not in xmember_ids]
I = fuse([solid_of[k] for k in guide_ids])
I = sorted(solids(cut(I, [S_infl, U_infl, C_infl])), key=volume, reverse=True)
I_infl = fuse(inflated_boxes([solid_of[k] for k in guide_ids]))
# The chute crossmembers sit 1.6 mm under the guides with the panel edge sandwiched between; partition them from the guides.
X = sorted(solids(cut(fuse([solid_of[k] for k in xmember_ids]), [I_infl, S_infl, U_infl, C_infl])), key=volume, reverse=True)
for i, piece in enumerate(X):
    add(f"Inlet chute crossmember {i + 1}", piece, group="inlet", role="mechanism",
        function="Fixed chute crossmember under the feed-panel joint", parent="fixed: clipped to the inlet guides",
        motion="fixed", bounds="Outer section prism of the r3 crossmember, trimmed 0.5 mm under the guide-panel sandwich",
        missing=["Crossmember end clips to the guides are not modelled in r3"])
I_infl = fuse([I_infl] + inflated_boxes([solid_of[k] for k in xmember_ids]))
for i, piece in enumerate(I):
    add("Inlet guides and frame" + ("" if i == 0 else f" part {i + 1}"), piece, group="inlet", role="mechanism",
        function="Fixed inlet side guides, upper guide spines, posts and mounting feet" + ("" if i == 0 else " (separated piece)"),
        parent="fixed: feet bolted to the upper stringer extensions", motion="fixed",
        bounds="Outer section prisms of the r3 'Fixed inlet' guide, spine, post and foot parts",
        missing=["Inlet portal end clips and splice rows (pan README open item 6)",
                 "P3 inlet-post reduction (3/4 x 3/4 x 1/16 in) is a proposal; this reserve is the current r3 post"])
Pn = fuse([solid_of[k] for k in panel_ids])
Pn = sorted(solids(cut(Pn, [I_infl, C_infl, U_infl, S_infl])), key=volume, reverse=True)
for i, piece in enumerate(Pn):
    add("Inlet chute panels" + ("" if i == 0 else f" part {i + 1}"), piece, group="inlet", role="mechanism",
        function="Fixed feed panels 1-3: the station chute surface the pipe rolls down into the stowed pan",
        parent="fixed: carried by the inlet frame", motion="fixed",
        bounds="Outer loft prisms of the three r3 feed panels", missing=["Brush or foam damping liner (fixed-path study) is not part of the pan architecture"])

# ---------------------------------------------------------------- bumpers (project bumper record)
inner, outer = offset_polygon(OUTLINE, G), offset_polygon(OUTLINE, 75.5)
ring = cut(prism_xy(outer, 43.0, 181.0), [prism_xy(inner, 42.0, 182.0)])
add("Bumper ring", solids(ring)[0], group="bumpers", role="bumper",
    function="Continuous bumper backing, foam and cover around the chamfered frame, both alliance covers",
    parent="fixed: removable bumper on the frame perimeter", motion="fixed",
    bounds=f"Mitred normal offsets of the actual 740 x 730 mm frame plan hull ({CUTBACK:.2f} mm rear cutback set by the lower cassette corners): 1 mm gap, 4 mm backer, 69.5 mm foam, 0.5 mm cover "
           "(cad/rear-frame-facets/construction.json), Z 43-181 mm; inner face partitioned at 2.1 mm",
    missing=["002 bumper construction, mounts and the applicable R4xx edition check are not issued; the 75 mm stack is carried from the project's rear-facet record",
             "A bumper following the 42 mm upper chamfer (or the recorded 57.8 mm chamfer) would intersect the lower cassette corners; trim the cassettes or keep this outline",
             "Front-strike crush (18 mm triangular stop at 1.8 m/s) not represented"])

# ---------------------------------------------------------------- electronics (552-r5 prospective placements)
elec = json.loads((ROOT / ELECTRICAL).read_text(encoding="utf-8"))["candidatePlacements"]
names = {"Power distribution": "Power distribution hub", "Control computer": "roboRIO 2", "Radio": "VH-109 radio",
         "Main 120 A breaker": "Main 120 A breaker"}
for key, row in elec.items():
    if key in ("Chassis battery plug", "Battery plug"):
        continue
    lo, hi = row["boundsMm"][:3], row["boundsMm"][3:]
    nm = names.get(key, key[0].upper() + key[1:] + " (SPARK MAX)" if "controller" in key else key)
    add(nm, box(lo, hi), group="electronics", role="other",
        function=f"{row['source']} at its 552-r5 prospective placement on the equipment sheet",
        parent="fixed: on standoffs on the equipment sheet (Z 205.2)", motion="fixed",
        bounds="World bounds of the mounted supplier solids (" + row["source"] + "); the breaker source is 16 open shells",
        missing=["Device brackets and standoffs between Z 205.2 and the device (deck tabs reach Z 208.1)",
                 "Connector and lead bend-radius envelopes at each device"])
plug = elec["Chassis battery plug"]["boundsMm"]
cp = fuse([box([plug[0], 80.1, plug[2]], [plug[3], plug[4], plug[5]]),
           box([-331.9 + G, 80.1, 180.0], [plug[0], plug[4], 196.0])])
add("Chassis battery plug and bracket", cp, group="electronics", role="other",
    function="Robot-side SB50 (mates the battery plug) and its bracket to the rear perimeter",
    parent="fixed: bracket to structural perimeter 2", motion="fixed",
    bounds="SB50 supplier bounds partitioned at the mating plane Y 78 (3.1 mm beside the moving battery plug); bracket reserve X -329.8 to -317",
    missing=["Bracket design and fasteners", "Mating engagement (0 mm short-mate per inset-electrical-package) is inside the partition shell"])

# ---------------------------------------------------------------- wiring reserves
add("Main power loom", fuse([box([-317.0, 121.1, 180.0], [-281.0, 140.0, 222.0]),
                             box([-281.0, 121.1, 198.1], [-270.2, 229.9, 222.0]),
                             box([-270.2, 210.5, 210.2], [-122.1, 234.9, 222.0])]),
    group="wiring", role="other",
    function="6 AWG main leads: chassis SB50 riser, rear run and run to the 120 A breaker and PDH",
    parent="fixed: tied to the rear frame and the equipment sheet", motion="fixed",
    bounds="Rectangular 15.9 x 18.9 to 24.4 x 11.8 mm channels clear of the battery lift, plug grip and inlet truss legs",
    missing=["Lead bend radius at the breaker and PDH terminals", "Strain relief and tie points"])
trunk = fuse([box([-28.9, -20.0, 210.2], [240.0, 20.0, 226.0]),
              box([20.0, -96.6, 210.2], [56.0, 96.6, 226.0]),
              box([180.0, -96.6, 210.2], [216.0, 96.6, 226.0])])
add("Branch and CAN trunk", trunk, group="wiring", role="other",
    function="PDH branch and CAN trunk on the centerline with cross runs to the inner drive and steer controllers",
    parent="fixed: supported from the equipment sheets across the centre gap", motion="fixed",
    bounds="15.8 mm high channels at Z 210.2-226 crossing the centre gap above the pan crossmembers",
    missing=["Runs to the outer controllers and the pan controller cross the pan-truss members; route undesigned",
             "CAN termination, sensor leads to the limit switch and CANcoders"])
for k, (sx, sy) in {1: (1, 1), 2: (1, -1), 3: (-1, -1), 4: (-1, 1)}.items():
    x0, x1 = sorted((150.0 * sx, 205.0 * sx)); y0, y1 = sorted((145.0 * sy, 200.0 * sy))
    add(f"Module {k} lead reserve", box([x0, y0, 174.4], [x1, y1, 201.1]), group="wiring", role="other",
        function=f"Drive, steer and CANcoder leads leaving module {k} under the equipment sheet",
        parent="fixed: between the module housing and the sheet", motion="fixed",
        bounds="55 x 55 x 26.7 mm reserve over the inboard motor ends", missing=["Sheet pass-through and grommet"])

# ---------------------------------------------------------------- battery and retention
tray = fuse([box([-250.0, -100.5, 21.0], [-50.0, 100.5, 29.9]),
             *[box([x0, s0, 29.9], [x0 + 20.0, s1, 109.0]) for x0 in (-220.0, -100.0) for s0, s1 in ((60.0, 100.5), (-100.5, -60.0))],
             box([-248.7, -30.0, 29.9], [-244.6, 30.0, 100.0]), box([-55.4, -30.0, 29.9], [-51.3, 30.0, 100.0])])
# Reserves beside the withdrawn battery keep the wider 3.1 mm moving partition (tray, keepers, sheet edge, robot-side plug).
add("Battery tray and retention", tray, group="battery retention", role="other",
    function="Battery tray, four hangers to the inboard sills and two end keepers",
    parent="fixed: hangers bolted under the inboard sills", motion="fixed",
    bounds="Planning reserve sized around the largest battery case; not a design",
    missing=["Tray, hanger and keeper design; top hold-down strap (removed before the lift) is not represented",
             "Ground clearance under the 22 mm tray bottom against field obstacles"])
battery = fuse([box([-241.5, -39.0, 33.0], [-58.5, 39.0, 202.0]),
                box([-283.0, 10.0, 170.0], [-241.5, 64.9, 201.0]),
                box([-317.0, 37.0, 180.0], [-281.0, 77.0, 196.0])])
add("Dressed battery", battery, group="battery", role="battery",
    function="Largest supported battery case, its leads and the battery-side SB50",
    parent="fixed in operation on the tray; withdrawn for service", motion="service: lift 250 mm then rearward",
    bounds="Largest-battery gauge (inset-electrical-package gaugesMm) plus lead and plug reserves; plug partitioned at Y 78",
    missing=["Exact battery article (readiness C7)", "Grip handle above the case top inside the 250 mm lift gauge"])

# ---------------------------------------------------------------- service hands (tools)
add("Breaker reset hand", box([-96.0, 258.0, 600.0], [-64.0, 290.0, 760.0]), group="breaker service hand", role="service",
    function="Hand reaching the CB185 red trip-reset button from above, outboard of the port stringer extension",
    parent="service envelope, parked above the breaker", motion="service: down to 3.2 mm above the breaker",
    bounds="32 x 32 mm hand column over the red button target (-79.87, 274, 254.15)",
    missing=["Finger contact on the button is the separate inset-breaker-access check"])
add("Plug grip hand", box([-321.0, 27.0, 455.0], [-286.1, 115.0, 640.0]), group="plug service hand", role="service",
    function="Hand gripping the SB50 pair from above to disconnect the battery",
    parent="service envelope, parked under feed panel 1", motion="service: down to 3.1 mm above both plugs",
    bounds="plug grip from above gauge (inset-electrical-package gaugesMm) Y footprint, X narrowed to -321..-286.1 so the grip stays 3.1 mm off the battery leads",
    missing=["Unmating stroke of the SB50 pair"])

# ---------------------------------------------------------------- the finite pipe
d = (330.0, -93.0); L = math.hypot(*d); d = (d[0] / L, d[1] / L); n = (-d[1], d[0])
rc = PIPE_APOTHEM
pipe = cylinder((0.0, -PIPE_LENGTH / 2, 0.0), (0, 1, 0), PIPE_APOTHEM, PIPE_LENGTH)
s2 = (-225.0, 685.0 - 30.0 * 93.0 / 330.0)
P0 = (s2[0] + n[0] * (rc + 3.5), s2[1] + n[1] * (rc + 3.5))
s3 = (60.0, 592.0 + 15.0 * 93.0 / 330.0)
P1 = (s3[0] + n[0] * (rc + 3.5), s3[1] + n[1] * (rc + 3.5))
EDGE = (75.0, 592.0)
a = math.radians(STOW_FROM_FLAT)
def flat_to_stow(u, w):
    x, z = u - PIVOT[0], w - PIVOT[2]
    return (PIVOT[0] + x * math.cos(a) + z * math.sin(a), PIVOT[2] - x * math.sin(a) + z * math.cos(a))
SEAT_GAP = 3.0
SEAT_FLAT = (115.0 + rc + SEAT_GAP, 580.0 + rc + SEAT_GAP)
PS = flat_to_stow(*SEAT_FLAT)
r1 = math.dist(P1, EDGE); ang1 = math.atan2(P1[1] - EDGE[1], P1[0] - EDGE[0])
angs = math.atan2(PS[1] - EDGE[1], PS[0] - EDGE[0])
ROLL = math.degrees(ang1 - angs)    # rotation about +Y through the edge (positive moves the centre forward and down)
P2 = (EDGE[0] + r1 * math.cos(angs), EDGE[1] + r1 * math.sin(angs))
add("CORAL pipe", moved(pipe, (PS[0], 0.0, PS[1])), group="pipe", role="payload",
    function="One station-issued CORAL from the station chute exit through receive, carry, dump and release",
    parent="moving: installed pose is the stow seat; free from the station to the seat, then carried with the pan (carriedWith) to release",
    motion="station drop to the chute, roll-over, seat, carry, tip, release",
    bounds="Cylinder of the 114.3 mm OD, 301.625 mm long pipe (hollow bore filled)",
    missing=["CORAL geometry is not registered in kickoff/rules-v0.json (game pieces omitted)",
             "Contact with the chute and the pan is partitioned at 2.1 mm; rolling and seated custody are qualified by pan-stop-study-05, not this layout"])

# ---------------------------------------------------------------- write STEP
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.Interface import Interface_Static

for name in order:
    s = blocks[name]["shape"]
    assert valid(s) and volume(s) > 0, name
doc = TDocStd_Document(TCollection_ExtendedString("BinXCAF"))
table = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
for name in order:
    TDataStd_Name.Set_s(table.AddShape(blocks[name]["shape"], False), TCollection_ExtendedString(name))
Interface_Static.SetCVal_s("write.step.unit", "MM")
writer = STEPCAFControl_Writer()
OUT.mkdir(parents=True, exist_ok=True)
step_path = OUT / "layout.step"
assert not step_path.exists()
assert writer.Transfer(doc, STEPControl_AsIs) and writer.Write(str(step_path)) == IFSelect_RetDone

manifest = dict(
    schema="red2/preliminary-layout-blocks@1",
    partitionMm=G,
    inputs={k: pin(v) for k, v in dict(chassisStep=CHASSIS_STEP, panGeometry=PAN_GEOMETRY, electrical=ELECTRICAL,
                                       panOperatingIntent="iterations/002/design/pan/operating-intent.json",
                                       bumperRecord="cad/rear-frame-facets/construction.json").items()},
    generator={"file": "iterations/002/design/preliminary/build_layout.py", "sha256": sha(Path(__file__))},
    frameOutlineMm=OUTLINE, rearCutbackMm=CUTBACK,
    poses=dict(pivotMm=list(PIVOT), stowFromFlatDeg=STOW_FROM_FLAT, tipDeg=72.0,
               pipe=dict(apothemMm=PIPE_APOTHEM, circumradiusMm=rc, lengthMm=PIPE_LENGTH,
                         stationExitXZ=P0, chuteEndXZ=P1, rollEdgeXZ=EDGE, rollDeg=ROLL, afterRollXZ=P2,
                         seatStowXZ=PS, seatFlatXZ=SEAT_FLAT, seatGapMm=SEAT_GAP, chuteGapMm=3.5),
               partitionMm=dict(default=G, besideMoving=GM, steerToHousing=GT),
               stowPadReserveMm=pads["Port"][1], steer={str(k): v for k, v in turret_bands.items()}),
    blocks=[dict(name=n, **{k: v for k, v in blocks[n].items() if k != "shape"},
                 boundsMm=[round(x, 3) for x in aabb(blocks[n]["shape"])], volumeMm3=round(volume(blocks[n]["shape"]), 1))
            for n in order])
(OUT / "layout-blocks.json").write_text(json.dumps(manifest, indent=1) + "\n", encoding="utf-8")
print(len(order), "blocks;", "roll", round(ROLL, 2), "seat", [round(v, 2) for v in PS], "pad", round(pads["Port"][1], 2))
