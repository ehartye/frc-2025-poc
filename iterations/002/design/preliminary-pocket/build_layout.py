"""Build the B'x pocket preliminary package (layout-002): labeled reserves in the robot datum.

    python -I -B build_layout.py PROJECT_ROOT RELEASE_DIR PLUGIN_SCRIPTS FIELD_TRIALS_JSON

Writes RELEASE_DIR/package.step (the robot and the finite pipe), RELEASE_DIR/field-fixtures.step (the station chute,
the L1 trough and the first scored pipe, as motion-study fixtures) and RELEASE_DIR/layout-blocks.json.

The chassis, modules, bumpers, electronics, wiring, battery and service gauges are rebuilt exactly as the
iteration 002 pan baseline builds them (iterations/002/design/preliminary/build_layout.py), from the same pinned
inputs, as revisable premises. The pan, its cassette, upper support and fixed inlet are not carried. The pocket
section comes from pocket_geometry.py, the same file the field trials integrate against.

Reserve conventions (baseline): mated reserves are partitioned at >= 2.1 mm (3.1 mm beside a moving reserve), so
every stationary pair keeps the 2.0 mm case reserve. Pipe-contact surfaces (floor, deployed lip) are reserved
2.5 mm below the contact line: seated and rolling contact is the qualified interface of the field trials, not of
this clearance layout. The nip rollers are reserved as their rigid hub and core (radius 16.4 mm); the outer 9 mm
of each 2 in tread is the designed 6 mm squeeze plus 3 mm and is checked by the trials, not here.
"""
import hashlib
import json
import math
import sys
from pathlib import Path

ROOT = Path(sys.argv[1]).resolve()
OUT = Path(sys.argv[2]).resolve()
sys.path.insert(0, str(Path(sys.argv[3]).resolve()))
TRIALS = Path(sys.argv[4]).resolve()
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from showcase_contacts import load_solids  # noqa: E402
from layout_occ import (box, aabb, volume, solids, fuse, cut, common, revolved, distance, valid, loft_solid, prism_xy,  # noqa: E402
                        cylinder, nodes, obb_box, rotated, moved, offset_polygon)
import pocket_geometry as pg  # noqa: E402

G = 2.1
GM = 3.1
GT = 4.0
CHASSIS_STEP = "artifacts/resume-20261121/screws-place-01/four-seated-screws.step"
ELECTRICAL = "iterations/002/design/chassis/inset-electrical-package.json"
SRC = "iterations/002/design/preliminary-pocket"            # this builder and pocket_geometry.py
REL = OUT.relative_to(ROOT).as_posix()                       # the release folder, project-relative
FIXTURE_GROUPS = ("field trough", "field station", "field first pipe")
CONTACT_RESERVE = 2.5      # pipe-contact surfaces reserved below their contact line
LIP_RESERVE = 3.5          # the lip, whose root corner the pipe pivots over, keeps 3.5 mm so that arc stays resolvable
W_IN = 161.0               # pocket inner half-width (322 mm, the kept pan profile width)
W_T = 3.175                # 1/8 in polycarbonate walls and floor
PLATE_Y = (W_IN + W_T + GM, W_IN + W_T + GM + 6.35)   # 1/4 in nip side plates
SHAFT_Y = PLATE_Y[1] + 2.35
DRIVE_Y0 = SHAFT_Y + G


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pin(rel):
    return {"file": rel, "sha256": sha(ROOT / rel)}


blocks, order = {}, []


def add(name, shape, **meta):
    if name in blocks:
        raise ValueError("duplicate block " + name)
    assert valid(shape) and volume(shape) > 0, name
    assert len(solids(shape)) == 1, (name, len(solids(shape)))   # one closed solid per placed ID
    blocks[name] = dict(shape=shape, **meta)
    order.append(name)


def box_gap_(a, b):
    return math.sqrt(sum(max(0.0, a[i] - b[i + 3], b[i] - a[i + 3]) ** 2 for i in range(3)))


def inflated_boxes(shapes_list):
    return [obb_box(s, G)[0] for s in shapes_list]


def sec_y(poly_xz, y0, y1):
    """Prism of an XZ polygon between two Y planes."""
    return loft_solid([(x, y0, z) for x, z in poly_xz], [(x, y1, z) for x, z in poly_xz])


def band(points, below, above=0.0):
    """XZ polygon of a strip under a polyline (offset normals: 'above' up, 'below' down), per segment, fused later."""
    quads = []
    for (x0, z0), (x1, z1) in zip(points, points[1:]):
        L = math.hypot(x1 - x0, z1 - z0)
        nx, nz = -(z1 - z0) / L, (x1 - x0) / L     # left normal of a +X run is +Z
        if nz < 0:
            nx, nz = -nx, -nz
        quads.append([(x0 + nx * above, z0 + nz * above), (x1 + nx * above, z1 + nz * above),
                      (x1 - nx * below, z1 - nz * below), (x0 - nx * below, z0 - nz * below)])
    # Fill the wedge between consecutive segment strips at each bend with the convex hull of their end edges.
    joints = []
    for qa, qb in zip(quads, quads[1:]):
        hull = convex_hull([qa[1], qa[2], qb[0], qb[3]])
        if len(hull) >= 3 and abs(polygon_area(hull)) > 1e-3:
            joints.append(hull)
    return quads + joints


def polygon_area(poly):
    return 0.5 * sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(poly, poly[1:] + poly[:1]))


def convex_hull(points):
    pts = sorted(set((round(x, 9), round(z, 9)) for x, z in points))
    if len(pts) < 3:
        return pts
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p_ in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p_) <= 1e-12:
            lower.pop()
        lower.append(p_)
    for p_ in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p_) <= 1e-12:
            upper.pop()
        upper.append(p_)
    return lower[:-1] + upper[:-1]


trials = json.loads(TRIALS.read_text(encoding="utf-8"))
TS = trials["summary"]
MIN_NIP = TS["g2MinNipMps"]
BACKDRIVE = TS["g1BackDriveNm"]
SEAT = tuple(trials["carrySeatsByTreadStiffness"]["80.0"])
import field_trials as ft  # noqa: E402  (station registration and dock gap: the same numbers the trials integrate)

# ================================================================ carried-forward chassis (baseline code, unchanged)
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
CUTBACK = 370.0 + 365.0 - max(abs(y) - x for n in chassis for x, y, z in nodes(chassis[n]))
OUTLINE = [(370.0, -365.0), (370.0, 365.0), (-370.0 + CUTBACK, 365.0), (-370.0, 365.0 - CUTBACK),
           (-370.0, -365.0 + CUTBACK), (-370.0 + CUTBACK, -365.0)]
S_infl = fuse(inflated_boxes([chassis[n] for n in structure]))
CHASSIS_GROUPS = [
    ("Chassis perimeter", ("Structural perimeter", "Frame support", "Sleeve"),
     "Structural perimeter tubes, the two rear chamfer tubes, frame supports and wheel-station sleeves"),
    ("Chassis cassettes and socket rails", ("Cassette", "socket rail"),
     "Lower module cassettes and the eight socket-rail ends the module plates clamp"),
    ("Chassis pan trusses and stringers", ("Pan truss", "Upper pan stringer"),
     "Port and starboard pan trusses (members 0-5) and the upper pan stringers, kept as the pocket landings"),
    ("Chassis sills, decks and crossmembers", None,
     "Inboard sills, pan crossmembers, truss landings and nutbars, equipment sheets, tabs, perimeter keys, keyed pedestals "
     "and nutbars, and the four seated 91251A542 clamp screws"),
]
clip = prism_xy(OUTLINE, -1.0, 600.0)
assigned = {}
for title, keys, _ in CHASSIS_GROUPS:
    assigned[title] = [n for n in structure if (keys is None or any(k in n for k in keys))
                       and not any(n in v for v in assigned.values())]
assert not [n for n in structure if not any(n in v for v in assigned.values())]
higher = []
for title, keys, text in CHASSIS_GROUPS:
    names_ = assigned[title]
    raw = common(fuse([obb_box(chassis[n])[0] for n in names_]), clip)
    pieces = sorted(solids(cut(raw, higher)) if higher else solids(raw), key=volume, reverse=True)
    for i, piece in enumerate(pieces):
        add(title + ("" if i == 0 else f" part {i + 1}"), piece, group="structure", role="structure",
            function=text + ("" if i == 0 else " (separated piece of the same reserve)"),
            parent="fixed (robot datum)", motion="fixed",
            bounds=f"Union of per-body boxes of {len(names_)} chassis bodies in " + CHASSIS_STEP +
                   " clipped to the frame plan hull and partitioned 2.1 mm from the higher-priority chassis reserves; holes and pockets filled",
            missing=["Bolted and riveted joints between chassis reserves lie inside the 2.1 mm partition shells"],
            carried=True)
    higher.append(fuse(inflated_boxes([chassis[n] for n in names_])))

turret_bands = {}
for k, mod in modules.items():
    ring = [aabb(chassis[n]) for n in mod["turret"] if "A035XPO" in "".join(ids[n]["occurrencePath"])]
    assert len(ring) == 1
    cx, cy = (ring[0][0] + ring[0][3]) / 2, (ring[0][1] + ring[0][4]) / 2
    pts = [p for n in mod["turret"] for p in nodes(chassis[n])]
    ztop_t = max(p[2] for p in pts)
    slabs, z = [], 0.0
    while z < ztop_t - 1e-9:
        z1 = min(z + 4.0, ztop_t)
        r = max((math.hypot(x - cx, y - cy) for x, y, zz in pts if z - 1e-6 <= zz <= z1 + 1e-6), default=0.0)
        slabs.append([z, z1, max(r + 0.5, 1.0)])
        z = z1
    T = revolved((cx, cy), [tuple(sl) for sl in slabs])
    T_infl = revolved((cx, cy), [(-GT, 0.0, slabs[0][2] + GT)] + [(a_, b_, r_ + GT) for a_, b_, r_ in slabs] + [(ztop_t, ztop_t + GT, slabs[-1][2] + GT)])
    T_infl = fuse([T_infl] + [revolved((cx, cy), [(max(a_ - GT, -GT), b_ + GT, r_ + GT)]) for a_, b_, r_ in slabs])
    reach = max(sl[2] for sl in slabs)
    turret_bands[k] = dict(axis=[cx, cy], revolvedProfile=slabs, envelopeRadiusMm=reach)
    corner = {1: "front-left", 2: "front-right", 3: "rear-right", 4: "rear-left"}[k]
    add(f"Module {k} steer sweep", T, group="steer sweeps", role="drive",
        function=f"{corner} SDS MK4i L2 turret: wheel, wheel mounts, steering pulley and turret gears, full 360 deg steer swept volume",
        parent=f"static envelope about the module {k} steer axis ({cx:.2f}, {cy:.2f}) mm, +Z; covers every azimuth",
        motion="none (the revolved envelope already contains every steer angle)",
        bounds=f"Solid of revolution of the turret constituents' radial profile in 4 mm slabs (max meshed radius + 0.5 mm, "
               f"largest {reach:.1f} mm) about the steer axis, Z 0-{ztop_t:.1f} mm",
        missing=["Loaded tire radius and tread compression (factory tire floor Z = 0)"], carried=True)
    H = fuse([obb_box(chassis[n])[0] for n in mod["housing"]])
    for i, piece in enumerate(sorted(solids(cut(H, [T_infl, S_infl])), key=volume, reverse=True)):
        add(f"Module {k} housing" + ("" if i == 0 else f" part {i + 1}"), piece, group="module housings", role="drive",
            function=f"{corner} module fixed housing: plates, drive and steer NEOs, CANcoder, gears, belt and 10-32 frame screws"
                     + ("" if i == 0 else " (separated piece of the same housing after partition)"),
            parent="fixed: clamped to the chassis cassette and socket rails", motion="fixed",
            bounds="Per-body boxes of the 84 non-turret module solids, partitioned from the chassis and the steer sweep",
            missing=["Frame-screw seating and clamp are inside the 2.1 mm partition shell"], carried=True)

inner, outer = offset_polygon(OUTLINE, G), offset_polygon(OUTLINE, 75.5)
ring = cut(prism_xy(outer, 43.0, 181.0), [prism_xy(inner, 42.0, 182.0)])
add("Bumper ring", solids(ring)[0], group="bumpers", role="bumper",
    function="Continuous bumper backing, foam and cover around the chamfered frame, both alliance covers",
    parent="fixed: removable bumper on the frame perimeter", motion="fixed",
    bounds=f"Mitred normal offsets of the actual frame plan hull ({CUTBACK:.2f} mm rear cutback): 1 mm gap, 4 mm backer, 69.5 mm foam, 0.5 mm cover, Z 43-181 mm",
    missing=["002 bumper construction, mounts and the applicable R4xx edition check are not issued",
             "Front-strike crush not represented"], carried=True)

elec = json.loads((ROOT / ELECTRICAL).read_text(encoding="utf-8"))["candidatePlacements"]
names = {"Power distribution": "Power distribution hub", "Control computer": "roboRIO 2", "Radio": "VH-109 radio",
         "Main 120 A breaker": "Main 120 A breaker"}
for key, row in elec.items():
    if key in ("Chassis battery plug", "Battery plug"):
        continue
    lo, hi = row["boundsMm"][:3], row["boundsMm"][3:]
    nm = names.get(key, key[0].upper() + key[1:] + " (SPARK MAX)" if "controller" in key else key)
    nm = nm.replace("Pan controller", "Nip controller")
    add(nm, box(lo, hi), group="electronics", role="other",
        function=f"{row['source']} at its 552-r5 prospective placement on the equipment sheet",
        parent="fixed: on standoffs on the equipment sheet (Z 205.2)", motion="fixed",
        bounds="World bounds of the mounted supplier solids (" + row["source"] + ")",
        missing=["Device brackets and standoffs between Z 205.2 and the device", "Connector and lead bend-radius envelopes"],
        carried=True)
plug = elec["Chassis battery plug"]["boundsMm"]
add("Chassis battery plug and bracket", fuse([box([plug[0], 80.1, plug[2]], [plug[3], plug[4], plug[5]]),
                                               box([-331.9 + G, 80.1, 180.0], [plug[0], plug[4], 196.0])]),
    group="electronics", role="other", function="Robot-side SB50 (mates the battery plug) and its bracket to the rear perimeter",
    parent="fixed: bracket to structural perimeter 2", motion="fixed",
    bounds="SB50 supplier bounds partitioned at the mating plane Y 78; bracket reserve", missing=["Bracket design and fasteners"], carried=True)
add("Main power loom", fuse([box([-317.0, 121.1, 180.0], [-281.0, 140.0, 222.0]),
                             box([-281.0, 121.1, 198.1], [-270.2, 229.9, 222.0]),
                             box([-270.2, 210.5, 210.2], [-122.1, 234.9, 222.0])]),
    group="wiring", role="other", function="6 AWG main leads: chassis SB50 riser, rear run and run to the 120 A breaker and PDH",
    parent="fixed: tied to the rear frame and the equipment sheet", motion="fixed",
    bounds="Rectangular channels clear of the battery lift and plug grip", missing=["Lead bend radius at terminals", "Strain relief"], carried=True)
add("Branch and CAN trunk", fuse([box([-28.9, -20.0, 210.2], [240.0, 20.0, 226.0]),
                                  box([20.0, -96.6, 210.2], [56.0, 96.6, 226.0]),
                                  box([180.0, -96.6, 210.2], [216.0, 96.6, 226.0])]),
    group="wiring", role="other", function="PDH branch and CAN trunk on the centerline with cross runs to the drive and steer controllers",
    parent="fixed: supported from the equipment sheets across the centre gap", motion="fixed",
    bounds="15.8 mm high channels at Z 210.2-226", missing=["Runs to the outer controllers and the nip and latch leads; route undesigned"], carried=True)
for k, (sx, sy) in {1: (1, 1), 2: (1, -1), 3: (-1, -1), 4: (-1, 1)}.items():
    x0, x1 = sorted((150.0 * sx, 205.0 * sx)); y0, y1 = sorted((145.0 * sy, 200.0 * sy))
    add(f"Module {k} lead reserve", box([x0, y0, 174.4], [x1, y1, 201.1]), group="wiring", role="other",
        function=f"Drive, steer and CANcoder leads leaving module {k} under the equipment sheet",
        parent="fixed: between the module housing and the sheet", motion="fixed",
        bounds="55 x 55 x 26.7 mm reserve over the inboard motor ends", missing=["Sheet pass-through and grommet"], carried=True)
tray = fuse([box([-250.0, -100.5, 21.0], [-50.0, 100.5, 29.9]),
             *[box([x0, s0, 29.9], [x0 + 20.0, s1, 109.0]) for x0 in (-220.0, -100.0) for s0, s1 in ((60.0, 100.5), (-100.5, -60.0))],
             box([-248.7, -30.0, 29.9], [-244.6, 30.0, 100.0]), box([-55.4, -30.0, 29.9], [-51.3, 30.0, 100.0])])
add("Battery tray and retention", tray, group="battery retention", role="other",
    function="Battery tray, four hangers to the inboard sills and two end keepers", parent="fixed: hangers bolted under the inboard sills",
    motion="fixed", bounds="Planning reserve sized around the largest battery case; not a design",
    missing=["Tray, hanger and keeper design; top hold-down strap"], carried=True)
add("Dressed battery", fuse([box([-241.5, -39.0, 33.0], [-58.5, 39.0, 202.0]), box([-283.0, 10.0, 170.0], [-241.5, 64.9, 201.0]),
                             box([-317.0, 37.0, 180.0], [-281.0, 77.0, 196.0])]),
    group="battery", role="battery", function="Largest supported battery case, its leads and the battery-side SB50",
    parent="fixed in operation on the tray; withdrawn for service", motion="service: lift 250 mm then rearward",
    bounds="Largest-battery gauge plus lead and plug reserves", missing=["Exact battery article"], carried=True)
add("Breaker reset hand", box([-96.0, 258.0, 600.0], [-64.0, 290.0, 760.0]), group="breaker service hand", role="service",
    function="Hand reaching the CB185 red trip-reset button from above", parent="service envelope, parked above the breaker",
    motion="service: down to 3.2 mm above the breaker", bounds="32 x 32 mm hand column", missing=["Finger contact on the button"], carried=True)
add("Plug grip hand", box([-321.0, 27.0, 455.0], [-286.1, 115.0, 640.0]), group="plug service hand", role="service",
    function="Hand gripping the SB50 pair from above to disconnect the battery", parent="service envelope, parked under the inlet chute",
    motion="service: down to 3.1 mm above both plugs", bounds="plug grip from above gauge", missing=["Unmating stroke of the SB50 pair"], carried=True)

# ================================================================ B'x pocket package (new in this layout)
c15 = math.cos(math.radians(pg.FLOOR_DEG))
# --- pocket tray: floor under the contact polyline plus the two side walls, one removable weldment-like reserve
floor_quads = band(pg.INLET, below=CONTACT_RESERVE + W_T, above=-CONTACT_RESERVE)
floor = fuse([sec_y(q, -(W_IN + W_T), W_IN + W_T) for q in floor_quads])
WALL_TOP = 150.0
wall_quads = band(pg.INLET, below=CONTACT_RESERVE + W_T, above=WALL_TOP)
WALL_X1 = 300.0
wall_clip = box([-400.0, -400.0, 0.0], [WALL_X1, 400.0, 1200.0])
walls = [common(fuse([sec_y(q, s * W_IN, s * (W_IN + W_T)) if s > 0 else sec_y(q, -(W_IN + W_T), -W_IN) for q in wall_quads]), wall_clip)
         for s in (1, -1)]
tray_solid = fuse([floor] + walls)
tray_solid = sorted(solids(tray_solid), key=volume, reverse=True)
assert len(tray_solid) == 1, [volume(s) for s in tray_solid]
add("Pocket tray", tray_solid[0], group="pocket tray", role="mechanism",
    function="Station inlet chute (52.3 deg), 15.74 deg pocket floor and two 322 mm-apart side walls in 1/8 in polycarbonate",
    parent="fixed: on the rear uprights and the two floor posts on the upper stringers", motion="fixed",
    bounds=f"Floor 3.175 mm thick reserved {CONTACT_RESERVE} mm below the pipe-contact polyline {[tuple(round(v, 1) for v in p) for p in pg.INLET]}; "
           f"walls {WALL_TOP:.0f} mm above the contact line, X to {WALL_X1:.0f}",
    missing=["Floor stiffening ribs and wall-to-floor joints are not represented",
             "Low-friction face (UHMW tape) and its measured mu; the trials sweep 0.15-0.35"])
# --- hood (removable for pit unload)
hz0 = lambda x: pg.hood_z(x)
hood_poly = [(pg.HOOD[0][0], hz0(pg.HOOD[0][0])), (pg.HOOD[1][0], hz0(pg.HOOD[1][0])),
             (pg.HOOD[1][0], hz0(pg.HOOD[1][0]) + 1.6), (pg.HOOD[0][0], hz0(pg.HOOD[0][0]) + 1.6)]
HOOD_W = W_IN - GM          # 3.1 mm from the walls: the hood lifts out between them
(ffx, ffz), (fhx, fhz) = pg.HOOD_FLARE
flare_poly = [(ffx, ffz), (fhx + 2.0, hz0(fhx + 2.0)), (fhx + 2.0, hz0(fhx + 2.0) + 1.6), (ffx, ffz + 1.6)]
hood_solid = sorted(solids(fuse([sec_y(hood_poly, -HOOD_W, HOOD_W), sec_y(flare_poly, -HOOD_W, HOOD_W)])), key=volume, reverse=True)
assert len(hood_solid) == 1
add("Hood (removable)", hood_solid[0], group="hood", role="mechanism",
    function="1/16 in polycarbonate hood 10 mm above the seated pipe, with a 90 mm lead-in flare 30 deg above the hood line, carrying the "
             "check-flap pin; thumb-screw removable",
    parent="fixed: tabs on the side-wall tops (thumb screws)", motion="fixed (service: lift off)",
    bounds="1.6 mm plate on the hood line, flap to the upper roller, plus the flare plate",
    missing=["Hood tabs and thumb screws", "Hood and flare deflection under a strike (unsized; the trials treat both as rigid)"])
# --- check flap (hangs from a 3 mm pin 4 mm under the hood)
hx, hzz = pg.FLAP_HINGE
flap_plate = [(hx - 0.8, hzz), (hx + 0.8, hzz), (hx + 0.8, hzz - pg.FLAP_LENGTH), (hx - 0.8, hzz - pg.FLAP_LENGTH)]
FLAP_W = 152.0
flap = fuse([sec_y(flap_plate, -FLAP_W, FLAP_W), cylinder((hx, -FLAP_W, hzz), (0, 1, 0), 1.5, 2 * FLAP_W)])
add("Check flap", flap, group="check flap", role="mechanism",
    function="One-way spring check flap: the arriving pipe swings it forward and up; its rear stop at vertical blocks a rebound or rearward carry load",
    parent=f"moving: revolute on a 3 mm pin at ({hx:.1f}, {hzz:.1f}) mm about +Y, on the hood", motion=f"0 to {pg.FLAP_OPEN_DEG:.1f} deg open",
    bounds="1.6 mm polycarbonate plate 304 mm wide to 6 mm above the floor, plus the pin",
    missing=["Torsion spring, its preload and the rear stop tab are not represented (trials: 0.04 N m preload, 0.06 N m/rad)"])
# --- nip side plates (land on the upper stringers)
# Seated-pipe break beam, along Y just under the top of a seated pipe: 54 mm above the seat centre the pipe wall is solid
# for +/-18.7 mm, so the beam is blocked for pipe centres X 296.0-333.4 (seats 304.1-310.3) and clear at 10 mm displaced.
BEAM = (314.7, pg.POCKET_Z + 54.0)
plate_poly = [(240.0, 491.7), (300.0, 491.7), (300.0, 425.0), (368.0, 425.0), (368.0, 722.0), (240.0, 722.0)]
plates = []
for s in (1, -1):
    y0, y1 = sorted((s * PLATE_Y[0], s * PLATE_Y[1]))
    p = sec_y(plate_poly, y0, y1)
    holes = [cylinder((c[0], -300.0, c[1]), (0, 1, 0), pg.SHAFT_R + 0.65 + G, 600.0) for c in (pg.LOWER_ROLLER, pg.UPPER_ROLLER)]
    holes.append(cylinder((BEAM[0], -300.0, BEAM[1]), (0, 1, 0), 8.5, 600.0))      # seated-pipe beam window
    plates.append(cut(p, holes))
for s, pl in zip(("Port", "Starboard"), plates):
    add(f"{s} nip side plate", pl, group="nip side plates", role="mechanism",
        function=f"{s} 1/4 in 6061 side plate: carries both roller bearings, the lip hinge bracket and the drive or coupling",
        parent="fixed: two 10-32 bolts 45 mm apart into the upper pan stringer top (X 245 and 290)", motion="fixed (service: lifts out with the nip cassette)",
        bounds="1/4 in plate profile X 240-368, Z 425-722; shaft holes partitioned 2.1 mm around the shaft",
        missing=["Flanged bearings sit inside the shaft-hole partition", "Pocketing (50 % assumed in the mass line) is not represented",
                 "Bolt pattern is sized by calculation (structure-checks.json P1-P3), not drawn"])
for nm, c in (("Lower nip roller", pg.LOWER_ROLLER), ("Upper nip roller", pg.UPPER_ROLLER)):
    core = cylinder((c[0], -160.0, c[1]), (0, 1, 0), pg.CORE_R, 320.0)
    shaft = cylinder((c[0], -SHAFT_Y, c[1]), (0, 1, 0), pg.SHAFT_R + 0.65, 2 * SHAFT_Y)
    add(nm, fuse([core, shaft]), group=nm.lower(), role="mechanism",
        function=f"{nm.split()[0]} roller of the nip: 2 in compliant wheels (AndyMark am-3462 class, 1/2 in hex bore) on a 1/2 in steel hex shaft; "
                 "wheel count and durometer are set on the bench to 40-120 N/mm at the 6 mm squeeze. With its mate it is the pocket's front stop and the eject drive",
        parent=f"moving (spin): revolute about +Y through ({c[0]:.2f}, {c[1]:.2f}) mm in the side-plate bearings",
        motion="spins; its occupied core is axisymmetric",
        bounds=f"Rigid hub/core reserve radius {pg.CORE_R} mm over |Y| <= 160 plus the shaft to |Y| {SHAFT_Y:.2f}; "
               f"the outer {pg.ROLLER_R - pg.CORE_R:.1f} mm of tread (6 mm designed squeeze + 3 mm) is the trial-qualified contact",
        missing=["Wheel hubs, spacers and shaft collars", "Installed tread stiffness: no supplier publishes it; bench gate G0 measures 240-720 N at 6 mm"])
# --- drive in line with the lower shaft on the port plate outside; coupling belt on the starboard plate outside
lz = pg.LOWER_ROLLER[1]
GB_LEN, MOTOR_LEN = 92.0, 62.0
drive = fuse([box([pg.THROAT_X - 27.0, DRIVE_Y0, lz - 27.0], [pg.THROAT_X + 27.0, DRIVE_Y0 + GB_LEN, lz + 27.0]),
              cylinder((pg.THROAT_X, DRIVE_Y0 + GB_LEN - 0.5, lz), (0, 1, 0), 18.5, MOTOR_LEN + 0.5)])
add("Nip drive (NEO 550 and MAXPlanetary 36:1)", drive, group="nip drive", role="mechanism",
    function="REV NEO 550 into a REV MAXPlanetary 4:1 x 3:1 x 3:1 (36:1) whose 1/2 in hex socket takes the lower roller shaft: "
             "powered position hold on receive and carry, open-loop eject, back-drivable when the robot is disabled",
    parent="fixed: gearbox face on the port side plate outside face, coaxial with the lower shaft", motion="fixed",
    bounds=f"54 x 54 x {GB_LEN:.0f} mm gearbox envelope plus a 37 mm x {MOTOR_LEN:.0f} mm motor cylinder along +Y",
    missing=["Envelope is a planning reserve: confirm the three-stage length against REV's STEP in detailed design",
             f"Efficiency is assumed (0.9 per stage): the trials need >= {MIN_NIP:.2f} m/s rim speed through the squeeze at 9 V and hold "
             f"{BACKDRIVE:.2f} N m on receive", "Motor encoder lead exit"])
add("Nip coupling belt", box([322.0, -232.0, lz - 26.0], [368.0, -DRIVE_Y0, pg.UPPER_ROLLER[1] + 26.0]), group="nip coupling",
    role="mechanism", function="Serpentine belt, two pulleys and an idler counter-rotating the upper roller from the lower shaft",
    parent="fixed: on the starboard side plate outside face", motion="fixed (belt runs inside the envelope)",
    bounds="Box envelope over both pulleys, the idler and the belt run", missing=["Belt article, idler and tension adjustment"])
for s_, label in ((1, "emitter (port)"), (-1, "receiver (starboard)")):
    y0, y1 = sorted((s_ * 162.0, s_ * 182.0))
    add(f"Seated-pipe beam {label}", box([BEAM[0] - 5.0, y0, BEAM[1] - 4.0], [BEAM[0] + 5.0, y1, BEAM[1] + 4.0]), group="pipe sensor", role="other",
        function=f"Adafruit 2168 5 mm IR break-beam {label.split()[0]}: the beam crosses the pocket along Y just under the top of a seated pipe, so it "
                 "is blocked only while the pipe sits in the bite (centre X 296-333) and clears when the pipe is 10 mm back or gone",
        parent=f"fixed: through a window in the {label.split('(')[1][:-1]} nip side plate", motion="fixed",
        bounds="20 x 10 x 8 mm sensor body along Y through the plate window", missing=["Sensor clip on the plate", "Lead to the roboRIO DIO"])
hz_sw = max(pg.hood_z(hx - 13.0), pg.hood_z(hx + 13.0)) + 1.6 + G
add("Flap-closed switch", box([hx - 13.0, 118.0, hz_sw], [hx + 13.0, 150.0, hz_sw + 10.0]), group="flap sensor", role="other",
    function="REV magnetic limit switch (REV-31-1462) on the hood top over the flap hinge, with a magnet on the flap: flap closed again behind "
             "the pipe is the captive signal that releases the robot from the station",
    parent="fixed: on the hood top at the flap hinge line", motion="fixed (service: lifts off with the hood)",
    bounds="26 x 32 x 10 mm switch envelope 2.1 mm above the hood plate", missing=["Magnet on the flap and its 10 mm trigger gap", "Lead to the roboRIO DIO"])
add("Lip-deployed switch", box([340.0, -232.0, 705.0], [366.0, -DRIVE_Y0, 720.0]), group="lip sensor", role="other",
    function="REV magnetic limit switch (REV-31-1462) on the starboard side plate that the deployed lip's magnet tab closes: lip state for the eject interlock",
    parent="fixed: on the starboard side plate outside face above the belt", motion="fixed",
    bounds="26 x 54 x 15 mm switch envelope", missing=["The lip tab and magnet that reach it"])
add("Reef range sensor (CANrange)", box([329.0, -(PLATE_Y[0] - G) , 427.2], [366.0, -(PLATE_Y[0] - G - 18.0), 461.7]), group="reef sensor", role="other",
    function="CTRE CANrange time-of-flight sensor looking forward under the lip at the trough face: eject is allowed only at a range that "
             "means a stand-off of 100 mm or less, which also puts the bumper at least 255 mm inside the reef zone (G412)",
    parent="fixed: on the starboard side plate inside face, below the lower roller", motion="fixed",
    bounds="36.8 x 18 x 34.5 mm sensor envelope (1.45 x 0.71 x 1.36 in), optical axis +X at Z 444",
    missing=["Bracket to the plate", "CAN and power lead", "Registered reef structure below the trough lip that the beam reads"])
add("Lip latch servo", box([330.0, DRIVE_Y0, 600.0], [366.0, 232.0, 650.0]), group="lip latch", role="mechanism",
    function="Start latch: a REV Smart Robot Servo pin holds the upright stowed lip and releases it once at match start",
    parent="fixed: on the port side plate outside face above the drive", motion="fixed (pin stroke inside the envelope)",
    bounds="36 x 51 x 50 mm servo and pin envelope", missing=["PWM port and power", "Pin engagement with the lip tab through the plate"])
# --- deploy-once exit lip: built deployed from the profile, stored stowed (rotated back about the hinge)
prof = pg.lip_profile()
lip_quads = band(prof, below=LIP_RESERVE + pg.LIP_T, above=-LIP_RESERVE)
LIP_W = W_IN - 3.0
lip_dep = fuse([sec_y(q, -LIP_W, LIP_W) for q in lip_quads] +
               [cylinder((pg.LIP_HINGE[0], -LIP_W, pg.LIP_HINGE[1]), (0, 1, 0), 3.0, 2 * LIP_W)] +
               [sec_y([(pg.LIP_HINGE[0] - 3.0, pg.LIP_HINGE[1]), (pg.LIP_HINGE[0] + 3.0, pg.LIP_HINGE[1]),
                       (pg.LIP_HINGE[0] + 3.0, prof[0][1] - LIP_RESERVE - 1.0),
                       (pg.LIP_HINGE[0] - 3.0, prof[0][1] - LIP_RESERVE - 1.0)], -LIP_W, LIP_W)])
lip_dep = sorted(solids(lip_dep), key=volume, reverse=True)
assert len(lip_dep) == 1
HINGE3 = (pg.LIP_HINGE[0], 0.0, pg.LIP_HINGE[1])
lip_stowed = rotated(lip_dep[0], HINGE3, (0, 1, 0), pg.LIP_STOW_DEG)
add("Deployable exit lip", lip_dep[0], group="exit lip", role="mechanism",
    function="Deploy-once polycarbonate exit lip: stowed folded up in front of the nip inside the frame, released at match start and swung "
             "120 deg forward and down by gravity and its spring to a 30 deg ramp and a 25 mm land whose tip is 100 mm beyond the frame line",
    parent=f"moving: revolute about +Y through ({pg.LIP_HINGE[0]}, {pg.LIP_HINGE[1]}) mm on hinge brackets on both side plates",
    motion=f"installed deployed (carry configuration); stowed for the start is {pg.LIP_STOW_DEG:.0f} deg about +Y",
    bounds=f"1/8 in plate reserved {LIP_RESERVE} mm under the deployed profile {[tuple(round(v, 1) for v in p) for p in prof]}, hinge knuckle radius 3.0 mm",
    missing=["Sprung deployed stop (two extension springs on a stop bar 30 mm from the hinge, 6 N m preload, 31 deg over-travel) is sized in "
             "structure-checks.json L1-L9 and not drawn; the trials treat the lip as rigid", "Deploy torsion spring article"])
lip_deployed_bounds = aabb(lip_dep[0])
# --- tray supports: rear uprights on the rear perimeter and two floor posts on the upper stringers
fb = lambda x: pg.floor_z(x) - (CONTACT_RESERVE + W_T) / c15
for s, label in ((1, "port"), (-1, "starboard")):
    y0, y1 = sorted((s * 170.0, s * 195.4))
    add(f"Rear inlet upright {label}", box([-368.0, y0, 205.6], [-343.0, y1, 800.0]), group="tray supports", role="structure",
        function=f"{label.capitalize()} 1 x 1 in upright carrying the inlet walls at the station face",
        parent="fixed: foot bracket on the rear perimeter tube", motion="fixed",
        bounds="25 x 25.4 mm tube reserve Z 205.6-800", missing=["Foot bracket to the rear perimeter and the wall clips", "Gusset"])
    top = min(fb(110.0), fb(135.0)) - G
    add(f"Floor post {label}", box([110.0, min(s * 140.0, s * 155.0), 491.7], [135.0, max(s * 140.0, s * 155.0), top]),
        group="tray supports", role="structure", function=f"{label.capitalize()} post from the upper pan stringer to the pocket floor",
        parent="fixed: on the upper pan stringer top (Z 489.5)", motion="fixed",
        bounds=f"25 x 15 mm post Z 491.7-{top:.1f}", missing=["Floor clip and stringer bolt"])

# ---------------------------------------------------------------- the finite pipe at its carry seat
pipe = cylinder((SEAT[0], -pg.PIPE_LENGTH / 2, SEAT[1]), (0, 1, 0), pg.R, pg.PIPE_LENGTH)
add("CORAL pipe", pipe, group="pipe", role="payload",
    function="One station-issued CORAL from the station chute through receive, carry, eject and the trough",
    parent=f"moving: installed pose is the carry seat, the 80 N/mm nominal receive end state (statically held: {trials['seatCreepMmOver3s']['80.0']} mm settling over a 3 s dwell)",
    motion="station -> chute -> ramp -> flap -> seat -> carry -> throat -> over the lower roller -> lip -> tip -> trough",
    bounds="Cylinder of the 114.3 mm OD, 301.625 mm pipe (bore filled)", missing=["CORAL geometry is not registered in kickoff/rules-v0.json"])

# ---------------------------------------------------------------- field: red L1 trough sector A at the shown dock
t = pg.TROUGH
xl = pg.trough_x(pg.NOMINAL_STANDOFF)
xr = xl + t["riserX"]
fl = lambda x: t["floor0"] + t["slope"] * (x - xl)
hw = t["sectorHalfWidth"]
trough = fuse([sec_y([(xl, 300.0), (xl + t["lipDepth"], 300.0), (xl + t["lipDepth"], t["lipTop"]), (xl, t["lipTop"])], -hw, hw),
               sec_y([(xl + t["lipDepth"] - 0.5, fl(xl + t["lipDepth"])), (xr + 0.5, fl(xr)), (xr + 0.5, fl(xr) - 6.35),
                      (xl + t["lipDepth"] - 0.5, fl(xl + t["lipDepth"]) - 6.35)], -hw, hw),
               sec_y([(xr, 300.0), (xr + t["riserT"], 300.0), (xr + t["riserT"], t["riserTop"]), (xr, t["riserTop"])], -hw, hw)])
add("L1 trough sector A (field)", sorted(solids(trough), key=volume, reverse=True)[0], group="field trough", role="other",
    function="Red reef L1 trough section at the shown dock: outer lip, 15 deg floor and inner riser, from the 2025 field model extraction",
    parent=f"field: lip outer face at X {xl:.1f} mm ({pg.NOMINAL_STANDOFF:.0f} mm stand-off from the bumper face), sector A |Y| <= {hw}",
    motion="field (recedes 300 mm in the robot frame during withdraw-park)",
    bounds="experiments/six-wheel/mission/target.json section (lip top 457.2, lip 12.7, floor 0.4049 + 0.268 x, riser 247.8-254.2, top 517.5) extruded over sector A",
    missing=["Reef structure below Z 300 mm and above the riser (L2 branch) is not placed", "Panel mounting holes and the sector divider"])

# ---------------------------------------------------------------- field: red coral station chute at the receive dock
ch = ft.station_chute(ft.NOMINAL_DOCK_GAP)
sa = math.radians(ft.STATION["slopeDeg"])
sn = (math.sin(sa), math.cos(sa))
CHUTE_T = 12.7
chute_quad = [ch[0], ch[1], (ch[1][0] - sn[0] * CHUTE_T, ch[1][1] - sn[1] * CHUTE_T), (ch[0][0] - sn[0] * CHUTE_T, ch[0][1] - sn[1] * CHUTE_T)]
STATION_HW = 170.0
station = fuse([sec_y(chute_quad, -STATION_HW, STATION_HW), box([ch[1][0] - CHUTE_T, -STATION_HW, 0.0], [ch[1][0], STATION_HW, ch[1][1] - 2.5])])
station = sorted(solids(station), key=volume, reverse=True)
assert len(station) == 1
add("Coral station chute (field)", station[0], group="field station", role="other",
    function="Red coral station at the receive dock: the 35 deg chute the pipe is released on and the station wall below its exit sill",
    parent=f"field: wall face at X {ch[1][0]:.1f} mm, {ft.NOMINAL_DOCK_GAP:.0f} mm behind the rear bumper face; exit sill at Z {ch[1][1]:.1f}",
    motion="field (present for the receive cases only)",
    bounds=f"Chute top surface on the trial polyline {[tuple(round(v, 1) for v in p) for p in ch]} (the true contact line, no reserve), 12.7 mm plate, "
           f"wall to the floor, {2 * STATION_HW:.0f} mm of the 1930 mm opening width",
    missing=["Opening upper guard (7 in opening) and side guides (121.99 mm mouth in the field STEP) are not placed",
             "Sill datum: the V0 manual gives the opening sill at 37.5 in; the chute exit is taken on the wall's field face"])
first = trials["eject"][next(p_["index"] for p_ in trials["playback"] if p_["id"] == "eject-nominal")]["finalFromLipMm"]
SECTOR_PITCH = 471.0
fp = cylinder((xl + first[0], -SECTOR_PITCH - pg.PIPE_LENGTH / 2, first[1] + 0.0), (0, 1, 0), pg.R - 3.5, pg.PIPE_LENGTH)
add("First scored pipe in sector A (field)", fp, group="field first pipe", role="other",
    function="The first pipe at its trial rest pose in sector A while the robot docks at sector B for the second delivery",
    parent=f"field: the nominal eject rest pose, {SECTOR_PITCH:.0f} mm toward -Y (one sector pitch)", motion="field (second-trip cases only)",
    bounds="Pipe cylinder at the trial rest pose, radius reduced 3.5 mm so its resting contact with the trough is not a clearance finding",
    missing=["Sector divider between A and B", "Lateral dock error: the pipe ends are 169.4 mm apart at nominal docks (premise lateral-dock-error)"])

# ================================================================ write STEP and manifest
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.Interface import Interface_Static

Interface_Static.SetCVal_s("write.step.unit", "MM")
OUT.mkdir(parents=True, exist_ok=True)
for fname, pick_ in (("package.step", lambda n: blocks[n]["group"] not in FIXTURE_GROUPS), ("field-fixtures.step", lambda n: blocks[n]["group"] in FIXTURE_GROUPS)):
    doc = TDocStd_Document(TCollection_ExtendedString("BinXCAF"))
    table = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    for name in order:
        if pick_(name):
            TDataStd_Name.Set_s(table.AddShape(blocks[name]["shape"], False), TCollection_ExtendedString(name))
    writer = STEPCAFControl_Writer()
    step_path = OUT / fname
    assert not step_path.exists(), fname
    assert writer.Transfer(doc, STEPControl_AsIs) and writer.Write(str(step_path)) == IFSelect_RetDone
# neighbours of every block new or moved in layout-002: exact distance to every other robot block
NEW = ["Nip drive (NEO 550 and MAXPlanetary 36:1)", "Nip coupling belt", "Seated-pipe beam emitter (port)", "Seated-pipe beam receiver (starboard)",
       "Flap-closed switch", "Reef range sensor (CANrange)", "Hood (removable)", "Coral station chute (field)"]
new_rows = []
for n in NEW:
    for o in order:
        if o == n or (blocks[o]["group"] in FIXTURE_GROUPS and blocks[n]["group"] in FIXTURE_GROUPS) or o == "CORAL pipe":
            continue
        if box_gap_(aabb(blocks[n]["shape"]), aabb(blocks[o]["shape"])) < 12.0:
            new_rows.append(dict(block=n, neighbour=o, gapMm=round(distance(blocks[n]["shape"], blocks[o]["shape"]), 3)))
for r_ in sorted(new_rows, key=lambda r: r["gapMm"])[:12]:
    print("neighbour", r_)
assert all(r_["gapMm"] >= 2.0 for r_ in new_rows), [r_ for r_ in new_rows if r_["gapMm"] < 2.0]

robot_names = [n for n in order if blocks[n]["group"] not in FIXTURE_GROUPS + ("breaker service hand", "plug service hand", "pipe", "bumpers")]
# sprung-stop over-travel: the deployed lip turned a further 31.2 deg down (frontal hit), against every other robot block
OVER_DEG = 31.2
lip_over = rotated(lip_dep[0], HINGE3, (0, 1, 0), OVER_DEG)
over_rows = [dict(neighbour=o, gapMm=round(distance(lip_over, blocks[o]["shape"]), 3)) for o in order
             if o not in ("Deployable exit lip", "CORAL pipe") and blocks[o]["group"] not in FIXTURE_GROUPS
             and box_gap_(aabb(lip_over), aabb(blocks[o]["shape"])) < 15.0]
over_min = min(r_["gapMm"] for r_ in over_rows)
print("lip over-travel min gap", over_min, sorted(over_rows, key=lambda r: r["gapMm"])[:3], "tip bounds", [round(v, 1) for v in aabb(lip_over)])
start_shape = {n: (lip_stowed if n == "Deployable exit lip" else blocks[n]["shape"]) for n in robot_names}
max_x_start = max(aabb(v)[3] for v in start_shape.values())
max_z_start = max(aabb(v)[5] for v in start_shape.values())
stowed_bounds = aabb(lip_stowed)
swing_max_x = max(aabb(rotated(lip_dep[0], HINGE3, (0, 1, 0), pg.LIP_STOW_DEG * f / 24.0))[3] for f in range(25))
# Unloaded-tread clearance of every non-pipe neighbour (the layout reserves only the rigid roller cores for the pipe)
treads = {nm: cylinder((c[0], -160.0, c[1]), (0, 1, 0), pg.ROLLER_R, 320.0) for nm, c in (("lower", pg.LOWER_ROLLER), ("upper", pg.UPPER_ROLLER))}
tread_rows = []
for n in order:
    if blocks[n]["group"] in ("pipe", "lower nip roller", "upper nip roller"):
        continue
    for nm, tr_ in treads.items():
        if box_gap_(aabb(blocks[n]["shape"]), aabb(tr_)) < 10.0:
            tread_rows.append(dict(block=n, tread=nm, pose="installed", gapMm=round(distance(blocks[n]["shape"], tr_), 3)))
for a_ in range(0, int(abs(pg.LIP_STOW_DEG)) + 1, 10):
    shp = rotated(lip_dep[0], HINGE3, (0, 1, 0), math.copysign(a_, pg.LIP_STOW_DEG))
    for nm, tr_ in treads.items():
        tread_rows.append(dict(block="Deployable exit lip", tread=nm, pose=f"swing {a_} deg toward stowed", gapMm=round(distance(shp, tr_), 3)))
tread_min = min(r["gapMm"] for r in tread_rows)
assert tread_min >= 2.0, sorted(tread_rows, key=lambda r: r["gapMm"])[:3]
manifest = dict(
    schema="red2/preliminary-layout-blocks@1", partitionMm=G,
    inputs={k: pin(v) for k, v in dict(chassisStep=CHASSIS_STEP, electrical=ELECTRICAL,
                                       bumperRecord="cad/rear-frame-facets/construction.json",
                                       pocketGeometry=f"{SRC}/pocket_geometry.py",
                                       fieldTrials=TRIALS.relative_to(ROOT).as_posix(),
                                       trough="experiments/six-wheel/mission/target.json").items()},
    generator={"file": f"{SRC}/build_layout.py", "sha256": sha(Path(__file__))},
    fixtureGroups=list(FIXTURE_GROUPS), beamMm=list(BEAM), sectorPitchMm=SECTOR_PITCH, firstPipeFromLipMm=first,
    stationChute=dict(polylineMm=[list(p) for p in ch], dockGapMm=ft.NOMINAL_DOCK_GAP, halfWidthMm=STATION_HW),
    newBlockNeighbours=sorted(new_rows, key=lambda r: r["gapMm"])[:20],
    lipOverTravel=dict(deg=OVER_DEG, minGapMm=over_min, rows=sorted(over_rows, key=lambda r: r["gapMm"])[:8], boundsMm=[round(v, 2) for v in aabb(lip_over)]),
    frameOutlineMm=OUTLINE, rearCutbackMm=CUTBACK,
    poses=dict(seatMm=list(SEAT), throatMm=[pg.THROAT_X, pg.POCKET_Z], lowerRollerMm=list(pg.LOWER_ROLLER),
               upperRollerMm=list(pg.UPPER_ROLLER), coreRadiusMm=pg.CORE_R, inlet=pg.INLET, kink=list(pg.KINK),
               flapHingeMm=list(pg.FLAP_HINGE), flapLengthMm=pg.FLAP_LENGTH, flapOpenDeg=pg.FLAP_OPEN_DEG,
               hood=pg.HOOD, hoodFlare=pg.HOOD_FLARE, hoodGapMm=pg.HOOD_GAP, lipHingeMm=list(pg.LIP_HINGE), lipProfile=prof, lipDeployDeg=pg.LIP_DEPLOY_DEG,
               troughLipXMm=xl, standOffMm=pg.NOMINAL_STANDOFF, contactReserveMm=CONTACT_RESERVE, lipReserveMm=LIP_RESERVE,
               partitionMm=dict(default=G, besideMoving=GM, steerToHousing=GT), steer={str(k): v for k, v in turret_bands.items()}),
    treadClearance=dict(minGapMm=tread_min, rows=tread_rows,
                        basis="Distance of every non-pipe reserve to the unloaded 2 in tread cylinders (|Y| <= 160), installed and along the lip's stow swing"),
    rules=dict(deployedLipBoundsMm=[round(v, 3) for v in lip_deployed_bounds], stowedLipBoundsMm=[round(v, 3) for v in stowed_bounds],
               stowedLipInsideFrameMm=round(pg.FRAME_FRONT - stowed_bounds[3], 3),
               deploySwingMaxProjectionMm=round(swing_max_x - pg.FRAME_FRONT, 3),
               deployedProjectionBeyondFrameMm=round(lip_deployed_bounds[3] - pg.FRAME_FRONT, 3),
               startMaxXMm=round(max_x_start, 3), startProjectionBeyondFrontMm=round(max_x_start - pg.FRAME_FRONT, 3),
               startMaxZMm=round(max_z_start, 3), startExcludes="bumpers, field fixtures, service hands and the pipe; the lip in its stowed pose"),
    blocks=[dict(name=n, **{k: v for k, v in blocks[n].items() if k != "shape"},
                 boundsMm=[round(x, 3) for x in aabb(blocks[n]["shape"])], volumeMm3=round(volume(blocks[n]["shape"]), 1))
            for n in order])
(OUT / "layout-blocks.json").write_text(json.dumps(manifest, indent=1) + "\n", encoding="utf-8")
print(len(order), "blocks; tread min", tread_min, "stowed inside", round(pg.FRAME_FRONT - stowed_bounds[3], 2), "seat", SEAT, "lip deployed max X", round(lip_deployed_bounds[3], 2), "start max X", round(max_x_start, 2),
      "max Z", round(max_z_start, 2))
