"""Write the motion study, budgets, premises, records and layout plan for the fixed-pocket preliminary design (Fixed pocket 1.0).

    python -B write_package.py PROJECT_ROOT PLUGIN_SCRIPTS [RELEASE_DIR] [VIEW]

VIEW is the review page the layout check writes, project-relative (the plugin currently wants it beside the project
root; omit it to check without a page).

PROJECT_ROOT is the repository root (the project root of the layout check). RELEASE_DIR, project-relative, defaults to
cad/preliminary-design/releases/1.0 and already holds field/*.step and evidence/official-fixtures.json
(cad/field/official/checked_fixtures.py), design.json, package.step, scored-pipe.step, layout-blocks.json (build_layout.py),
evidence/field-trials.json (field_trials.py) and evidence/structure-checks.json (structure_checks.py). Every pin is project-relative.

Every mission case's pipe and flap motion is a chord of a solved trial trace (receive-mission, custody-power-off-hit and
eject-nominal in field-trials.json): the trace is reduced to straight strokes that stay within CHORD_TOL of it, split at each change of
support, and each case records its trace window and its largest deviation. Only the lip deploy (a hinge rotation, timed
by structure-checks.json L8) and the final park drive are prescribed, and they say so.
"""
import hashlib
import itertools
import json
import math
import sys
from pathlib import Path

ROOT = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(Path(sys.argv[2]).resolve()))
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from showcase_contacts import load_solids, box_gap  # noqa: E402
from layout_occ import aabb, distance, moved, rotated  # noqa: E402
import pocket_geometry as pg  # noqa: E402
import field_trials as ft  # noqa: E402

SRC = "iterations/002/design/preliminary-pocket"
REL = sys.argv[3] if len(sys.argv) > 3 else "cad/preliminary-design/releases/1.0"
VIEW = sys.argv[4] if len(sys.argv) > 4 else None
OUT = ROOT / REL
ISSUE = "fixed-pocket-1.0"
GAP = 2.0
R = pg.R
CHORD_TOL = 10.0     # mm: largest distance of a case stroke from the solved trace (the coarse whole-cycle tolerance)
# A phase listed here was unresolved at the coarse tolerance (a pair closer than the tolerance) and is split finer: phase -> mm.
REFINED_TOL = {}
TOL_NOTE = "".join(f" ({v:g} mm in the {k} phase)" for k, v in REFINED_TOL.items())
FLAP_TOL = 10.0      # deg: largest flap-angle error along a stroke


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pin(rel):
    return {"file": rel}


def write(rel, obj):
    path = ROOT / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise FileExistsError(rel)
    path.write_text(json.dumps(obj, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return pin(rel)


def read(rel):
    return json.loads((ROOT / rel).read_text(encoding="utf-8-sig"))


blocks = read(f"{REL}/layout-blocks.json")
poses = blocks["poses"]
trials_pin = pin(f"{REL}/evidence/field-trials.json")
trials = read(trials_pin["file"])
S = trials["summary"]
struct_pin = pin(f"{REL}/evidence/structure-checks.json")
struct = read(struct_pin["file"])
SC = {r["id"]: r for r in struct["rows"]}
assert not struct["failing"], struct["failing"]
frac = lambda t_: f"{t_['ok']}/{t_['n']}"
EJ, RC, SL = trials["eject"], trials["receive"], trials["sled"]
BAND = S["acceptanceBandNmm"]
design = pin(f"{REL}/design.json")
step = pin(f"{REL}/package.step")
pipe_step = pin(f"{REL}/scored-pipe.step")
official_pin = pin(f"{REL}/evidence/official-fixtures.json")
OFFICIAL = read(official_pin["file"])
OFF_ROLE = {r["role"]: r["name"] for r in OFFICIAL["parts"]}
OFFICIAL_SOURCE = f"FIRST official field model ({OFFICIAL['source']['url']}; {OFFICIAL['source']['edition']})"
datum = dict(name="Iteration 002 robot frame = taps-02 root: chassis centre on the factory-tire floor (Z = 0), "
                  "+X toward the reef (delivery) face, +Y port, +Z up, millimetres",
             robotToStepMm=[1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])
FIXTURE_GROUPS = tuple(blocks["fixtureGroups"])
by_group = {}
for b in blocks["blocks"]:
    by_group.setdefault(b["group"], []).append(b["name"])
roles = {b["group"]: b["role"] for b in blocks["blocks"]}
groups = {g: dict(role=roles[g], members=m) for g, m in by_group.items()}
by_name = {b["name"]: b for b in blocks["blocks"]}
groups_of = lambda n: by_name[n]["group"] if n in by_name else None

# ------------------------------------------------------------------ the two mission traces
pb = {p["id"]: p for p in trials["playback"]}
play = lambda pid: trials[pb[pid]["kind"]][pb[pid]["index"]]
REC, EJN = play("receive-mission"), play("eject-nominal")
assert REC["outcome"] == "seated" and EJN["outcome"].startswith("captured") and EJN["standOffMm"] == pg.NOMINAL_STANDOFF
SEAT = tuple(poses["seatMm"])
assert math.dist(SEAT, EJN["trace"][0][1:3]) < 0.05 and math.dist(SEAT, REC["final"]) < 0.2, (SEAT, EJN["trace"][0], REC["final"])
t = pg.TROUGH
xl = pg.trough_x(pg.NOMINAL_STANDOFF)
xr = xl + t["riserX"]
fl = lambda x: t["floor0"] + t["slope"] * (x - xl)
trough_segs = [((xl, 300.0), (xl, t["lipTop"])), ((xl, t["lipTop"]), (xl + t["lipDepth"], t["lipTop"])),
               ((xl + t["lipDepth"], fl(xl + t["lipDepth"])), (xl + t["lipDepth"], t["lipTop"])),
               ((xl + t["lipDepth"], fl(xl + t["lipDepth"])), (xr, fl(xr))), ((xr, fl(xr)), (xr, t["riserTop"])),
               ((xr, t["riserTop"]), (xr + t["riserT"], t["riserTop"]))]


def seg_dist(p, a, b):
    dx, dz = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dz * dz
    u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dz) / L2))
    return math.dist(p, (a[0] + u * dx, a[1] + u * dz))


def reduce_trace(pts, lo, hi, tol):
    """Indices lo..hi of pts (t, x, z, flapDeg) kept so every stroke stays within tol mm and FLAP_TOL of the trace."""
    keep = {lo, hi}
    stack = [(lo, hi)]
    while stack:
        i, j = stack.pop()
        if j - i < 2:
            continue
        a, b = pts[i], pts[j]
        run = [0.0]
        for k in range(i, j):
            run.append(run[-1] + math.dist(pts[k][1:3], pts[k + 1][1:3]))
        worst, wk = 0.0, None
        for k in range(i + 1, j):
            e = seg_dist(pts[k][1:3], a[1:3], b[1:3]) / tol
            f = (run[k - i] / run[-1]) if run[-1] > 0 else 0.0
            e = max(e, abs(pts[k][3] - (a[3] + f * (b[3] - a[3]))) / FLAP_TOL)
            if e > worst:
                worst, wk = e, k
        if worst <= 1.0 and (i, j) == (lo, hi) and math.dist(a[1:3], b[1:3]) < 0.5:
            # an excursion that returns to its start (a hit shorter than the tolerance): keep its far point so the case has travel
            wk = max(range(i + 1, j), key=lambda k: math.dist(pts[k][1:3], a[1:3]))
            worst = 2.0
        if worst > 1.0:
            keep.add(wk)
            stack += [(i, wk), (wk, j)]
    return sorted(keep)


def first_index(pts, pred, start=0):
    return next(k for k in range(start, len(pts)) if pred(pts[k]))


ur = (math.cos(math.radians(-pg.FLOOR_DEG)), math.sin(math.radians(-pg.FLOOR_DEG)))      # down the pocket floor
nr = (math.sin(math.radians(pg.FLOOR_DEG)), math.cos(math.radians(pg.FLOOR_DEG)))        # floor normal, up

# ------------------------------------------------------------------ motion cases (robot frame)
def rot(origin, deg):
    return dict(kind="rotation", originMm=[float(origin[0]), 0.0, float(origin[1])], axis=[0.0, 1.0, 0.0], travelDeg=float(deg))


def tr(a, b):
    return dict(kind="translation", travelMm=[float(b[0] - a[0]), 0.0, float(b[1] - a[1])])


def tr3(v):
    return dict(kind="translation", travelMm=[float(x) for x in v])


pipe, lip, flap, hood, lower = "CORAL pipe", "Deployable exit lip", "Check flap", "Hood (removable)", "Lower nip roller"
first_pipe = "First scored pipe in sector A (field)"
# The official parts the pipe touches: the chute plate at the station, and the lip, floor and riser of the docked trough.
station = OFF_ROLE["chute"]
TROUGH_PARTS = [OFF_ROLE["lip"], OFF_ROLE["floor"], OFF_ROLE["riser"]]
trough = OFF_ROLE["lip"]
FLAP_SW = "Flap-closed switch"
FL = (pg.FLAP_HINGE[0], pg.FLAP_HINGE[1])
HINGE = pg.LIP_HINGE
STATION_SET, REEF_SET = list(by_group["field station"]), list(by_group["field reef"])
FIELD_SET = STATION_SET + REEF_SET + [first_pipe]
FIX = {n: dict(id=n, file=pin(f"{REL}/{by_name[n]['fixtureFile']}"), solid=n,
               basis=f"Field fixture: {by_name[n]['function']}. {OFFICIAL_SOURCE}. The official solid, placed at the "
                     + (f"receive dock with the lower barrier {ft.NOMINAL_DOCK_GAP:.0f} mm behind the rear bumper face."
                        if n in STATION_SET else f"delivery dock with the trough lip face {pg.NOMINAL_STANDOFF:.0f} mm ahead of the front bumper face."))
       for n in STATION_SET + REEF_SET}
FIX[first_pipe] = dict(id=first_pipe, file=pipe_step, solid=first_pipe,
                       basis="Field fixture for the second trip: the first pipe at its trial rest pose in the next sector, one 471 mm sector pitch toward -Y, while "
                             "the robot delivers at the docked sector. The eject cases are the same robot-frame motion at either sector, so the fixture is shown in "
                             "them; on the first trip it is absent.")
cases, timelines, case_meta = [], {}, {}
state = {pipe: None, flap: 0.0, lip: pg.LIP_STOW_DEG, trough: 0.0}     # pipe centre (mm), flap open deg, lip deg, trough X offset


def initial_pose():
    init = {}
    if state[pipe] is not None and math.dist(state[pipe], SEAT) > 0:
        init[pipe] = [tr(SEAT, state[pipe])]
    if state[flap]:
        init[flap] = [rot(FL, -state[flap])]
    if state[lip]:
        init[lip] = [rot(HINGE, state[lip])]
    if state[trough]:
        for n in REEF_SET:
            init[n] = [tr3([state[trough], 0, 0])]
    return init


def case(id_, stage, basis, moves, fixtures=(), maxDepth=None, **extra):
    c = dict(id=id_, stage=stage, minGapMm=GAP, basis=basis)
    if maxDepth:
        c["maxDepth"] = maxDepth
    init = initial_pose()
    if not any(f == trough for f in fixtures):
        for n in REEF_SET:
            init.pop(n, None)
    if init:
        c["initialPose"] = init
    c["moves"] = moves
    c.update(extra)
    cases.append(c)
    return c


def trace_cases(prefix, rec, pts, phases, fixtures, snap_end=None):
    """One case per reduced stroke of pts (t, x, z, flapDeg) inside each (name, i0, i1, spec) phase."""
    n = 0
    for name, i0, i1, spec in phases:
        if i1 <= i0:
            continue
        tol = REFINED_TOL.get(name, CHORD_TOL)
        idx = reduce_trace(pts, i0, i1, tol)
        for a, b in zip(idx, idx[1:]):
            n += 1
            cid = f"{prefix}-{n:02d}-{name}"
            pa, pb_ = pts[a], pts[b]
            end = (pb_[1], pb_[2])
            snap = 0.0
            if snap_end is not None and b == len(pts) - 1:
                snap, end = math.dist(end, snap_end), snap_end
            start = state[pipe]
            moves = {pipe: [tr(start, end)]}
            phi = pb_[3] if abs(pb_[3]) > 0.05 else 0.0
            if phi != state[flap]:
                moves[flap] = [rot(FL, -(phi - state[flap]))]
            dev = max((seg_dist(pts[k][1:3], start, end) for k in range(a, b + 1)), default=0.0)
            basis = (f"{spec['what']} Trace {rec} of field-trials.json, {pa[0]:.3f}-{pb_[0]:.3f} s: one straight stroke between two solved poses, "
                     f"at most {dev:.2f} mm from the solved path" + (f" (end moved {snap:.3f} mm onto the carry seat the eject trials start from)" if snap else "")
                     + ("; the flap turns with it from the same trace" if flap in moves else "") + ".")
            c = case(cid, "operating", basis, moves, fixtures=fixtures, maxDepth=11)
            frames = 25
            times = [round(pa[0] + (pb_[0] - pa[0]) * k / (frames - 1), 4) for k in range(frames)]
            timelines[cid] = dict(time=times, measure=[
                f"{spec['label']} · trial time {tm:.3f} s · pipe centre X {start[0] + (end[0] - start[0]) * k / (frames - 1):.0f}, "
                f"Z {start[1] + (end[1] - start[1]) * k / (frames - 1):.0f} mm" for k, tm in enumerate(times)])
            case_meta[cid] = dict(spec=spec, playback=rec, windowS=[pa[0], pb_[0]], maxDeviationMm=round(dev, 3), snapMm=round(snap, 4), tolMm=tol)
            state[pipe] = end
            state[flap] = phi
    return n


HOLD = "Held by the powered position hold: NEO 550 through the MAXPlanetary 36:1, 30 A limit"
TRAY = "Gravity on the fixed pocket floor"
FLAPD = "Spring check flap: the pipe pushes it open; closed, its rear stop holds the pipe"
SPEC = dict(
    chute=dict(kind="free", label="On the station chute", contacts=[station],
               what="The station-issued pipe, released from rest, rolls down the registered station chute.",
               basis="The pipe is on the field's chute: no robot mechanism carries it"),
    drop=dict(kind="free", label="Dropping from the station sill", contacts=[station, "Pocket tray"],
              what="The pipe leaves the station sill and falls toward the robot inlet.", basis="The pipe is in free flight between the station and the inlet"),
    inlet=dict(kind="transport", label="Landing and rolling down the inlet", contacts=["Pocket tray"], mechanisms=["pocket tray"],
               drive={"pocket tray": "Gravity on the fixed inlet chute and floor"},
               what="The pipe lands on the inlet and rolls down the 15.74 deg floor to the check flap.", basis="The pipe lands and rolls on the fixed tray under gravity"),
    pocket=dict(kind="transport", label="Through the check flap into the bite", contacts=["Pocket tray", flap],
                mechanisms=["pocket tray", "check flap", "lower nip roller", "upper nip roller"],
                drive={"pocket tray": TRAY, "check flap": FLAPD, "lower nip roller": HOLD, "upper nip roller": HOLD + ", through the coupling belt"},
                what="The pipe pushes the check flap open, meets the held nip and the flap closes behind it.",
                basis="The pipe rolls under the flap into the bite of the held nip"),
    captive=dict(kind="transport", label="Captive: flap closed, robot still docked", contacts=["Pocket tray", flap],
                 mechanisms=["pocket tray", "check flap", "lower nip roller", "upper nip roller"],
                 drive={"pocket tray": TRAY, "check flap": FLAPD, "lower nip roller": HOLD, "upper nip roller": HOLD + ", through the coupling belt"},
                 what=f"The flap is closed: the pipe is captive and the flap-closed switch releases the robot {ft.SENSOR_LATENCY * 1e3:.0f} ms later.",
                 basis="The pipe rebounds between the held nip and the closed flap"),
    away=dict(kind="transport", label="Driving away at 3 m/s2", contacts=["Pocket tray", flap],
              mechanisms=["pocket tray", "check flap", "lower nip roller", "upper nip roller"],
              drive={"pocket tray": TRAY, "check flap": FLAPD, "lower nip roller": HOLD, "upper nip roller": HOLD + ", through the coupling belt"},
              what="The robot accelerates toward the reef at 3 m/s2 and cruises; the pipe rides back against the closed flap.",
              basis="Chassis acceleration presses the pipe on the flap's rear stop"),
    brake=dict(kind="transport", label="Braking at the reef", contacts=["Pocket tray", flap],
               mechanisms=["pocket tray", "check flap", "lower nip roller", "upper nip roller"],
               drive={"pocket tray": TRAY, "check flap": FLAPD, "lower nip roller": HOLD, "upper nip roller": HOLD + ", through the coupling belt"},
               what="The robot brakes at 3 m/s2; the pipe rolls forward into the held nip.", basis="Braking rolls the pipe down the floor into the bite"),
    seat=dict(kind="transport", label="Settling on the carry seat", contacts=["Pocket tray", flap],
              mechanisms=["pocket tray", "check flap", "lower nip roller", "upper nip roller"],
              drive={"pocket tray": TRAY, "check flap": FLAPD, "lower nip roller": HOLD, "upper nip roller": HOLD + ", through the coupling belt"},
              what="The robot is stopped; the pipe settles on its carry seat and the seated-pipe beam is blocked.", basis="The pipe settles in the bite of the held nip"),
    poweroff=dict(kind="transport", label="Hit with the main power off", contacts=["Pocket tray"],
                  mechanisms=["pocket tray", "lower nip roller", "upper nip roller"],
                  drive={"pocket tray": TRAY, "lower nip roller": "Unpowered: the 6 mm tread squeeze holds the pipe and the coasting gearbox adds only its inertia",
                         "upper nip roller": "Unpowered, coupled to the lower shaft by the belt"},
                  what="Carrying with the main power off, the robot is hit: 8 g for 50 ms drives the pipe into the unpowered nip; the squeeze stops it and returns it to the seat.",
                  basis="The pipe climbs part of the tread squeeze and rolls back to its seat with no power on the nip"),
    nip=dict(kind="transport", label="Driven through the nip", contacts=["Pocket tray"], mechanisms=["lower nip roller", "upper nip roller"],
             drive={"lower nip roller": "NEO 550 through the MAXPlanetary 36:1 at 12 V open loop, 30 A limit",
                    "upper nip roller": "Counter-rotated from the lower shaft by the coupling belt"},
             what="Docked at the reef, the interlock passes and the nip draws the pipe through the 12 mm squeeze (roller cores stay 3 mm clear).",
             basis="Both treads squeeze and drive the pipe through the throat"),
    lip=dict(kind="transport", label="Over the lower roller and down the lip", contacts=[lip], mechanisms=["lower nip roller", "exit lip"],
             drive={"lower nip roller": "NEO 550 through the MAXPlanetary 36:1 at 12 V open loop", "exit lip": "Gravity on the deployed 30 deg ramp and land"},
             what="The pipe rolls over the driven lower roller onto the deployed lip and down its ramp to the tip.",
             basis="The driven lower roller hands the pipe to the deployed lip, which it rolls down"),
    flight=dict(kind="free", label="Off the tip toward the trough", contacts=[lip, *TROUGH_PARTS], what="The pipe leaves the lip tip and drops toward the trough.",
                basis="The pipe has left the lip and falls free"),
    trough=dict(kind="free", label="Landing and coming to rest in the trough", contacts=TROUGH_PARTS,
                what="The pipe lands in the trough, runs to the riser and rolls back to rest against the outer lip.",
                basis="The pipe is in the field's trough: no robot mechanism touches it"),
)

# lip deploy (prescribed hinge rotation, timed by the structure calculation); the pipe waits on the station chute
rec_pts = [(q[0], q[1], q[2], q[4]) for q in REC["trace"]]
state[pipe] = (rec_pts[0][1], rec_pts[0][2])
c = case("lip-deploy", "operating", "Prescribed hinge rotation, not a trial trace: at match start the servo pin releases the upright stowed lip and its torsion "
         f"spring and gravity swing it 120 deg onto the sprung deployed stop in {struct['lip']['deployS']:.2f} s (structure-checks.json L8, a rigid-body "
         "integration); the pipe is still in the station", {lip: [rot(HINGE, pg.LIP_DEPLOY_DEG)]}, fixtures=(station,), maxDepth=11)
timelines["lip-deploy"] = dict(time=[round(struct["lip"]["deployS"] * k / 24, 4) for k in range(25)],
                               measure=[f"Lip {120 * k / 24:.0f} deg of its 120 deg deploy swing (prescribed; duration from the L8 calculation)" for k in range(25)])
state[lip] = 0.0
sill_x = ft.station_chute(REC["dockGapMm"])[1][0]
i_exit = first_index(rec_pts, lambda q: q[1] > sill_x)
i_land = first_index(rec_pts, lambda q: min(seg_dist(q[1:3], a, b) for a, b in zip(pg.INLET, pg.INLET[1:])) <= R + 0.5, i_exit)
i_flap = min(first_index(rec_pts, lambda q: q[0] >= REC["passedFlapS"]), first_index(rec_pts, lambda q: q[3] > 0.05) - 1)   # first touch of the flap
i_closed = first_index(rec_pts, lambda q: q[0] >= REC["flapClosedS"])
i_go = first_index(rec_pts, lambda q: q[0] >= REC["goAtS"])
i_brake = first_index(rec_pts, lambda q: q[0] >= REC["goAtS"] + REC["go"]["accelS"] + REC["go"]["cruiseS"])
i_stop = first_index(rec_pts, lambda q: q[0] >= REC["stopAtS"])
n_rec = trace_cases("receive", "receive-mission", rec_pts,
                    [("station-chute", 0, i_exit, SPEC["chute"]), ("drop", i_exit, i_land, SPEC["drop"]), ("inlet", i_land, i_flap, SPEC["inlet"]),
                     ("pocket", i_flap, i_closed, SPEC["pocket"]), ("captive", i_closed, i_go, SPEC["captive"]), ("drive-away", i_go, i_brake, SPEC["away"]),
                     ("brake", i_brake, i_stop, SPEC["brake"]), ("seat", i_stop, len(rec_pts) - 1, SPEC["seat"])], fixtures=(station,), snap_end=SEAT)
assert state[pipe] == SEAT and state[flap] == 0.0, state
CUH = play("custody-power-off-hit")
assert CUH["outcome"] == "retained" and CUH["nipState"] == "main power off" and math.dist(SEAT, CUH["trace"][0][1:3]) < 0.2, CUH["outcome"]
cu_pts = [(q[0], q[1], q[2], q[4]) for q in CUH["trace"]]
n_cu = trace_cases("carry", "custody-power-off-hit", cu_pts, [("power-off-hit", 0, len(cu_pts) - 1, SPEC["poweroff"])], fixtures=(station,), snap_end=SEAT)
assert state[pipe] == SEAT and state[flap] == 0.0, state
ej_all = [(q[0], q[1] - q[5], q[2], 0.0, q[5]) for q in EJN["trace"]]
i_wd = first_index(ej_all, lambda q: q[4] != 0.0) - 1
ej_pts = [q[:4] for q in ej_all[:i_wd + 1]]
i_nip = first_index(ej_pts, lambda q: q[1] > pg.THROAT_X + 25.0)
i_tip = first_index(ej_pts, lambda q: q[0] >= EJN["departure"]["t"])
i_tr = first_index(ej_pts, lambda q: min(seg_dist(q[1:3], a, b) for a, b in trough_segs) <= R + 0.5, i_tip)
n_ej = trace_cases("eject", "eject-nominal", ej_pts,
                   [("nip", 0, i_nip, SPEC["nip"]), ("lip", i_nip, i_tip, SPEC["lip"]), ("flight", i_tip, i_tr, SPEC["flight"]),
                    ("trough", i_tr, len(ej_pts) - 1, SPEC["trough"])], fixtures=(trough, first_pipe))
REST = state[pipe]
case("withdraw-park", "park", "Prescribed robot drive, not a mechanism trial: the robot backs 300 mm away and parks with the lip deployed and the pocket empty; in the "
     "robot frame the scored pipe and the trough recede together. The eject trials withdraw the first 120 mm at 0.3 m/s and assess the pipe 2 s after "
     "it is again at rest", {n: [tr3([300, 0, 0])] for n in [pipe, *REEF_SET, first_pipe]}, fixtures=(trough, first_pipe))
cycle = [c["id"] for c in cases]
TS_ = {s_["id"]: s_ for s_ in trials["timestepSensitivity"]}
# Receive bracket on tread stiffness, read from the trials: the stiffest swept value below the band at which any receive
# fails, and the softest above it at which every receive seats.
_RSB = S["receiveStiffnessBracket"]
RECEIVE_K_FAIL = max(int(k) for k, v in _RSB.items() if v["ok"] < v["n"])
RECEIVE_K_PASS = min(int(k) for k, v in _RSB.items() if v["ok"] == v["n"] and int(k) > RECEIVE_K_FAIL)
QUAL = {
    "receive-mission": dict(
        result="pass", control=dict(condition=f"Check flap removed, same receive and go: {frac(S['goNoFlap'])} retained", rejected=True),
        brackets=[dict(input="tread stiffness per roller", unit="N/mm", passing=RECEIVE_K_PASS, failing=RECEIVE_K_FAIL),
                  dict(input="check flap", unit="present", passing=1, failing=0)],
        timestep=dict(coarseS=trials["dtS"], fineS=TS_["receive-mission"]["dtS"], sameOutcome=TS_["receive-mission"]["sameOutcome"])),
    "eject-nominal": dict(
        result="pass", control=dict(condition=f"Latch failed, lip still stowed: {frac(S['latchFailed'])} delivered; robot disabled: {frac(S['ejectDisabled'])} delivered", rejected=True),
        brackets=[dict(input="dock stand-off", unit="mm", passing=100, failing=125),
                  dict(input="tread stiffness per roller", unit="N/mm", passing=160, failing=240),
                  dict(input="lip tip beyond the frame line at 100 mm stand-off", unit="mm", passing=100, failing=72)],
        timestep=dict(coarseS=trials["dtS"], fineS=TS_["eject-nominal"]["dtS"], sameOutcome=TS_["eject-nominal"]["sameOutcome"])),
}
CU = S["custody"]
OFF, DEC = "main power off", "control: rollers decoupled from the drive"
CU_CONTROL = play("custody-control-decoupled-soft")
soft = str(int(S["bandNmm"][0]))
QUAL["custody-power-off-hit"] = dict(
    result="pass",
    control=dict(condition=f"Rollers decoupled from the drive on a 20 N/mm tread (below the band), same 8 g hit: {CU_CONTROL['outcome']}", rejected=True),
    brackets=[dict(input=f"50 ms hit into the nip, main power off, {soft} N/mm tread", unit="g", passing=CU["hitLimitsG"][OFF][soft]["largestRetainedG"],
                   failing=CU["hitLimitsG"][OFF][soft]["smallestLostG"]),
              dict(input=f"0.5 s deceleration into the nip, main power off, {soft} N/mm tread", unit="g", passing=CU["sustainedLimitsG"][OFF][soft]["largestRetainedG"],
                   failing=CU["sustainedLimitsG"][OFF][soft]["smallestLostG"]),
              dict(input="tread stiffness per roller, rollers decoupled, 8 g hit", unit="N/mm", passing=40, failing=20)],
    timestep=dict(coarseS=trials["dtS"], fineS=TS_["custody-power-off-hit"]["dtS"], sameOutcome=TS_["custody-power-off-hit"]["sameOutcome"]))
assert CU_CONTROL["outcome"] != "retained" and CU["powerOffRequired"]["ok"] == CU["powerOffRequired"]["n"] > 0, CU["powerOffRequired"]
assert all(b["passing"] is not None and b["failing"] is not None for b in QUAL["custody-power-off-hit"]["brackets"])
assert CU["hitLimitsG"][DEC]["40"]["largestRetainedG"] >= 8.0 and CU["hitLimitsG"][DEC]["20"]["smallestLostG"] == 8.0
assert CU["sustainedLimitsG"][OFF][soft]["largestRetainedG"] >= 2 * CU["required"]["stepG"] and CU["hitLimitsG"][OFF][soft]["largestRetainedG"] >= CU["required"]["pulseG"]
assert all(TS_[i]["sameOutcome"] for i in TS_ if i.startswith("custody-")), [i for i in TS_ if i.startswith("custody-") and not TS_[i]["sameOutcome"]]
assert all(q["timestep"]["sameOutcome"] for q in QUAL.values())
assert S["goNoFlap"]["ok"] == 0 and S["latchFailed"]["ok"] == 0 and S["standOff125"]["ok"] < S["standOff125"]["n"]
RECEIVE_IDS = [c for c in cycle if c.startswith("receive-")]
EJECT_IDS = [c for c in cycle if c.startswith("eject-")]
CARRY_IDS = [c for c in cycle if c.startswith("carry-")]
MISSION = dict(acquired=next(c for c in RECEIVE_IDS if c.endswith("-seat") and c == RECEIVE_IDS[-1]), delivered=EJECT_IDS[-1],
               ejectLoaded=next(c for c in EJECT_IDS if "-lip" in c))

# service cases start from the installed (carry-configuration) assembly: pipe at its carry seat, lip deployed.
# Service lifts are checked to clear the robot. They are not registered as extractions: an extraction must leave the box
# around everything else in the scene, the field fixtures are in every case's roster and reach 1.83 m, and an extraction
# case may not declare the official field's own joints. So each lifted group rises clear of the robot, and the cases after
# it start with that group set down on the floor beside the robot.
PIV = (SEAT[0] + 250.0 * math.sin(math.radians(pg.FLOOR_DEG)), SEAT[1] + 250.0 * math.cos(math.radians(pg.FLOOR_DEG)))
HOOD_SET = [hood, flap, FLAP_SW]
HOOD_LIFT, PIPE_LIFT, CASSETTE_LIFT = 400.0, 150.0, 520.0
low = lambda names: min(by_name[n]["boundsMm"][2] for n in names)
ASIDE_Y = 900.0          # beside the robot: outside the bumpers, between the station behind and the reef ahead
hood_aside = {n: [tr3([0, ASIDE_Y, 10.0 - low(HOOD_SET)])] for n in HOOD_SET}
pipe_aside = {pipe: [tr3([0, -ASIDE_Y, 10.0 - low([pipe])])]}
# The check needs one full service withdrawal: a group that ends clear of the box around everything else in the scene. An
# extraction case may declare no intended interface, and the official field parts sit closer to each other than the 2.0 mm
# reserve where FIRST joins them, so this one case is checked for interference only (0 mm). The field fixtures are in every
# case's roster and the reef branch reaches 1.83 m, so the hood is drawn straight up to above it. service-hood-lift checks
# the first 400 mm of the same lift with the 2.0 mm reserve.
HOOD_OUT = math.ceil((max(by_name[n]["boundsMm"][5] for n in by_name) - low(HOOD_SET) + 20.0) / 10.0) * 10.0
cases.append(dict(id="service-hood-withdrawal", stage="service", minGapMm=0.0,
                  basis=f"Full service withdrawal: the hood, check flap and flap-closed switch lifted straight up {HOOD_OUT:.0f} mm, above everything in the "
                        "checked scene (the official reef branch reaches 1.83 m). Interference only (0 mm reserve): the official field parts are closer to each "
                        "other than 2.0 mm where FIRST joins them and an extraction case may declare no interface. service-hood-lift checks the first "
                        f"{HOOD_LIFT:.0f} mm of the same lift with the 2.0 mm reserve",
                  moves={n: [tr3([0, 0, HOOD_OUT])] for n in HOOD_SET}, extract=HOOD_SET))
cases.append(dict(id="service-hood-lift", stage="service", minGapMm=GAP,
                  basis=f"Pit unload, step 1: release the thumb screws and lift the hood, with the check flap on its pin and the flap-closed switch on its top, {HOOD_LIFT:.0f} mm: "
                        "clear of the whole robot after 320 mm. It is then set down beside the robot, where the later service cases show it",
                  moves={n: [tr3([0, 0, HOOD_LIFT])] for n in HOOD_SET}))
cases.append(dict(id="service-unload-pipe", stage="service", minGapMm=GAP,
                  basis="Pit unload, step 2, with the hood set down beside the robot: roll the unscored pipe back up the floor and lift it out on one hand arc "
                        f"(110 deg about an axis 250 mm along the seat normal), clear of the upper roller and above the robot, then {PIPE_LIFT:.0f} mm straight up",
                  initialPose=dict(hood_aside), moves={pipe: [rot(PIV, 110.0), tr3([0, 0, PIPE_LIFT])]}, maxDepth=11))
cassette = by_group["nip side plates"] + by_group["lower nip roller"] + by_group["upper nip roller"] + by_group["nip drive"] + \
    by_group["nip coupling"] + by_group["lip latch"] + by_group["exit lip"] + by_group["lip sensor"] + by_group["pipe sensor"] + by_group["reef sensor"]
cases.append(dict(id="service-nip-cassette-lift", stage="service", minGapMm=GAP,
                  basis="With the hood and pipe out and set down beside the robot, unbolt the two side plates from the upper stringers and lift the nip "
                        f"cassette (plates, rollers, drive, belt, latch, sensors and deployed lip) {CASSETTE_LIFT:.0f} mm: clear of the inlet walls after 480 mm",
                  initialPose=dict(hood_aside, **pipe_aside),
                  moves={n: [tr3([0, 0, CASSETTE_LIFT])] for n in cassette}, maxDepth=12))
cases.append(dict(id="service-lip-restow", stage="service", minGapMm=GAP,
                  basis="Between matches, with the pocket emptied (hood and pipe set down beside the robot): fold the deployed lip 120 deg up by hand and set the latch pin (the deploy-once reset)",
                  initialPose=dict(hood_aside, **pipe_aside), moves={lip: [rot(HINGE, pg.LIP_STOW_DEG)]}, maxDepth=11))
cases.append(dict(id="service-lip-over-travel", stage="service", minGapMm=GAP,
                  basis=f"Sprung-stop over-travel: a frontal hit turns the deployed lip a further {struct['stop']['overTravelDeg']} deg down against its stop springs "
                        "until the bumper carries the load (structure-checks.json L5); the empty lip must clear every neighbour through that swing",
                  initialPose=dict(hood_aside, **pipe_aside), moves={lip: [rot(HINGE, struct["stop"]["overTravelDeg"])]}, maxDepth=11))
# The hand gauges are drawn at the end of their reach (build_layout.py), so they rest on what they operate in every other case.
# Their own cases start them raised by the reach and bring them down. The plug hand keeps hold of the battery-side plug and
# leaves with the battery.
PLUG_HAND, BREAKER_HAND = "Plug grip hand", "Breaker reset hand"
PLUG_REACH, BREAKER_REACH = 455.0 - 199.1, 600.0 - 257.2
cases.append(dict(id="service-battery-lift", stage="service", minGapMm=GAP,
                  basis="After unplugging, lift the dressed battery 250 mm through the centre gap between the equipment sheets; the hand that unplugged it "
                        "keeps hold of the battery-side plug and rises with it",
                  moves={"Dressed battery": [tr3([0, 0, 250.0])], PLUG_HAND: [tr3([0, 0, 250.0])]}))
cases.append(dict(id="service-battery-rear-carry", stage="service", minGapMm=GAP,
                  basis="Continue from the 250 mm lift rearward under the inlet chute and over the rear bumper until clear, the hand still on the plug",
                  initialPose={"Dressed battery": [tr3([0, 0, 250.0])], PLUG_HAND: [tr3([0, 0, 250.0])]},
                  moves={"Dressed battery": [tr3([-420.0, 0, 0])], PLUG_HAND: [tr3([-420.0, 0, 0])]}))
PIT_CASES = ("service-battery-rear-carry",)       # the battery leaves over the rear bumper, where the station stands when docked
cases.append(dict(id="service-plug-disconnect", stage="service", minGapMm=GAP, basis="A hand reaches down to grip the SB50 pair from above",
                  initialPose={PLUG_HAND: [tr3([0, 0, PLUG_REACH])]}, moves={PLUG_HAND: [tr3([0, 0, -PLUG_REACH])]}))
cases.append(dict(id="service-breaker-reset", stage="service", minGapMm=GAP, basis="A hand reaches the CB185 reset button outboard of the pocket walls",
                  initialPose={BREAKER_HAND: [tr3([0, 0, BREAKER_REACH])]}, moves={BREAKER_HAND: [tr3([0, 0, -BREAKER_REACH])]}))
# The checker needs one fixture roster in every case of a package, so every field fixture is registered everywhere. Each is
# physically present only at its own dock: the station behind the robot at the receive, the reef and first pipe ahead of
# it at the delivery. They are 950 mm apart in the robot frame and never interact.
for c in cases:
    c["fixtures"] = [FIX[f] for f in FIELD_SET]
    if c["id"] in PIT_CASES:
        # In the pit there is no station behind the robot. The roster must still list it, so it starts 1500 mm away.
        c["initialPose"] = dict(c.get("initialPose", {}), **{n: [tr3([-1500.0, 0, 0])] for n in STATION_SET})
        c["basis"] += ". In the pit no station stands behind the robot: the station fixtures, which every case must list, start 1500 mm rearward"
by_case = {c["id"]: c for c in cases}
# The official parts touch each other where FIRST joins them (welds, slots, bolted faces). Those joints are the field's own
# and are declared once, as intended interfaces of every case, so the check spends nothing on the field against itself.
JOINTS = [j["pair"] for j in OFFICIAL["officialJoints"]]
joint_ev = write(f"{REL}/evidence/interfaces/official-field-joints.json", dict(
    schema="red2/intended-contact@1", issue=ISSUE, status="pass", source=official_pin,
    basis="Joints of the official field model: each pair is two official parts that FIRST joins or seats on each other, measured closer than "
          "the 2.0 mm check reserve on the placed official solids (officialJoints in official-fixtures.json). They are field construction, not robot clearances.",
    interfaces=[dict(case=c["id"], pair=pr, status="pass") for c in cases if not c.get("extract") for pr in JOINTS]))
JOINT_INTERFACES = [dict(pair=pr, basis="Joint of the official field model: two official parts FIRST joins", qualifiedBy=joint_ev) for pr in JOINTS]

# ------------------------------------------------------------------ evidence bound to each trial-driven case
transport_evidence, interface_evidence = {}, {}
for cid, meta in case_meta.items():
    c = by_case[cid]
    spec = meta["spec"]
    rec = dict(case=cid, payload="pipe", moves={n: c["moves"][n] for n in by_group["pipe"]},
               initialPose={n: c.get("initialPose", {}).get(n, []) for n in by_group["pipe"]})
    if spec["kind"] == "transport":
        rec = dict(case=cid, payload="pipe", mechanisms=sorted(spec["mechanisms"]), moves=rec["moves"], initialPose=rec["initialPose"])
    transport_evidence[cid] = write(f"{REL}/evidence/transport/{cid}.json", {
        "schema": "red2/trial-path@2", "issue": ISSUE, ("transport" if spec["kind"] == "transport" else "freePath"): rec, "fieldTrials": trials_pin,
        "playback": meta["playback"], "traceWindowS": meta["windowS"], "maxDeviationFromTraceMm": meta["maxDeviationMm"], "endSnapMm": meta["snapMm"],
        "chordToleranceMm": meta["tolMm"], "qualification": QUAL[meta["playback"]],
        "basis": spec["basis"] + ". The stroke joins two poses of the solved free-piece trace and stays within the recorded deviation of it; the trace, "
                 "its control and its half-timestep repeat are in field-trials.json."})
    present = {f["id"] for f in c.get("fixtures", [])} | {b["name"] for b in blocks["blocks"] if b["group"] not in FIXTURE_GROUPS}
    ints = []
    for other in spec["contacts"]:
        if other not in present:
            continue
        ev = write(f"{REL}/evidence/interfaces/{cid}--{other.split(' (')[0].lower().replace(' ', '-')}.json", dict(
            schema="red2/intended-contact@1", issue=ISSUE, case=cid, pair=[pipe, other], status="pass",
            fieldTrials=trials_pin, playback=meta["playback"], traceWindowS=meta["windowS"],
            basis=f"The trial solves this contact: the pipe bears on {other} during this window as a compliant penalty contact, so the pair is the "
                  "trial's qualified interface, not a clearance pair. Other pairs in the case keep the 2.0 mm reserve."))
        ints.append(dict(pair=[pipe, other], basis=f"Trial-solved contact of the pipe with {other} ({meta['playback']}, {meta['windowS'][0]:.3f}-{meta['windowS'][1]:.3f} s)",
                         qualifiedBy=ev))
    c["intendedInterfaces"] = ints + JOINT_INTERFACES
# the pipe waits on the station chute while the lip deploys
ev = write(f"{REL}/evidence/interfaces/lip-deploy--station-chute-plate.json", dict(
    schema="red2/intended-contact@1", issue=ISSUE, case="lip-deploy", pair=[pipe, station], status="pass", fieldTrials=trials_pin,
    playback="receive-mission", traceWindowS=[0.0, 0.0], basis="The pipe rests on the station chute plate at the receive trial's start pose"))
by_case["lip-deploy"]["intendedInterfaces"] = [dict(pair=[pipe, station], basis="The pipe rests on the station chute plate before it is released", qualifiedBy=ev)] + JOINT_INTERFACES
ev = write(f"{REL}/evidence/interfaces/withdraw-park--reef-trough.json", dict(
    schema="red2/intended-contact@1", issue=ISSUE, status="pass", fieldTrials=trials_pin,
    playback="eject-nominal", traceWindowS=[ej_all[i_wd][0], ej_all[-1][0]],
    basis="The scored pipe rests in the trough at the eject trial's rest pose and recedes with it; the trial assesses it 2 s after the withdrawal",
    interfaces=[dict(case="withdraw-park", pair=[pipe, n], status="pass") for n in TROUGH_PARTS]))
by_case["withdraw-park"]["intendedInterfaces"] = [dict(pair=[pipe, n], basis="The scored pipe rests in the trough", qualifiedBy=ev) for n in TROUGH_PARTS] + JOINT_INTERFACES
for c in cases:
    if c.get("extract"):
        c.pop("intendedInterfaces", None)
    else:
        c.setdefault("intendedInterfaces", JOINT_INTERFACES)
motion = write(f"{REL}/paths.json", dict(schema="agent-frc/mechanical-motion@1", step=step, layoutDatum=datum, cases=cases))

operating_basis = ("2.0 mm reserve between distinct reserves: fabrication and working-load deflection of polycarbonate and "
                   "1/4 in plate; every mated face is partitioned at 2.1 mm. The roller treads are reserved by their rigid core. Each contact the "
                   "trial solves in a case (pipe on the tray, flap, lip, station chute or trough) is declared as that case's intended interface and "
                   "bound to evidence/interfaces; every other pair keeps the reserve along the trial path")
service_basis = "2.0 mm clearance for hand, tool and withdrawal paths around conservative envelopes; mated faces partitioned at 2.1 mm"
park_basis = "2.0 mm packaging reserve between parked reserves, the same partition as operation"
stages = {}
for c in cases:
    moving = sorted({g for g, m in by_group.items() if set(m) & set(c["moves"])})
    basis = {"operating": operating_basis, "park": park_basis, "service": service_basis}[c["stage"]]
    stages[c["id"]] = dict(purpose=c["stage"], movingGroups=moving, carriedWith={}, allowanceBasis=basis)
    if c["id"] in case_meta:
        spec = case_meta[c["id"]]["spec"]
        stages[c["id"]]["pathToleranceMm"] = case_meta[c["id"]]["tolMm"]
        if spec["kind"] == "transport":
            stages[c["id"]]["transportedBy"] = {"pipe": dict(mechanisms=spec["mechanisms"], drive=spec["drive"], evidence=transport_evidence[c["id"]], basis=spec["basis"])}
        else:
            stages[c["id"]]["freePath"] = {"pipe": dict(evidence=transport_evidence[c["id"]], basis=spec["basis"])}

# ------------------------------------------------------------------ mounts
def nearest_host(prefix, block):
    bb_ = by_name[block]["boundsMm"]; c = [(bb_[i] + bb_[i + 3]) / 2 for i in range(3)]
    def gap(n):
        b = by_name[n]["boundsMm"]
        return sum(max(b[i] - c[i], 0, c[i] - b[i + 3]) ** 2 for i in range(3))
    return min((n for n in by_group["structure"] if n.startswith(prefix)), key=gap)


def clamp_point(block, host):
    bb, hb = by_name[block]["boundsMm"], by_name[host]["boundsMm"]
    c = [(bb[i] + bb[i + 3]) / 2 for i in range(3)]
    return [round(min(max(c[i], hb[i]), hb[i + 3]), 3) for i in range(3)]


def host_at(prefix, point):
    for n in by_group["structure"]:
        if n.startswith(prefix):
            b = by_name[n]["boundsMm"]
            if all(b[i] - 1e-6 <= point[i] <= b[i + 3] + 1e-6 for i in range(3)):
                return n
    raise ValueError("no host at " + prefix + str(point))


lz, uz = pg.LOWER_ROLLER[1], pg.UPPER_ROLLER[1]
mounts = [
    dict(group="pocket tray", on="Floor post port", pointMm=[122.5, 147.5, round(by_name["Floor post port"]["boundsMm"][5] - 0.5, 3)],
         basis="Floor clip on the port floor post top (post to stringer below); rear end on the inlet uprights"),
    dict(group="tray supports", on=host_at("Chassis pan trusses and stringers", [205.0, 241.5, 489.525]), pointMm=[205.0, 241.5, 489.525],
         basis="Port pocket landing shelf bolted to the upper pan stringer top (Z 489.5, material outboard of Y 232); the floor post stands on the shelf; same on starboard"),
    dict(group="hood", on="Pocket tray", pointMm=[200.0, 162.5, 700.0], basis="Hood tab on the port side-wall top edge (thumb screw)"),
    dict(group="check flap", on="Hood (removable)", pointMm=[pg.FLAP_HINGE[0], 0.0, 700.0],
         basis="3 mm flap pin in clips under the hood, 4 mm under its underside"),
    dict(group="nip side plates", on=host_at("Chassis pan trusses and stringers", [250.0, 241.5, 489.525]), pointMm=[250.0, 241.5, 489.525],
         basis="Port side plate foot: two 10-32 bolts 45 mm apart into the pocket landing shelf, which bolts to the upper pan stringer top "
               "(structure-checks.json P1, P4 and P5), same on starboard"),
    dict(group="lower nip roller", on="Port nip side plate", pointMm=[pg.LOWER_ROLLER[0], 170.4, lz], basis="Flanged hex bearing in the port side plate"),
    dict(group="upper nip roller", on="Port nip side plate", pointMm=[pg.UPPER_ROLLER[0], 170.4, uz], basis="Flanged hex bearing in the port side plate"),
    dict(group="nip drive", on="Port nip side plate", pointMm=[335.0, 173.6, lz], basis="MAXPlanetary face bolted to the port side plate, its hex socket on the lower shaft"),
    dict(group="pipe sensor", on="Port nip side plate", pointMm=[blocks["beamMm"][0], 170.4, blocks["beamMm"][1]], member="Seated-pipe beam emitter (port)",
         basis="Break-beam emitter clipped in its window in the port side plate"),
    dict(group="pipe sensor", on="Starboard nip side plate", pointMm=[blocks["beamMm"][0], -170.4, blocks["beamMm"][1]], member="Seated-pipe beam receiver (starboard)",
         basis="Break-beam receiver clipped in its window in the starboard side plate"),
    dict(group="flap sensor", on="Hood (removable)", pointMm=[pg.FLAP_HINGE[0], 134.0, round(by_name["Flap-closed switch"]["boundsMm"][2] - 2.5, 3)],
         basis="Magnetic switch on the hood top over the flap hinge"),
    dict(group="lip sensor", on="Starboard nip side plate", pointMm=[353.0, -173.6, 712.0], basis="Magnetic switch on the starboard side plate outside face"),
    dict(group="reef sensor", on="Starboard nip side plate", pointMm=[347.0, -167.3, 444.0], basis="CANrange bracket on the starboard side plate inside face"),
    dict(group="nip coupling", on="Starboard nip side plate", pointMm=[335.0, -170.4, 600.0], basis="Idler post on the starboard side plate"),
    dict(group="lip latch", on="Starboard nip side plate", pointMm=[348.0, -170.4, 455.0], basis="Servo bracket on the starboard side plate extension"),
    dict(group="exit lip", on="Port nip side plate", pointMm=[pg.LIP_HINGE[0], 170.4, pg.LIP_HINGE[1]], basis="Lip hinge pin in brackets on both side plates"),
    dict(group="battery", on="Battery tray and retention", pointMm=[-150.0, 0.0, 29.9], basis="Battery case bottom on the tray"),
    dict(group="battery retention", on=host_at("Chassis sills, decks and crossmembers", [-210.0, 110.0, 111.13]), pointMm=[-210.0, 110.0, 111.13], basis="Port rear hanger under inboard sill 1"),
    dict(group="bumpers", on=host_at("Chassis perimeter", [370.0, 0.0, 150.0]), pointMm=[370.0, 0.0, 150.0], basis="Front bumper backer on the front perimeter face (mount undesigned)"),
]
for k, v in poses["steer"].items():
    mounts.append(dict(group="steer sweeps", on=f"Module {k} housing", pointMm=[v["axis"][0], v["axis"][1], 100.0], basis="Steer axis through the A035XPO turret bearing"))
for n in by_group["module housings"]:
    if "part" not in n:
        h = nearest_host("Chassis cassettes and socket rails", n)
        mounts.append(dict(group="module housings", on=h, pointMm=clamp_point(n, h), basis="Six 10-32 frame screws clamping the socket rail (nearest host point)"))
for g in ("electronics", "wiring"):
    for n in by_group[g]:
        h = nearest_host("Chassis sills, decks and crossmembers", n)
        mounts.append(dict(group=g, on=h, pointMm=clamp_point(n, h), basis="Proposed attachment: host-envelope point nearest the reserve centre (bracket undesigned)"))

# ------------------------------------------------------------------ budgets (same basis as the pan baseline)
mass = read("iterations/artifacts/resume-20261121/mass-01/mass-study.json")
line = {r["id"]: r["kg"] for r in mass["ledger"]["lines"]}
growth = {r["item"]: r["kg"] for r in mass["ledger"]["growth"]}
gw = lambda *ids: sum(growth[i] for i in ids)
POCKET_GROWTH = 0.20       # evaluation basis for every B-family line
fab_items = [("tray floor including the station chute, 1/8 in polycarbonate", 0.7606), ("two side walls, 1/8 in polycarbonate, 150 mm", 0.6142),
             ("station inlet guides and chute extension, polycarbonate", 0.8), ("hood, 1/16 in polycarbonate", 0.1546),
             ("rear check flap, pin and torsion spring", 0.08), ("two nip side plates, 1/4 in 6061, 50 % pocketed", 0.5143),
             ("deploy-once lip: polycarbonate lip, hinge, deploy spring and stops", 0.30),
             ("two rear uprights and two floor posts, 1 x 1 x 1/16 in with gussets", 0.5899),
             ("two pocket landing shelves, 200 x 114.5 x 6.35 mm 6061", 0.3926), ("rivets, screws and nuts", 0.35)]
buy_items = [("two roller shafts, 1/2 in steel hex, 360 mm (1.10 kg/m)", 0.792), ("2 in compliant wheels, up to twelve at 18 g (REV drawing mass)", 0.216), ("four flanged hex bearings", 0.08),
             ("coupling belt, two pulleys and idler", 0.11), ("NEO 550 (0.142 kg, REV-21-1651) and MAXPlanetary 4:1 x 3:1 x 3:1 (0.261 + 0.133 + 2 x 0.155 kg, REV load-rating page)", 0.846),
             ("seated-pipe break beam pair and leads", 0.02), ("flap-closed and lip-deployed magnetic switches, magnets and leads", 0.04),
             ("CANrange reef range sensor (0.6 oz) and bracket", 0.04), ("lip latch servo (58 g) and pin", 0.07),
             ("sprung lip stop: stop bar and two extension springs", 0.06)]
fab_kg = sum(k for _, k in fab_items)
buy_kg = sum(k for _, k in buy_items)
pocket_fab = by_group["pocket tray"] + by_group["hood"] + by_group["check flap"] + by_group["nip side plates"] + by_group["exit lip"] + by_group["tray supports"]
pocket_buy = (by_group["lower nip roller"] + by_group["upper nip roller"] + by_group["nip drive"] + by_group["nip coupling"] + by_group["lip latch"]
              + by_group["pipe sensor"] + by_group["lip sensor"] + by_group["flap sensor"] + by_group["reef sensor"])
ledger = [
    ("sds-modules", by_group["steer sweeps"] + by_group["module housings"], line["modules"], gw("modules"),
     "Catalog 6.0 lb SDS MK4i L2 NEO x 4, wiring included; 3 % item growth (mass-01)"),
    ("chassis-fabricated", by_group["structure"], line["chassis-fabricated"], gw("chassis-fabricated"),
     "146 taps-02 solids by measured CAD volume at 2700 kg/m3, including the 0.908 kg pan trusses kept as pocket landings; 2 % growth (mass-01)"),
    ("chassis-purchased-hardware", [], line["chassis-screws"] + line["taps02-clamp-screws"] + line["axle-spacer"],
     gw("chassis-screws", "taps02-clamp-screws", "axle-spacer"), "116 McMaster chassis screws, 4 seated clamp screws and the steel spacer; 5-20 % growth"),
    ("pocket-package-fabricated", pocket_fab, fab_kg, fab_kg * POCKET_GROWTH,
     "Estimate: " + "; ".join(f"{n} {k:.4f} kg" for n, k in fab_items) + "; +20 % growth (evaluation B'x lines; the 0.159 kg fence is not used)"),
    ("pocket-package-purchased", pocket_buy, buy_kg, buy_kg * POCKET_GROWTH,
     "Estimate: " + "; ".join(f"{n} {k:.4f} kg" for n, k in buy_items) + "; +20 % growth"),
    ("joining-allowance", [], line["joining-allowance"], 0.0, "Named allowance: unresolved joints, splices and backing (mass-01, single layer)"),
    ("electrical-allowance", by_group["electronics"] + by_group["wiring"], line["electrical-allowance"], 0.0,
     "Named allowance: controllers, PDH, roboRIO, radio, breaker, robot-side plug, harness and mounts (mass-01)"),
    ("battery-tray-retention", by_group["battery retention"], 0.35, 0.0, "Named allowance carved from the mass-01 hardware-and-finish allowance (as the baseline)"),
    ("permanent-bumper-mounts", [], 0.30, 0.0, "Named allowance carved from the mass-01 hardware-and-finish allowance; R103 counts permanent mounts"),
    ("hardware-finish-remainder", [], line["hardware-finish-allowance"] - 0.65, 0.0, "Named allowance: the remaining unspecified installed hardware and finish"),
    ("bumpers-excluded", by_group["bumpers"], 0.0, 0.0, "R103 excludes bumpers; the 7.5 kg one-colour set is carried in the running mass"),
    ("battery-excluded", by_group["battery"], 0.0, 0.0, "R103 excludes the battery and battery-side leads; 6.17 kg carried in the running mass"),
    ("field-fixtures-excluded", [n for g_ in FIXTURE_GROUPS for n in by_group[g_]], 0.0, 0.0, "Field elements placed for the receive and delivery interactions; not robot mass"),
]
masses = [dict(id=k, members=m, kg=[lo, lo + up], basis=b) for k, m, lo, up, b in ledger]
total = [0.0, 0.0]
for r in masses:                      # same accumulation order as the plugin budget review
    total = [a + b for a, b in zip(total, r["kg"])]
payload = [0.816466 + 6.17 + 7.5] * 2
running = [x + y for x, y in zip(total, payload)]

SOURCES = ["drive", "steer", "nip", "latch", "controls"]
station_l = dict(drive=[0, 10], steer=[0, 4], nip=[0, 30], latch=[0, 0.2], controls=[3, 6.6])
moving = dict(drive=[0, 52], steer=[0, 20], nip=[0, 5], latch=[0, 0.2], controls=[3, 6.6])
eject = dict(drive=[0, 10], steer=[0, 4], nip=[0, 30], latch=[0, 0.2], controls=[3, 6.6])
start = dict(drive=[0, 10], steer=[0, 4], nip=[0, 0], latch=[0, 1.5], controls=[3, 6.6])
unpowered = dict(drive=[0, 0], steer=[0, 0], nip=[0, 0], latch=[0, 0], controls=[0, 6.6])


def loads_for(cid):
    if cid == "lip-deploy":
        return start
    if cid.startswith("carry-"):
        return unpowered
    if cid == "withdraw-park" or "-drive-away" in cid or "-brake" in cid:
        return moving
    return eject if cid.startswith("eject-") else station_l


stage_loads = {cid: loads_for(cid) for cid in cycle}
packs = {"nominal": ([11.8, 12.4], [0.025, 0.027]), "weak": ([11.8, 11.8], [0.032, 0.032])}
duty = {"start": "Release the lip latch at match start (servo stall 1.5 A, robot stationary)",
        "station": "Docked at the station; the nip position hold may reach its 30 A limit for the receive impact",
        "moving": "Drive the loaded robot at up to 1.70 m/s cruise with module re-azimuth; the nip position hold draws up to 5 A",
        "eject": "Run the nip under its 30 A limit while docked at the reef",
        "unpowered": "Carry a seated pipe with every motor output off: main power off (0 A) or disabled with only the controls drawing"}
kind = {id(start): "start", id(station_l): "station", id(moving): "moving", id(eject): "eject", id(unpowered): "unpowered"}
commissioned, electrical = {}, {}
for st_ in cycle:
    for cond, (voc, res) in packs.items():
        key = f"{st_}-{cond}"
        k = kind[id(stage_loads[st_])]
        commissioned[key] = dict(stage=st_, batteryCondition=cond, sources=SOURCES, duty=duty[k],
                                 requiredMotionMinV=9.0 if k == "eject" else 6.0)
        electrical[key] = dict(loadedKg=running, batterySideA=stage_loads[st_], openCircuitV=voc, resistanceOhm=res,
                               basis=("Battery-side source ranges as the pan baseline: drive 0-52 A moving (adverse replay bus total less mechanism "
                                      "and controls) or 0-10 A docked; steer 0-20 A moving, 0-4 A docked; controls 3-6.6 A including the CANrange. Nip 0-30 A at its "
                                      "SPARK MAX current limit while ejecting and while holding against the receive impact (trials: the hold "
                                      "reaches the limit for the impact), 0-5 A holding in the carry (the 8 g sled back-drive is "
                                      f"{S['g3BackDriveNm']:.2f} N m against {SC['D2']['value']:.2f} N m at 30 A). Latch servo 1.5 A stall at the "
                                      "release, 0.2 A idle. Required: eject 9.0 V, the lowest bus the torque-limited eject trials capture at "
                                      "(field-trials.json G2, 9-13 V); other duties 6.0 V; the 9 V project floor governs. Pack: 12.4 V / 25 mOhm and 11.8 V / "
                                      "27 mOhm nominal, 11.8 V / 32 mOhm weak (experiments/six-wheel/conditioned-match/supply.py)."))
limits = write(f"{REL}/package-limits.json", dict(
    basis="R103 115 lb (2025 kickoff V0 pp. 76/86/112 per the mass-01 ruleBasis) and the project 9 V branch floor, the same "
          "limits as the pan baseline", inspectionLimitKg=mass["rule"]["limitKg"], operatingMinV=9.0,
    massLines=[r["id"] for r in masses], electricalCases=commissioned))
budgets = write(f"{REL}/budgets.json", dict(
    issue=ISSUE, step=step, motion=motion, design=design, requirements=limits, runningPayloadKg=payload,
    runningPayloadBasis="Carried CORAL 0.816466 kg plus the R103-excluded battery 6.17 kg and one-colour bumpers 7.5 kg",
    masses=masses, electricalCases=electrical))
weak_min = min(packs["weak"][0][0] - sum(v[1] for v in stage_loads[s].values()) * packs["weak"][1][1] for s in cycle)

# ------------------------------------------------------------------ installed scene: electronics clearance, start projection, retention sweeps
solids_, _ = load_solids(ROOT / step["file"])
bb = {k: aabb(v) for k, v in solids_.items()}
watch = set(by_group["electronics"] + by_group["wiring"] + by_group["battery"] + by_group["battery retention"])
elec_min = (math.inf, None)
for x, y in itertools.combinations(sorted(solids_), 2):
    if (x in watch or y in watch) and box_gap(bb[x], bb[y]) < 30:
        d = distance(solids_[x], solids_[y])
        if d < elec_min[0]:
            elec_min = (d, [x, y])
pipe_s = solids_[pipe]
sweeps = {}
dirs = {"rearward up the floor toward the inlet": (-ur[0], 0.0, -ur[1]), "up (+Z) toward the hood": (0.0, 0.0, 1.0),
        "forward (+X) into the nip": (1.0, 0.0, 0.0), "down the floor normal": (-nr[0], 0.0, -nr[1])}
for label, dv in dirs.items():
    first = None
    for travel in [float(x) for x in range(0, 401)]:
        shape = moved(pipe_s, tuple(travel * c for c in dv))
        b2 = aabb(shape)
        hits = [n for n in solids_ if n != pipe and box_gap(b2, bb[n]) < 0.6 and distance(shape, solids_[n]) <= 1e-6]
        if hits:
            first = dict(travelMm=travel, obstruction=sorted(hits))
            break
    sweeps[label] = first or dict(travelMm=None, obstruction=[], note=("no rigid obstruction within 400 mm: the rigid roller cores clear "
                                                                       "the pipe through the throat; the forward stop is the compliant tread "
                                                                       "squeeze under the powered hold, qualified by the receive and sled trials" if "nip" in label else "free beyond 400 mm"))

rules = blocks["rules"]
RULES = pin("experiments/six-wheel/kickoff/rule-register-v0.json")
ruling = write(f"{REL}/rules-rulings.json", dict(
    schema="red2/rules-rulings@1", issue=ISSUE, register=RULES,
    edition="Register 'Kickoff V0' (2025-01-04 release-day packet, experiments/six-wheel/kickoff/rule-register-v0.json)",
    rulings=[
        dict(rule="R102 / R105 (register id 'extension': 'The starting projection and the 18-inch extension limit refer to the perimeter.')",
             verdict="complies",
             facts=dict(deployedProjectionMm=rules["deployedProjectionBeyondFrameMm"], limitMm=457.2,
                        marginMm=round(457.2 - rules["deployedProjectionBeyondFrameMm"], 3),
                        startProjectionMm=rules["startProjectionBeyondFrontMm"], stowedLipMaxXMm=rules["stowedLipBoundsMm"][3],
                        stowedLipInsideFrameMm=rules["stowedLipInsideFrameMm"], deploySwingMaxProjectionMm=rules["deploySwingMaxProjectionMm"],
                        startHeightMm=rules["startMaxZMm"], startHeightLimitMm=1066.8),
             basis="The deployed lip is the only member beyond the frame perimeter: its tip is "
                   f"{rules['deployedProjectionBeyondFrameMm']:.1f} mm out (the swing passes {rules['deploySwingMaxProjectionMm']:.1f} mm momentarily while deploying), "
                   "inside the register's 18 in (457.2 mm) limit, in one direction. "
                   f"In the starting configuration the lip is folded up with its forward-most point {rules['stowedLipInsideFrameMm']:.1f} mm inside "
                   f"the frame face, so no robot reserve projects beyond the front face, and the start height is {rules['startMaxZMm']:.1f} mm "
                   "against the register's 42 in (R104, modeled).",
             open=["The register marks 'extension' unmodeled (no numeric field); the 18 in figure is its statement text. Confirm the "
                   "measurement datum against the event's manual revision and measure the as-built deployed tip at inspection."]),
        dict(rule="G412 (register id 'launch': 'A launched CORAL requires a bumper partially in its own reef zone.')", verdict="complies by interlock",
             facts=dict(reefZoneFaceToFaceMm=2374.9, reefFaceToFaceMm=1663.7, zoneTapeOutsideReefFaceMm=355.6, acceptedStandOffMaxMm=100.0,
                        bumperInsideZoneAtMaxStandOffMm=255.6, rollOffDepartureMps=S["g2DepartureMps"], rangeSensor="CTRE CANrange, 1 mm resolution, 100 Hz short-range mode",
                        rangeAtMaxStandOffMm=round(pg.BUMPER_FACE - by_name["Reef range sensor (CANrange)"]["boundsMm"][3] + 100.0, 1),
                        manualEditionRead="final 2025 manual (HTML), sections 5.2, 5.3 and G412; the V0 archive would not download on 2026-11-22"),
             basis="Final-manual text: the REEF ZONE is 'an infinitely tall 6-sided, 7 ft. 9 1/2 in. wide (face to face)' volume around a "
                   "'5 ft. 5 1/2 in.' hexagonal REEF and includes its tape, so the tape's outer edge is (93.5 - 65.5) / 2 = 14.0 in (355.6 mm) outside "
                   "the reef face. G412: 'A ROBOT may not launch CORAL unless their BUMPERS are partially in their REEF ZONE'; a CORAL is launched if it "
                   "is 'shot into the air, kicked across the floor ... or thrown in a forceful way'. The roll-off leaves the lip at "
                   f"{S['g2DepartureMps'][0]:.2f}-{S['g2DepartureMps'][1]:.2f} m/s and drops about 90 mm, so it is arguably a placement, but this "
                   "package does not rely on that reading. It complies by interlock: the nip runs forward only when the reef range sensor reads a "
                   "stand-off of 100 mm or less, the seated-pipe beam is blocked and the lip-deployed switch is closed. At 100 mm stand-off the front "
                   "bumper face is at least 255.6 mm inside the zone tape, so a robot close enough to score is in the zone with 255 mm to spare for "
                   "range error, bumper wear and the trough's offset from the reef face.",
             interlock="reef range <= 100 mm stand-off AND seated-pipe beam blocked AND lip-deployed switch closed",
             open=["The trough lip's horizontal offset from the reef base face is not in the manual text; confirm it on the field STEP. It cannot exceed "
                   "the 248 mm trough depth, and the margin is 255.6 mm",
                   "Confirm the V0 wording against the final edition when the V0 archive is reachable (the register's own statement matches)",
                   "Bench gate G5 measures the range sensor against the real trough face"]),
    ],
    consequence="The lip complies with R102/R105 and the eject complies with G412 by interlock on three placed sensors. Sensor behaviour is bench gate G5."))

# ------------------------------------------------------------------ second trip: lateral separation at sector B
SECTOR_PITCH = blocks["sectorPitchMm"]
fixtures_s, _ = load_solids(ROOT / pipe_step["file"])
first = fixtures_s[first_pipe]
robot_bodies = [n for n in solids_ if groups_of(n) not in ("pipe", "breaker service hand", "plug service hand")]
rows_b = sorted(((round(distance(first, solids_[n]), 2), n) for n in robot_bodies if box_gap(aabb(first), bb[n]) < 250.0))[:3]
SEC = [r for r in EJ if r["label"].startswith("second trip")]
end_gap = SECTOR_PITCH - pg.PIPE_LENGTH
LATERAL_LIMIT = 70.0
second_trip = write(f"{REL}/evidence/second-trip.json", dict(
    schema="red2/second-trip@1", issue=ISSUE, step=step, fixtures=pipe_step, fieldTrials=trials_pin, sectorPitchMm=SECTOR_PITCH,
    pipeLengthMm=pg.PIPE_LENGTH, pipeEndGapAtNominalDocksMm=round(end_gap, 2), lateralDockErrorLimitMm=LATERAL_LIMIT,
    worstEndGapAtLimitMm=round(end_gap - 2 * LATERAL_LIMIT, 2),
    method="Three parts. (1) Clearance: the first pipe at its trial rest pose one sector pitch toward -Y is a fixture of every eject case, so the "
           "second delivery's whole trial path is checked against it. (2) Lateral budget: the pipe ends are 169.4 mm apart at nominal docks; with "
           "each dock within +/-70 mm the ends stay 29.4 mm apart. (3) Control: the planar trial with the first pipe in the same section, "
           "which is what a dock at the wrong sector or a lateral error beyond the budget gives.",
    firstPipeToRobotNearestMm=rows_b, sameSectionControl=S["secondTrip"], sameSectionDepartureMps=S["secondTripDepartureMps"],
    finding=f"With the first pipe in the same section only {S['secondTrip']['outcomes'].get('captured and supported 2 s after withdrawal; first pipe still in the trough', 0)}"
            f"/{S['secondTrip']['n']} second ejects are captured: the rest stay on the lip against the first pipe or fall short. The second trip therefore "
            "needs the sector-B dock inside the lateral budget; with it, the second delivery is the first delivery's physics (the trial is planar).",
    limits="The sector divider panel, pipe yaw and pipe-end contact are not modelled; G4 measures the lateral dock scatter"))

# ------------------------------------------------------------------ custody with the power off
BK = {k: v for k, v in CU["staticBarrier"].items()}
STOP_LOAD_N = round(pg.PIPE_MASS * ft.G * (CU["required"]["stepG"] + math.sin(math.radians(pg.FLOOR_DEG))), 1)   # 1.5 g stop plus the floor slope
PUSH_PASS_N = math.floor(0.85 * BK[soft]["peakForceN"])          # 85 % of the model's push-through force at the softest accepted tread
PUSH_FAIL_N = math.ceil(2 * STOP_LOAD_N)                         # a suddenly applied stop load can reach twice its static value
lim = lambda table, state, k: f"{table[state][k]['largestRetainedG']:g} g kept, {table[state][k]['smallestLostG']:g} g lost" \
    if table[state][k]["smallestLostG"] is not None else f"{table[state][k]['largestRetainedG']:g} g kept (largest run)"
BRAKE_REQUIREMENT = ("Handoff requirement: the nip motor controller's idle mode is brake, so the windings are shorted whenever the robot is disabled "
                     "with the controller powered. Coast idle mode is not accepted. Brake idle mode is not credited with the breaker off, where the squeeze alone holds.")
POWER_OFF_TEXT = (
    f"With the main power off the gearbox coasts and the pipe is held by the tread squeeze: {frac(CU['powerOffRequired'])} required cases retained over "
    f"{S['bandNmm'][0]:.0f}-{S['bandNmm'][-1]:.0f} N/mm (a {CU['required']['stepG']} g stop for {CU['required']['stepS']} s and an {CU['required']['pulseG']:.0f} g, 50 ms hit, "
    f"main power off and brake idle mode), largest travel into the nip {CU['maxForwardMmRequired']} mm. At {soft} N/mm with the power off: sustained "
    f"{lim(CU['sustainedLimitsG'], OFF, soft)}; 50 ms hit {lim(CU['hitLimitsG'], OFF, soft)}. Static push-through {BK[soft]['peakForceN']} N at {soft} N/mm "
    f"against a {STOP_LOAD_N} N stop load. Control: rollers decoupled on a 20 N/mm tread lose the pipe at 8 g.")

# ------------------------------------------------------------------ premise measurements
rules = blocks["rules"]
G2 = [r for r in EJ if r["label"] == "G2 eject"]
by_s = {}
for r in G2:
    by_s.setdefault(r["standOffMm"], []).append(r["outcome"].startswith("captured"))
capture_max = max(s_ for s_, ok in by_s.items() if all(ok) and all(all(by_s[s2]) for s2 in by_s if s2 <= s_))
INTERLOCK = "reef range <= 100 mm stand-off AND seated-pipe beam blocked AND lip-deployed switch closed"
CYCLE_SUMMARY = f"{SRC}/evidence/cycle-rehearsal/summary.json"
captive = round(max(S["g1CaptiveS"][1], S["pushedCaptiveS"][1]) + trials["sensorLatencyS"], 4)
measure = write(f"{REL}/evidence/premise-measurements.json", dict(
    schema="red2/premise-measurements@2", issue=ISSUE,
    receiveFace="front",
    stationPeakMps=S["g1PeakMps"][1],
    receiveCaptiveS=captive,
    ejectCaptureStandOffMaxMm=capture_max,
    lateralDockErrorLimitMm=LATERAL_LIMIT,
    treadStiffnessNominalNmm=S["nominalNmm"],
    powerOffSustainedG=CU["sustainedLimitsG"][OFF][soft]["largestRetainedG"],
    inspectionHeavyKg=total[1],
    weakPackMinBusV=weak_min,
    electronicsMinClearanceMm=elec_min[0],
    deployedLipProjectionMm=rules["deployedProjectionBeyondFrameMm"],
    startProjectionMm=rules["startProjectionBeyondFrontMm"],
    startHeightMm=rules["startMaxZMm"],
    ejectInterlock=INTERLOCK,
    selectedArchitecture="B'x",
    chassisInterface="002-connected-machining-654-r6",
    seatedClampScrews="four 91251A542 seated in UNC-2B taps",
    electronicsPlacements="552-r5 prospective placements",
    provenance=dict(
        receiveFace="The inlet mouth is on the robot -X face, the face that docks to the station (kickoff/selected-role.md).",
        stationPeakMps=dict(source=trials_pin, field="summary.g1PeakMps[1]", note="largest pipe speed before the check flap over the 108 rest-release G1 receives (dock gap 0-50 mm)"),
        receiveCaptiveS=dict(source=trials_pin, field="max(summary.g1CaptiveS[1], summary.pushedCaptiveS[1]) + sensorLatencyS",
                             note="flap closed behind the pipe, rest-release and pushed feeds, plus the 50 ms switch latency"),
        ejectCaptureStandOffMaxMm=dict(source=trials_pin, field="largest stand-off s with every G2 eject at s and below captured and supported 2 s"),
        lateralDockErrorLimitMm=dict(source=second_trip, field="lateralDockErrorLimitMm", note="allocated control requirement, not a measurement"),
        powerOffSustainedG=dict(source=trials_pin, field=f"summary.custody.sustainedLimitsG['main power off']['{soft}'].largestRetainedG",
                                note="largest 0.5 s deceleration into the nip that every run at the softest trialled tread retains with the main power off"),
        treadStiffnessNominalNmm=dict(source=trials_pin, field="summary.nominalNmm", note="design nominal; no supplier publishes it and bench gate G0 measures it"),
        inspectionHeavyKg=dict(source=budgets, field="sum of masses[].kg upper"),
        weakPackMinBusV=dict(source=budgets, field="min over cycle stages of weak Voc - sum(upper source A) x R"),
        electronicsMinClearanceMm=dict(source=step, pair=elec_min[1], note="installed scene, every electrical reserve against every other reserve"),
        deployedLipProjectionMm=dict(source=pin(f"{REL}/layout-blocks.json"), field="rules.deployedProjectionBeyondFrameMm"),
        startProjectionMm=dict(source=pin(f"{REL}/layout-blocks.json"), field="rules.startProjectionBeyondFrontMm"),
        startHeightMm=dict(source=pin(f"{REL}/layout-blocks.json"), field="rules.startMaxZMm"),
        ejectInterlock=dict(source=ruling, field="rulings[1].interlock", note="control requirement recorded by the G412 ruling"),
        selectedArchitecture=dict(source=pin("iterations/002/design/evaluation/evaluation.json"), field="recommendation.select"),
        chassisInterface=dict(source=pin("iterations/002/design/cad/fusion-authority.json"), field="registered revision"),
        seatedClampScrews=dict(source=pin("iterations/artifacts/resume-20261121/screws-place-01/native-receipt.json"), field="status"),
        electronicsPlacements=dict(source=pin("iterations/002/design/chassis/inset-electrical-package.json"), field="candidatePlacements"))))

RULES = pin("experiments/six-wheel/kickoff/rule-register-v0.json")
all_groups = sorted(groups)
robot_groups = sorted(g for g in groups if groups[g]["role"] not in ("payload", "service") and g not in FIXTURE_GROUPS)
NIP = ["lower nip roller", "upper nip roller"]
premise_specs = [
    ("station-receive-face", "conceptual-design", "face", {"equals": "front"}, ["pocket tray", "check flap", "pipe"],
     "Both TELEOP station receives at the red station, receiving face toward the station", "receiveFace", pin("kickoff/selected-role.md"),
     "Selected station-fed L1 role: the robot docks its receiving face to the station chute"),
    ("station-feed-speed", "preliminary-design", "m/s", {"range": [0.0, S["pushedPeakMps"][1]]}, ["pocket tray", "check flap", "hood"] + NIP + ["pipe", "field station"],
     "Station feed: largest pipe speed before the check flap, released from rest on the registered chute, dock gap 0-50 mm", "stationPeakMps", trials_pin,
     f"The receive is accepted up to {S['pushedPeakMps'][1]} m/s, the fastest pushed feed (2.0 m/s down the chute) that still seats: "
     f"{frac(S['pushed']['1.0'])} and {frac(S['pushed']['2.0'])} at 1.0 and 2.0 m/s pushes"),
    ("receive-captive-time", "conceptual-design", "s", {"range": [0.0, 1.0]}, ["pocket tray", "check flap", "flap sensor"] + NIP + ["pipe"],
     "Station receive dwell: check flap closed behind the pipe plus the 50 ms flap-switch latency, after which the robot may leave", "receiveCaptiveS",
     pin(CYCLE_SUMMARY),
     "Cycle-derived (cycle_rehearsal.py, plugin field rehearsal, Red 2 from its real slot with waitCycles yield): receive dwells of 0.50-1.00 s "
     "credit 15-14 L1 cycles and leave the two-delivery role unchanged, so a dwell up to 1.0 s costs it no score. The first draft held the robot "
     "until the pipe had seated (0.54 s model, 0.15-0.69 s with contact stiffness); the receive-and-go trials show it need not wait "
     f"({frac(S['go'])} retained and seated within {S['goSeatAfterStopS'][1]} s of the stop)"),
    ("eject-capture-standoff", "conceptual-design", "mm", {"range": [100.0, 1000.0]}, ["exit lip"] + NIP + ["pipe", "field reef", "reef sensor"],
     "Sector A and B docks: bumper face to trough lip 0-100 mm (evaluation dock scatter); eject captured and supported 2 s at every stand-off",
     "ejectCaptureStandOffMaxMm", pin("iterations/002/design/evaluation/evaluation.json"),
     "The evaluation and G2 use 0-100 mm stand-off; the reef range sensor holds the eject inside it"),
    ("lateral-dock-error", "preliminary-design", "mm", {"range": [0.0, round(end_gap / 2, 2)]}, ["pipe", "field reef", "field first pipe"],
     "Second trip: lateral dock error allowed at each reef sector so the second pipe lands beside the first, not on it", "lateralDockErrorLimitMm", second_trip,
     f"The pipe ends are {end_gap:.1f} mm apart at nominal docks one sector pitch apart; they meet when both docks err {end_gap / 2:.1f} mm toward "
     "each other. The control requirement is +/-70 mm, which leaves 29.4 mm"),
    ("tread-stiffness", "preliminary-design", "N/mm", {"range": list(BAND)}, NIP + ["nip drive", "pipe"],
     "Installed tread stiffness per roller at the 6 mm squeeze (design nominal until bench gate G0 measures it)", "treadStiffnessNominalNmm", trials_pin,
     f"Receive seats {frac(S['g1'])} and eject captures {frac(S['g2'])} over 40-160 N/mm; receive fails at 10 N/mm ({frac(S['receiveStiffnessBracket']['10'])}) "
     f"and the drive stalls at 240 N/mm ({frac(S['ejectStiffnessBracket']['240'])}). The bench band stops at 120 N/mm so the steel shafts stay within a "
     "quarter of the squeeze (structure-checks.json S2)"),
    ("power-off-custody", "preliminary-design", "g", {"range": [2 * CU["required"]["stepG"], 1000.0]}, NIP + ["nip drive", "pocket tray", "pipe"],
     "Seated pipe with the main power off: largest 0.5 s deceleration into the nip retained at the softest accepted tread", "powerOffSustainedG", trials_pin,
     f"A robot that is not driving stops no harder than its tyres grip, taken as {CU['required']['stepG']} g; the limit is twice that because a suddenly "
     "applied load can reach twice its static value. " + POWER_OFF_TEXT),
    ("inspection-mass", "preliminary-design", "kg", {"range": [0.0, mass["rule"]["limitKg"]]}, robot_groups,
     "R103 heavy single-contingency estimate of the inspected configuration", "inspectionHeavyKg",
     pin("iterations/artifacts/resume-20261121/mass-01/mass-study.json"), "Heavy estimate at or under 115 lb"),
    ("weak-pack-voltage", "preliminary-design", "V", {"range": [9.0, 13.0]}, ["nip drive", "steer sweeps", "electronics", "battery"],
     "Every cycle stage on the weak 11.8 V / 32 mOhm pack at the conservative simultaneous current", "weakPackMinBusV",
     pin("iterations/002/design/strategy/drive-envelope.json"), "Lumped bus stays at or above the 9 V project branch floor"),
    ("electronics-layout", "preliminary-design", "mm", {"range": [2.0, 1000.0]}, ["electronics", "wiring", "battery", "battery retention"],
     "552-r5 electronics placements, battery gauge and wiring reserves under the pocket", "electronicsMinClearanceMm",
     pin("iterations/002/design/chassis/inset-electrical-package.json"), "Every electrical reserve keeps the 2 mm package reserve"),
    ("r105-deployed-extension", "preliminary-design", "mm", {"range": [0.0, 457.2]}, ["exit lip"],
     "R105: the deployed lip is the only extension; projection beyond the frame (robot) perimeter for the whole match", "deployedLipProjectionMm",
     RULES, "R102/R105 (register 'extension'): at most 18 in (457.2 mm) beyond the perimeter; ruling in rules-rulings.json"),
    ("r102-start-configuration", "preliminary-design", "mm", {"range": [-1000.0, 0.0]}, ["exit lip", "nip side plates", "lip latch", "nip drive", "nip coupling", "reef sensor"],
     "R102: starting configuration with the lip stowed; no robot reserve beyond the front frame face (bumpers excluded)", "startProjectionMm",
     RULES, "Stowed lip inside the vertical projection of the perimeter"),
    ("r104-start-height", "preliminary-design", "mm", {"range": [0.0, 1066.8]}, robot_groups,
     "R104: starting height of every robot reserve", "startHeightMm", RULES, "42 in start height (register 'start-height', modeled)"),
    ("g412-eject-interlock", "preliminary-design", "control", {"equals": INTERLOCK},
     ["nip drive", "exit lip", "pipe", "reef sensor", "pipe sensor", "lip sensor"], "G412: eject only with a bumper partially in our reef zone, established by the reef range sensor",
     "ejectInterlock", RULES, "G412 ruling (rules-rulings.json): a stand-off of 100 mm or less puts the bumper at least 255.6 mm inside the zone tape"),
    ("architecture-selection", "conceptual-design", "architecture", {"equals": "B'x"},
     ["pocket tray", "hood", "check flap", "nip side plates"] + NIP + ["nip drive", "exit lip"],
     "Selected scoring architecture for this package: fixed pocket, powered-hold nip, deploy-once 100 mm lip", "selectedArchitecture",
     pin("iterations/002/design/evaluation/evaluation.json"),
     "Evaluation r2 recommendation (B family, B'x if the lip passes the bench). This design closes or bench-gates each the first draft risk; bench "
     "gates G0-G6 remain physical verification, with the guarded-pan baseline A as the fallback if G1 or G3 fails"),
    ("chassis-interface", "preliminary-design", "revision", {"equals": "002-connected-machining-654-r6"},
     ["structure", "module housings", "steer sweeps", "bumpers"],
     "Carried-forward iteration 002 chassis as a revisable premise of this layout, not a fixed boundary", "chassisInterface",
     pin("iterations/002/design/cad/fusion-authority.json"),
     "Retained on its evidence; the pocket lands on the kept upper pan stringers and the rear perimeter. Replace it if the pocket's "
     "loads or a lighter structure need other landings, if the bumper outline must change, or if a fresh frame serves the pocket better"),
    ("seated-clamp-screws", "preliminary-design", "fit", {"equals": "four 91251A542 seated in UNC-2B taps"}, ["structure"],
     "Four tapped nutbar holes and seated 1/4-20 x 1 in screws as a revisable premise", "seatedClampScrews",
     pin("iterations/artifacts/resume-20261121/screws-place-01/native-receipt.json"),
     "Retained with the chassis; preload, thread phase and service travel remain open"),
    ("electronics-placements", "preliminary-design", "placement", {"equals": "552-r5 prospective placements"}, ["electronics", "wiring", "battery", "battery retention"],
     "Electronics, wiring, battery tray and battery service paths carried from the pan baseline as revisable premises", "electronicsPlacements",
     pin("iterations/002/design/chassis/inset-electrical-package.json"),
     "Retained because the pocket leaves the equipment sheets untouched; replace if the nip and sensor leads or a lighter deck need another layout"),
]
premises = []
for key, phase, unit_, crit, ifaces, scenario, field_, source, basis in premise_specs:
    acc = write(f"{REL}/premises/{key}-accepted.json", dict(id=key, scenarioId=key, scenario=scenario, unit=unit_, criterion=crit, phase=phase,
                                                             interfaces=ifaces, source=source, basis=basis))
    value = read(measure["file"])[field_]
    cur = write(f"{REL}/premises/{key}-current.json", dict(id=key, scenarioId=key, scenario=scenario, unit=unit_, value=value, issue=ISSUE,
                                                          
                                                           source=measure, valueField=field_))
    premises.append(dict(accepted=acc, current=cur))

# ------------------------------------------------------------------ presentation
CATEGORY = {"structure": ("frame", "6061-T6 aluminium tube and plate (taps-02 bodies)"),
            "bumpers": ("bumper", "4 mm backer, 69.5 mm PE foam, 0.5 mm cover (project rear-facet record)"),
            "steer sweeps": ("wheel", "SDS MK4i turret: billet wheel, nitrile tread, aluminium mounts (supplier)"),
            "module housings": ("drive", "SDS MK4i housing with two NEOs and CANcoder (supplier)"),
            "pocket tray": ("receiver", "1/8 in polycarbonate with a UHMW contact face (proposal)"),
            "hood": ("guide", "1/16 in polycarbonate (proposal)"),
            "check flap": ("mechanism", "1/16 in polycarbonate, steel pin and torsion spring (proposal)"),
            "nip side plates": ("plate", "1/4 in 6061-T6, pocketed (proposal)"),
            "lower nip roller": ("roller", "2 in compliant wheels (AndyMark am-3462 class) on a 1/2 in steel hex shaft; count and durometer set on the bench"),
            "upper nip roller": ("roller", "2 in compliant wheels (AndyMark am-3462 class) on a 1/2 in steel hex shaft; count and durometer set on the bench"),
            "nip drive": ("actuator", "REV NEO 550 (REV-21-1651) and REV MAXPlanetary 4:1 x 3:1 x 3:1"),
            "nip coupling": ("transmission", "Timing belt, pulleys and idler (article not selected)"),
            "lip latch": ("actuator", "REV Smart Robot Servo and steel pin"),
            "pipe sensor": ("sensor", "Adafruit 2168 5 mm IR break-beam pair"),
            "lip sensor": ("sensor", "REV magnetic limit switch REV-31-1462"),
            "flap sensor": ("sensor", "REV magnetic limit switch REV-31-1462"),
            "reef sensor": ("sensor", "CTRE CANrange time-of-flight sensor"),
            "field station": ("field", "FIRST official field part (field element, not robot)"),
            "field first pipe": ("game-piece", "First scored CORAL, resting in the trough (field element for the second trip)"),
            "exit lip": ("mechanism", "1/8 in polycarbonate, hinge pins, deploy torsion spring and sprung stop (proposal)"),
            "tray supports": ("structure", "1 x 1 x 1/16 in 6061-T6 tube (proposal)"),
            "wiring": ("electrical", "Copper conductor and insulation reserve"),
            "battery retention": ("plate", "Aluminium tray and hangers (planning reserve)"),
            "battery": ("battery", "Sealed lead-acid FRC battery with leads (excluded by R103)"),
            "pipe": ("game-piece", "PVC CORAL pipe (field element)"),
            "field reef": ("field", "FIRST official field part (field element, not robot)"),
            "breaker service hand": ("reserved-space", "Clearance gauge: no robot material"),
            "plug service hand": ("reserved-space", "Clearance gauge: no robot material")}
ELEC = {"Power distribution hub": ("power", "REV PDH (supplier)"), "Main 120 A breaker": ("power", "Eaton CB185 120 A breaker (supplier)"),
        "roboRIO 2": ("compute", "NI roboRIO 2 (supplier)"), "VH-109 radio": ("compute", "VH-109 radio (supplier)"),
        "Chassis battery plug and bracket": ("power", "Anderson SB50 and bracket reserve")}
components = {}
for b in blocks["blocks"]:
    if b["name"] in ELEC:
        cat, mat = ELEC[b["name"]]
    elif b["group"] == "electronics":
        cat, mat = "electrical", "REV SPARK MAX (supplier)"
    else:
        cat, mat = CATEGORY[b["group"]]
    components[b["name"]] = dict(category=cat, material=mat)

CARRIED_NOTE = " Carried forward from the iteration 002 baseline as a revisable premise."


def explain(name, group):
    k = name.split()[1] if name.startswith("Module ") else None
    base = name.split(" part ")[0]
    chassis_text = {
        "Chassis perimeter": ("Closes the frame perimeter that carries the bumpers and reacts strikes into the wheel stations.",
                              "The bumper ring and the perimeter rules bound the robot; strike loads must reach the module stations."),
        "Chassis cassettes and socket rails": ("Holds the four module cassettes; the module plates clamp the 50.8 mm socket rails.",
                                               "Four-module swerve needs a rigid, serviceable seat at each corner."),
        "Chassis sills, decks and crossmembers": ("Ties front to rear through the sills and carries the two equipment sheets and the battery hangers.",
                                                  "The two-sill architecture hangs the battery low and central and leaves the battery lift gap between the decks."),
        "Chassis pan trusses and stringers": ("Raises the upper stringers to Z 470-490; the nip side plates and floor posts land on them.",
                                              "The pocket needs landings near Z 490 on both sides of the pipe path; the existing stringers already sit there."),
    }
    if base in chassis_text:
        p, r = chassis_text[base]
        return (p + (" (separated piece after partition)" if base != name else ""), r + CARRIED_NOTE)
    if group == "steer sweeps":
        return (f"Occupies the module {k} wheel and turret through any steering angle and puts the tread on the floor at Z = 0.",
                "Four-module swerve is the project drivetrain; the full steer sweep must stay clear of the frame, bumpers, leads and battery." + CARRIED_NOTE)
    if group == "module housings":
        if "part" in name:
            return (f"Bounds a separated piece of the module {k} housing left after partition.",
                    "The fixed supplier housing must be reserved wherever it is not inside the steer sweep or the chassis reserve." + CARRIED_NOTE)
        return (f"Holds the module {k} drive and steer NEOs, gears, belt, CANcoder and plates on the socket rail.",
                "The SDS MK4i L2 corner is the selected purchased module." + CARRIED_NOTE)
    fixed = {
        "Pocket tray": ("Receives the station pipe on a 52.3 deg chute, rolls it down the 15.74 deg floor and holds it between two walls 10 mm clear of its ends.",
                        "A fixed path replaces the pan's moving receiver; the 15.74 deg floor keeps the seated pipe pressed toward the nip at 0.27 g and the walls square its 301.6 mm length."),
        "Hood (removable)": ("Caps the pocket 10 mm above the pipe, turns a high bounce from the station drop down into the pocket with its flared "
                             "lead-in, and carries the check-flap pin and the flap-closed switch.",
                             f"Isolation: without the hood {frac(S['receiveIso']['isolation: no hood'])} receives still seat, and without the flare "
                             f"{frac(S['receiveIso']['isolation: no hood flare'])} (peak rise with it {S['g1RiseMm']} mm), so the planar trials do not show either "
                             "earning its place. They are kept as the flap carrier and as a bound on a yawed or end-on bounce the trials cannot model; "
                             "the hood lifts off for the pit unload path."),
        "Check flap": ("Lets the arriving pipe in and closes behind it: the pipe is then captive and the robot may leave the station.",
                       f"Isolation: leaving the station without it loses the pipe rearward ({frac(S['goNoFlap'])} retained); with it {frac(S['go'])} "
                       f"receive-and-go runs are retained and seated, and every powered sled case is retained ({frac(S['g3'])})."),
        "Port nip side plate": ("Carries the port bearings of both rollers, the drive, the beam emitter and the lip hinge, and lands the cassette on the port stringer.",
                                f"The nip is the pocket's front stop and must hold its 12 mm squeeze: up to {S['g1RollerN']:.0f} N per roller on receive and "
                                f"{S['g2RollerN']:.0f} N in the eject squeeze at 160 N/mm (structure-checks.json P1-P3)."),
        "Starboard nip side plate": ("Carries the starboard bearings, the coupling belt and idler, the beam receiver, the reef range sensor and the lip hinge.",
                                     "Paired with the port plate it makes the nip a removable cassette."),
        "Lower nip roller": ("Lower half of the nip: front stop on receive, driven eject roller, and the roller the pipe rolls over onto the lip.",
                             "With the upper roller it replaces the evaluation's fence, which no nip could lift the pipe over (finding PD1)."),
        "Upper nip roller": ("Upper half of the nip: clamps the pipe in the bite and drives it through the throat.",
                             "A two-sided squeeze moves the pipe without spin and holds it in the bite under the powered hold."),
        "Nip drive (NEO 550 and MAXPlanetary 36:1)": (
            "Holds the nip against the arriving pipe with a powered position hold, then drives the pipe through the squeeze to eject it.",
            f"Replaces the first draft's unselected self-locking worm. Comparison: the worm stalls on treads of 110 N/mm and above "
            f"({frac(S['wormByStiffness']['110'])} at 110, {frac(S['wormByStiffness']['160'])} at 160) while this drive ejects {frac(S['g2'])} over 40-160 N/mm, "
            f"slowest {S['g2MinNipMps']:.2f} m/s in the nip. The lock is not needed: with the robot disabled the back-drivable nip still seats "
            f"{frac(S['receiveIso']['control: robot disabled (rollers back-drivable)'])} receives and holds {frac(S['g3Disabled'])} sled pulses."),
        "Nip coupling belt": ("Counter-rotates the upper roller from the lower shaft.", "Both rollers must turn so the pipe leaves the throat without spin and one drive holds both."),
        "Seated-pipe beam emitter (port)": ("Sends the break beam along the pocket just under the top of a seated pipe.",
                                            "The eject interlock needs the pipe in the bite, and the narrative requires detecting displacement or loss: the beam is "
                                            "blocked for pipe centres X 296-333 mm and clears when the pipe is 10 mm back or gone (gate G5)."),
        "Seated-pipe beam receiver (starboard)": ("Receives the break beam; a blocked beam is the seated signal.", "Paired with the emitter through windows in both side plates."),
        "Flap-closed switch": ("Reports that the check flap has closed again behind the pipe: the captive signal.",
                               f"The robot leaves on this signal instead of waiting for the pipe to settle: captive {S['g1CaptiveS'][0]}-{S['g1CaptiveS'][1]} s after the flap "
                               f"against a seat time of up to {S['g1SeatS'][1]} s."),
        "Reef range sensor (CANrange)": ("Measures the distance to the trough face under the lip; the eject runs only at 100 mm stand-off or less.",
                                         "One measurement holds the eject inside the stand-off the trials capture at and puts the bumper at least 255.6 mm inside the "
                                         "reef zone tape for G412."),
        "First scored pipe in sector A (field)": ("Field fixture: the first pipe at rest in sector A during the second delivery at sector B.",
                                                  "The second trip must clear it; in the same section it blocks the delivery (second-trip.json)."),
        "Lip-deployed switch": ("Reports that the lip reached its deployed stop.", "The eject interlock must not run the nip into a stowed lip (latch-failed control)."),
        "Lip latch servo": ("Holds the upright stowed lip for the starting configuration and releases it once at match start.",
                            "R102 needs the lip inside the perimeter at the start; the deploy happens on the field."),
        "Deployable exit lip": ("Rolls the ejected pipe off a tip 100 mm beyond the frame line at 0.55 m so it drops into the trough.",
                                f"Isolation: with the lip removed the 12 V eject is captured only bumper-to-trough ({frac(S['lipRemoved']['0'])} at 0 mm, "
                                f"{frac(S['lipRemoved']['50'])} at 50 mm); a lip pulled inside the bumper face loses {S['shortLip']['100']['n'] - S['shortLip']['100']['ok']} of "
                                f"{S['shortLip']['100']['n']} at 100 mm. Deployed to 100 mm, {frac(S['g2'])} ejects over 0-100 mm are captured and supported after "
                                "withdrawal. A latch failure (lip still up) blocks the exit. Its sprung stop lets a frontal hit fold it behind the bumper face."),
        "Floor post port": ("Supports the pocket floor from the port upper stringer.", "The floor spans 650 mm between the inlet and the nip and needs a mid support."),
        "Floor post starboard": ("Supports the pocket floor from the starboard upper stringer.", "Paired floor support."),
        "Pocket landing shelf port": ("Carries the port floor post and side-plate foot inboard from the port upper stringer.",
                                      "The stringer has material only outboard of Y 232; the post and the plate stand 60-90 mm inboard of it, so they need a shelf to land on."),
        "Pocket landing shelf starboard": ("Carries the starboard floor post and side-plate foot inboard from the starboard upper stringer.", "Paired landing shelf."),
        "Rear inlet upright port": ("Carries the inlet walls at the station face from the rear perimeter.", "The 0.8 m inlet lip is far above any chassis member."),
        "Rear inlet upright starboard": ("Carries the inlet walls at the station face from the rear perimeter.", "Paired rear support."),
        "CORAL pipe": ("The finite game piece followed from the station chute through receive, carry, eject, the lip and the trough.",
                       "The role scores two station pipes in the L1 trough; every clearance and trial uses its actual diameter and length."),
        "Bumper ring": ("Protects the frame perimeter in contact with field elements and robots.", "Required bumpers; the ring follows the actual frame plan hull." + CARRIED_NOTE),
        "Power distribution hub": ("Distributes battery power to the controllers and the control system.", "Every load in the voltage budget draws through it." + CARRIED_NOTE),
        "roboRIO 2": ("Runs the robot program, the eject interlock and the drive control.", "A control computer is required." + CARRIED_NOTE),
        "VH-109 radio": ("Links the robot to the driver station and field.", "Required communications." + CARRIED_NOTE),
        "Main 120 A breaker": ("Disconnects and protects the main battery circuit.", "Required main disconnect; reset path checked in service-breaker-reset." + CARRIED_NOTE),
        "Chassis battery plug and bracket": ("Mates the battery SB50 to the robot-side main leads.", "The battery disconnects at a plug the plug-grip hand can reach." + CARRIED_NOTE),
        "Main power loom": ("Reserves the main lead route from the SB50 to the breaker and PDH.", "The main leads must avoid the battery lift and the plug grip." + CARRIED_NOTE),
        "Branch and CAN trunk": ("Reserves the branch power and CAN route to the controllers.", "Branch and CAN wiring cross the centre gap above the crossmembers." + CARRIED_NOTE),
        "Battery tray and retention": ("Supports and retains the battery case below the sills.", "The battery sits low and central and lifts out through the centre gap." + CARRIED_NOTE),
        "Dressed battery": ("Supplies the robot; its case, leads and battery-side SB50 move together in service.", "The pack is the voltage source of the budget." + CARRIED_NOTE),
        "Breaker reset hand": ("Clearance gauge for a hand reaching the main breaker reset from above.", "Resetting the breaker must not need the pocket removed." + CARRIED_NOTE),
        "Plug grip hand": ("Clearance gauge for a hand gripping the SB50 pair.", "The battery swap starts with this disconnect." + CARRIED_NOTE),
        "Nip controller (SPARK MAX)": ("Commands the nip NEO 550 under its 30 A limit, gated by the eject interlock.",
                                       "Uses the pan controller's placement on the equipment sheet." + CARRIED_NOTE),
    }
    if name in fixed:
        return fixed[name]
    if group in ("field station", "field reef"):
        b_ = by_name[name]
        return (b_["function"][0].upper() + b_["function"][1:] + ".",
                ("The receive is checked and trialled against the official station at the shown dock. " if group == "field station" else
                 "The delivery is checked and trialled against the official REEF at the shown 50 mm dock. ")
                + "The official solid as FIRST drew it; the rest of the official assembly is left out of the check and says so.")
    if name.endswith("lead reserve"):
        return (f"Reserves the module {k} motor and CANcoder leads under the equipment sheet.", "Module leads must leave the housing clear of the steer sweep." + CARRIED_NOTE)
    if "controller" in name and name.startswith(("Port", "Starboard")):
        which = "drive" if name.split()[2] in ("1", "3") else "steer"
        return (f"Commands one module {which} NEO.", "Eight SPARK MAX controllers run the four drive and four steer motors." + CARRIED_NOTE)
    raise KeyError(name)


for b in blocks["blocks"]:
    p_, r_ = explain(b["name"], b["group"])
    components[b["name"]].update(purpose=p_, rationale=r_)

housings = [n for n in by_group["module housings"] if "part" not in n]
sweeps_g = by_group["steer sweeps"]
ctrl = [n for n in by_group["electronics"] if n.startswith(("Port controller", "Starboard controller"))]
DRIVE_BLOCK = "Nip drive (NEO 550 and MAXPlanetary 36:1)"
required = {"wheels": sweeps_g, "drive actuators": housings, "steer actuators": housings, "control computer": ["roboRIO 2"],
            "power distribution": ["Power distribution hub"], "main disconnect": ["Main 120 A breaker"],
            "battery supply": ["Dressed battery", "Chassis battery plug and bracket"], "comms": ["VH-109 radio"],
            "receiver": ["Pocket tray", "Hood (removable)", "Check flap"],
            "front stop and eject drive": ["Lower nip roller", "Upper nip roller", DRIVE_BLOCK, "Nip coupling belt"],
            "nip controller": ["Nip controller (SPARK MAX)"], "exit lip and latch": ["Deployable exit lip", "Lip latch servo"],
            "seated-pipe sensor": by_group["pipe sensor"], "flap-closed sensor": ["Flap-closed switch"], "lip-deployed sensor": ["Lip-deployed switch"],
            "reef range sensor": ["Reef range sensor (CANrange)"]}
load_members = {"drive": housings + [n for n in ctrl if n.split()[2] in ("1", "3")],
                "steer": housings + sweeps_g + [n for n in ctrl if n.split()[2] in ("2", "4")],
                "nip": [DRIVE_BLOCK, "Nip controller (SPARK MAX)", "Lower nip roller", "Upper nip roller"],
                "latch": ["Lip latch servo"],
                "controls": ["roboRIO 2", "VH-109 radio", "Power distribution hub", "Reef range sensor (CANrange)", "Flap-closed switch", "Lip-deployed switch"]
                            + by_group["pipe sensor"]}

# ------------------------------------------------------------------ records
DRIVE_NAME = "Nip drive (NEO 550 and MAXPlanetary 36:1)"
load_paths = write(f"{REL}/load-paths.json", dict(
    schema="red2/preliminary-load-paths@2", issue=ISSUE, step=step, structureChecks=struct_pin,
    paths=[
        dict(load="CORAL pipe at the seat and on receive", kind="support",
             chain=["pipe on the floor (UHMW face) and in the bite of both treads", "2 in compliant wheels on 1/2 in steel hex shafts",
                    "flanged hex bearings in both side plates", "1/4 in side plates", "two 10-32 bolts per plate, 45 mm apart, into the upper pan stringer top",
                    "upper pan stringers", "pan truss members 0-5", "truss landings, pan crossmembers, inboard sills and perimeter"],
             reserves=["CORAL pipe", "Lower nip roller", "Upper nip roller", "Port nip side plate", "Starboard nip side plate",
                       "Chassis pan trusses and stringers", "Chassis sills, decks and crossmembers"],
             loads=dict(receivePeakRollerN=S["g1RollerN"], ejectPeakRollerN=S["g2RollerN"], boltTensionN=SC["P1"]["value"], boltMargin=SC["P1"]["margin"],
                        shaftDeflectionAtBandMaxMm=next(r["value"] for r in struct["rows"] if r["id"].startswith("S2 (acceptance")),
                        note="per-roller peak tread normal force over the receive trials and the G2 ejects; sizing rows S1, S2, P1-P3"),
             absent=["Bearings and shaft collars are inside the shaft-hole partition", "Stringer local bearing under the bolts is not checked"]),
        dict(load="CORAL pipe on the floor and against the flap", kind="support",
             chain=["pipe on the floor", "pocket tray floor", "floor posts to the upper stringers and rear uprights to the rear perimeter",
                    "rearward: check flap rear stop tab -> flap pin -> hood -> hood tabs -> side walls"],
             reserves=["Pocket tray", "Floor post port", "Floor post starboard", "Rear inlet upright port", "Rear inlet upright starboard",
                       "Check flap", "Hood (removable)"],
             absent=["Rear stop tab, pin clips and hood tabs undesigned", "Hood and flare strike load unsized (bench gate G1 watches for a hood strike)"]),
        dict(load="Nip rollers", kind="drive",
             chain=["NEO 550 on the MAXPlanetary input", "three planetary stages, 36:1", "1/2 in hex socket on the lower roller shaft",
                    "lower pulley -> belt -> idler -> upper pulley", "upper roller hex shaft"],
             reserves=[DRIVE_NAME, "Lower nip roller", "Nip coupling belt", "Upper nip roller"],
             loads=dict(peakShaftDemandNm=S["g2ShaftDemandNm"], receiveBackDriveNm=S["g1BackDriveNm"], torqueAt30ANm=SC["D2"]["value"], gearboxMargin=SC["D1"]["margin"],
                        note="contact reaction fed back into the shaft speed in every trial; efficiency assumed 0.9 per stage"),
             provisional=["Belt and idler unselected", "Gearbox envelope to confirm against the supplier STEP"]),
        dict(load="Deployable exit lip", kind="support",
             chain=["lip plate", "two 3 mm hinge pins in brackets on both side plates", "sprung stop bar 30 mm from the hinge, two extension springs, 6 N m preload",
                    "stop seat on the side plates"],
             reserves=["Deployable exit lip", "Port nip side plate", "Starboard nip side plate"],
             loads=dict(rigidModelLipN=S["lipN"], rigidModelLipMomentNm=S["lipMomentNm"], landingStressMPa=SC["L3"]["value"], frontalHitStressMPa=SC["L5"]["value"],
                        frontalHitOnRigidStopMPa=SC["L5-control"]["value"], note="sizing rows L1-L9; the rigid-stop control fails, which is why the stop is sprung"),
             absent=["Stop bar, spring articles and hinge brackets are sized, not drawn"]),
        dict(load="Deployable exit lip", kind="drive",
             chain=["torsion spring at the hinge (0.10 N m stowed)", "lip", "servo pin holds the stowed lip until match start"],
             reserves=["Deployable exit lip", "Lip latch servo"], absent=["Latch pin engagement"]),
        dict(load="Each SDS module wheel", kind="support",
             chain=["wheel and tread", "wheel mounts and axle", "turret bearing", "module plates", "six 10-32 frame screws", "socket rails and cassettes",
                    "structural perimeter"], reserves=["Module steer sweeps", "Module housings", "Chassis cassettes and socket rails", "Chassis perimeter"]),
        dict(load="Dressed battery", kind="support", chain=["battery case on the tray", "end keepers", "four hangers", "inboard sill undersides"],
             reserves=["Dressed battery", "Battery tray and retention", "Chassis sills, decks and crossmembers"], absent=["Tray, hangers, keepers and hold-down undesigned"]),
        dict(load="Electronics", kind="support", chain=["devices on standoffs", "equipment sheets", "deck keys, pedestals and tabs"],
             reserves=by_group["electronics"], absent=["Device brackets and standoffs"]),
        dict(load="Bumper ring", kind="support", chain=["bumper backer", "permanent bumper mounts", "structural perimeter"],
             reserves=["Bumper ring", "Chassis perimeter"], absent=["The 002 bumper mounts are not designed (R103 counts them)"]),
    ]))
carry = write(f"{REL}/chassis-carry-forward.json", dict(
    schema="red2/selected-interface@1", issue=ISSUE, decision="revisable premise",
    ownerGuidance="The preliminary design must find what will actually work and inform the Fusion design; existing Fusion CAD does not "
                  "constrain it. Each carried item is a premise with its evidence and its replacement trigger.",
    premises=["chassis-interface", "seated-clamp-screws", "electronics-placements"],
    carried=sorted(b["name"] for b in blocks["blocks"] if b.get("carried")),
    notCarried=["Guarded pan, pan cassette, upper support, fixed inlet and the pan's controller role (A package)"],
    replaceIf=dict(
        chassis="The pocket's stringer landings carry the nip load poorly, a lighter or simpler frame serves the pocket better, or the bumper outline must change",
        electronics="The nip and sensor leads or a lighter deck need another arrangement",
        tapsAndScrews="Replacement of the chassis, or a load and maintenance review that rejects the clamp"),
    interface="Iteration 002 chassis: 146 fabricated bodies, four SDS MK4i L2 NEO modules and four seated 91251A542 clamp screws; electronics at 552-r5",
    reason="The pocket needs landings near Z 490 and a rear support; the existing upper stringers and rear perimeter provide them, and the "
           "chassis evidence (six issues, occupied-shape closeout, seated screws) is the deepest the project has. Carrying it lets this "
           "package test the scoring mechanism alone; it is not a constraint on the Fusion design.",
    evidence=[pin("iterations/002/design/cad/fusion-authority.json"), pin("iterations/002/design/cad/issue-006/lineage.json"),
              pin("iterations/artifacts/resume-20261121/screws-place-01/native-receipt.json"), pin(blocks["inputs"]["chassisStep"]["file"])]))

seat_g = math.sin(math.radians(pg.FLOOR_DEG))
iso_gs = S["sledIso"]
gs = lambda d: ", ".join(f"{k} g {frac(v)}" for k, v in d.items())
retention = write(f"{REL}/retention-review.json", dict(
    schema="red2/pocket-retention-review@2", issue=ISSUE, step=step,
    method="Static gravity projection at the seat, rigid sweep of the whole seated pipe from its carry seat toward each opening against "
           "every other reserve, and the free-piece receive, receive-and-go and sled trials (evidence/field-trials.json: stick-slip friction, so a seated "
           "pipe holds statically) for dynamic capture, power loss and the isolation of each retaining member.",
    seat=dict(poseMm=list(SEAT), creepOver3sMm=trials["seatCreepMmOver3s"]["80.0"], gravityAlongFloorTowardNipG=round(seat_g, 4),
              gravityIntoFloorG=round(math.cos(math.radians(pg.FLOOR_DEG)), 4),
              pushBack=["floor (UHMW face)", "lower and upper treads under the powered hold (front)", "hood 10 mm above (up)",
                        "check flap and its rear stop (rear)", "side walls 10.2 mm from the pipe ends (+/-Y)"]),
    sweepsFromSeat=sweeps,
    powerOff=dict(holder="The tread squeeze: to leave forward the pipe must climb the 12 mm squeeze, which needs no power. The 36:1 gearbox is back-drivable "
                         "and adds only its reflected inertia; brake idle mode adds damping while the controller is powered. Rearward the check flap's stop holds.",
                  summary=POWER_OFF_TEXT, controlRequirement=BRAKE_REQUIREMENT, required=CU["required"], requiredBasis=CU["requiredBasis"], requiredCases=CU["powerOffRequired"],
                  requiredByNipState=CU["requiredByState"], belowBand20NmmByNipState=CU["belowBandRequired"], towardFlap=CU["towardFlap"],
                  sustainedLimitsG=CU["sustainedLimitsG"], hitLimitsG=CU["hitLimitsG"], staticBarrier=CU["staticBarrier"], stopLoadN=STOP_LOAD_N,
                  atRest20s=dict(CU["atRest"], finalOffsetMm=CU["atRestFinalOffsetMm"]), treadRestitutionSensitivity=CU["restitution"],
                  sled=S["g3Disabled"], receive=S["receiveIso"]["control: robot disabled (rollers back-drivable)"], eject=S["ejectDisabled"],
                  escapeControl=dict(case="sled-isolation-no-flap and receive-and-go-no-flap (field-trials.json playback)",
                                     result=f"check flap removed: 8 g toward the inlet from a gently settled seat {frac(iso_gs['isolation: no check flap, gently settled seat']['8'])} "
                                            f"retained; receive and go {frac(S['goNoFlap'])} retained",
                                     firstOpening="rearward up the floor toward the inlet")),
    isolation=dict(
        drive=f"Powered hold: {frac(S['g1'])} receives seat over 40-160 N/mm. Robot disabled: {frac(S['receiveIso']['control: robot disabled (rollers back-drivable)'])} "
              f"receives seat and {frac(S['g3Disabled'])} sled pulses are retained. The the first draft worm lock seats {frac(S['receiveIso']['comparison: first-draft worm locked'])}: "
              "the lock adds nothing the powered hold and the gearbox do not give.",
        flap="No flap from the received seat: " + gs(iso_gs["isolation: no check flap, received seat"]) + "; from a gently settled seat: "
             + gs(iso_gs["isolation: no check flap, gently settled seat"]) + f"; receive and go without it: {frac(S['goNoFlap'])} retained",
        hood=f"No hood: {frac(S['receiveIso']['isolation: no hood'])} receives seat; no flare: {frac(S['receiveIso']['isolation: no hood flare'])}; peak rise with both {S['g1RiseMm']} mm. "
             "Not shown to earn their place in the planar model.",
        tread=f"10 N/mm: {frac(S['receiveStiffnessBracket']['10'])} receives seat; 20 N/mm {frac(S['receiveStiffnessBracket']['20'])}; 30 N/mm "
              f"{frac(S['receiveStiffnessBracket']['30'])}; 40-160 N/mm {frac(S['g1'])} with tread mu 0.3-1.0."),
    findings=[
        dict(id="RR1", status="bench gate G0", text=f"Tread stiffness decides receive custody: fails at 10 N/mm, {frac(S['receiveStiffnessBracket']['20'])} at 20 N/mm, all pass from 30 N/mm, accepted band {BAND[0]:.0f}-{BAND[1]:.0f} N/mm "
             "per roller. No supplier publishes it; G0 measures the installed stack and sets the wheel count."),
        dict(id="RR2", status="bench gate G3", text="Carry custody rests on the flap for rearward loads and the powered hold and squeeze for forward loads; "
             f"model: {frac(S['g3'])} powered and {frac(S['g3Disabled'])} disabled sled cases retained at 1-8 g."),
        dict(id="RR4", status="closed by trial; bench gate G7", text=POWER_OFF_TEXT),
        dict(id="RR3", status="closed in this layout; bench gate G5", text="Seated-pipe beam, flap-closed switch, lip-deployed switch and reef range sensor are selected and placed."),
    ]))

findings = write(f"{REL}/design-findings.json", dict(
    schema="red2/preliminary-design-findings@2", issue=ISSUE, predecessor="the first draft (issue red2-002-preliminary-layout-Bx)",
    sources=[trials_pin, struct_pin, second_trip, pin("iterations/002/design/evaluation/evaluation.json")],
    findings=[
        dict(id="PD2", item="Tread stiffness", was="40 N/mm passes, 10 N/mm fails, no measured value", status="bounded; bench gate G0",
             now=f"Acceptance band {BAND[0]:.0f}-{BAND[1]:.0f} N/mm per roller at the 6 mm squeeze, nominal 80. Receive: 10 N/mm {frac(S['receiveStiffnessBracket']['10'])}, "
                 f"20 N/mm {frac(S['receiveStiffnessBracket']['20'])}, 40-160 N/mm {frac(S['g1'])} (tread mu 0.3-1.0). Eject: 10-160 N/mm captured, 240 N/mm "
                 f"{frac(S['ejectStiffnessBracket']['240'])} (stalls). No supplier or community source publishes a 2 in compliant wheel's radial stiffness, "
                 "so the wheel count and durometer are set on the bench: 240-720 N at 6 mm on the installed roller."),
        dict(id="PD3/PD6", item="Self-locking worm, no part selected", was="must self-lock against 4.10 N m; reducer unselected", status="closed by a design change",
             now=f"The worm is removed. Drive: REV NEO 550 through a REV MAXPlanetary 4:1 x 3:1 x 3:1 (36:1, 1/2 in hex socket), with a powered position hold. "
                 f"The worm stalls on treads of 110 N/mm and above ({frac(S['wormByStiffness']['110'])}), which would have capped the tread band at about 80; the "
                 f"planetary ejects {frac(S['g2'])} at 9-13 V. Lower ratios overspeed the roll-off, so 36:1 is the ratio. Disabled, the back-drivable nip still "
                 f"seats {frac(S['receiveIso']['control: robot disabled (rollers back-drivable)'])} receives and holds {frac(S['g3Disabled'])} sled pulses. Peak shaft "
                 f"torque {S['g2ShaftDemandNm']} N m against a 180 N m class gearbox rating; the hold reaches its 30 A limit on the receive impact."),
        dict(id="PD17", item="Custody with the power off", was="the first draft relied on the worm's self-lock; with the worm removed, 18 disabled sled pulses were the only evidence",
             status="closed by trial and a control setting; bench gate G7",
             now=POWER_OFF_TEXT + " " + BRAKE_REQUIREMENT + " It holds further while the controller is powered "
                 f"({lim(CU['hitLimitsG'], 'disabled, brake idle mode', soft)} for a 50 ms hit at {soft} N/mm) but is not credited with the breaker off. "
                 f"Limit: at {soft} N/mm with the power off the pipe is pushed through by {CU['sustainedLimitsG'][OFF][soft]['smallestLostG']:g} g sustained or a "
                 f"{CU['hitLimitsG'][OFF][soft]['smallestLostG']:g} g hit; it then rolls out over the lip. No lock, detent or gate is added."),
        dict(id="PD11", item="Receive seat time", was="0.543 s slowest, 0.145-0.689 s with contact stiffness, against 1.0 s (cycle) and 0.5 s (bench target)",
             status="closed by a control change; bench gate G1",
             now=f"The dwell is the captive time: flap closed behind the pipe, {S['g1CaptiveS'][0]}-{max(S['g1CaptiveS'][1], S['pushedCaptiveS'][1])} s after it passes the flap, "
                 f"plus 50 ms for the flap switch: {captive} s against 1.0 s. Seat time is now {S['g1SeatS'][0]}-{S['g1SeatS'][1]} s (the receive starts on the station "
                 f"chute and rebounds) and no longer holds the robot: leaving 50 ms after the flap closes, at 3 and 6 m/s2, retains {frac(S['go'])} and the pipe is "
                 f"seated within {S['goSeatAfterStopS'][1]} s of the stop. Without the flap the same run loses the pipe ({frac(S['goNoFlap'])})."),
        dict(id="PD7", item="Exit-lip stop and tip load", was="stop, hinge, lip stiffness and tip load unsized; the lip is 24.5 mm ahead of the bumper all match",
             status="closed by a design change; bench gate G6",
             now=f"The lip stays 100 mm out because the reach is earned: a lip inside the bumper face captures only {frac(S['shortLip']['100'])} at 100 mm stand-off. "
                 f"A frontal hit on a rigid stop would bend the 1/8 in polycarbonate to {SC['L5-control']['value']:.0f} MPa (yield 62), so the deployed stop is sprung: "
                 f"6 N m preload, {struct['stop']['overTravelDeg']} deg over-travel, which folds the tip behind the bumper face at {SC['L5']['value']:.0f} MPa. Landing "
                 f"stress {SC['L3']['value']:.1f} MPa, static sag {SC['L2']['value']:.2f} mm, deploy {struct['lip']['deployS']} s. The over-travel swing is a checked case."),
        dict(id="PD12", item="Side-plate attachment", was="each plate bears on 20 mm of stringer; bolt pattern and bending unsized", status="closed by calculation",
             now=f"Two 10-32 bolts per plate 45 mm apart: {SC['P1']['value']:.0f} N bolt tension from the receive impact against 4500 N pull-out (margin {SC['P1']['margin']}); "
                 f"plate bending {SC['P2']['value']:.1f} MPa. Steel 1/2 in hex shafts: {next(r['value'] for r in struct['rows'] if r['id'].startswith('S2 (acceptance'))} mm "
                 "mid-span at the band maximum against a 1.5 mm allowance (aluminium would deflect 2.9 times as far)."),
        dict(id="RR3", item="Sensor brackets", was="seated-pipe sensor and lip switch proposed but undesigned", status="closed in this layout; bench gate G5",
             now="Selected and placed as blocks: Adafruit 2168 break beam through windows in both side plates (blocked for pipe centres X 296-333 mm), REV-31-1462 "
                 "magnetic switches for flap-closed and lip-deployed, and a CTRE CANrange reading the trough face."),
        dict(id="PD9", item="G412", was="ruling open; reef zone unregistered; zone sensing unspecified", status="closed by ruling and interlock; bench gate G5",
             now="Reef zone tape is 355.6 mm outside the reef face (final manual 5.2: 93.5 in zone, 65.5 in reef). The eject is interlocked on a reef range "
                 "of 100 mm stand-off or less, which puts the bumper at least 255.6 mm inside the zone. The trough lip's offset from the reef face is the one "
                 "unread dimension; it cannot exceed the margin."),
        dict(id="PD13", item="Second trip into sector B", was="clearance check only", status="bounded; bench gate G4",
             now=f"The first pipe is a fixture of every eject case. Control: in the same section the second eject is captured only "
                 f"{S['secondTrip']['outcomes'].get('captured and supported 2 s after withdrawal; first pipe still in the trough', 0)}/{S['secondTrip']['n']} times, so the "
                 f"second trip needs each dock within +/-{LATERAL_LIMIT:.0f} mm laterally (pipe ends {end_gap:.1f} mm apart at nominal docks). G4 measures the scatter."),
        dict(id="PD14", item="Station chute", was="not a registered fixture; receive trials started on the robot's inlet at an energy estimate", status="closed in this layout",
             now=f"The station chute is a registered fixture and the receive trial starts on it: rest release leaves the sill at {S['g1ExitMps'][0]}-{S['g1ExitMps'][1]} m/s "
                 f"and peaks at {S['g1PeakMps'][0]}-{S['g1PeakMps'][1]} m/s before the flap (the first draft assumed 2.22-2.47). Pushed feeds to {S['pushedPeakMps'][1]} m/s "
                 f"still seat ({frac(S['pushed']['2.0'])}). The pipe's contact with the chute is a qualified intended interface. A flared hood lead-in was added for "
                 "the bounce the drop produces."),
        dict(id="PD15", item="Prescribed mission motion", was="every mission case was a straight or circular stroke drawn from geometry, pinned to a trial",
             status="closed in this layout",
             now=f"Every receive and eject case is a stroke between two poses of a solved trace (receive-mission, eject-nominal), within {CHORD_TOL:.0f} mm of it{TOL_NOTE}, with the "
                 "flap angle from the same trace and the trace time on each frame. Only the lip deploy and the park drive are prescribed, and say so."),
        dict(id="PD16", item="Budgets", was="weak-pack bus 9.28 V against 9.0 V; 48.34 of 52.16 kg", status="within",
             now=f"Weak-pack minimum bus {weak_min:.2f} V against 9.0 V; heavy inspection estimate {total[1]:.2f} of {mass['rule']['limitKg']:.2f} kg. The steel shafts and the "
                 "planetary add mass; the hold adds 5 A to the carry and 30 A to the docked receive."),
        dict(id="PD5", item="B fallback", was="B (no lip) dock window ends between 75 and 100 mm", status="not re-run",
             now="This design does not re-run the lip-less B launch. The lip-removed isolation is its nearest evidence and B stays a fallback only in the sense of the evaluation."),
    ]))

BENCH_ARTICLE = ("Pocket bench article to this layout: 15.74 deg floor, flared hood, check flap with its magnetic switch, two-roller nip on 1/2 in steel hex shafts driven "
                 "by a NEO 550 and MAXPlanetary 36:1, the 100 mm lip on its sprung stop, the break beam and the range sensor. Station mock: 35 deg chute, 419 mm "
                 "ramp, sill 952.5 mm. Real 4.5 in pipes at 0.50-0.82 kg, new and worn; trough mock per target.json.")
gate = lambda id_, name, test, pass_, kill, model, predicted: dict(id=id_, name=name, test=test, pass_=pass_, kill=kill,
                                                                   prediction=dict(model=model, predicted=predicted), status="open")
bench = write(f"{REL}/bench-gates.json", dict(
    schema="red2/bench-gates@2", issue=ISSUE, status="open: no bench article built or tested. Physical results are not required to accept the preliminary design; "
    "each gate is the recorded test with its pass and fail numbers", source=pin("iterations/002/design/evaluation/evaluation.json"), article=BENCH_ARTICLE,
    gates=[
        gate("G0", "tread stiffness and friction", "Press a pipe 6 mm into each installed roller on a scale, three places along the roller; pull a pipe along a locked tread and along the UHMW floor",
             f"{BAND[0] * 6:.0f}-{BAND[1] * 6:.0f} N at 6 mm per roller ({BAND[0]:.0f}-{BAND[1]:.0f} N/mm); tread mu >= 0.3; floor mu 0.15-0.35. Adjust the wheel count or durometer until inside the band",
             "Below 120 N at 6 mm (20 N/mm) or above 1440 N (240 N/mm) with every available wheel option",
             f"Receive passes from 30 N/mm, seats {frac(S['receiveStiffnessBracket']['20'])} at 20 and fails at 10; eject passes to 160 and stalls at 240; shafts deflect {next(r['value'] for r in struct['rows'] if r['id'].startswith('S2 (acceptance'))} mm at 120",
             "unknown: no stiffness data exists; the band is three times wide and the wheel count tunes it"),
        gate("G1", "station receive", "30 feeds released from rest on the station mock at dock gaps 0, 10 and 50 mm, nip on powered hold; 10 pushed at about 2 m/s; 10 with the robot disabled",
             f"40/40 powered feeds captive (flap switch closed) within 0.50 s of the pipe passing the flap (model {S['g1CaptiveS'][0]}-{max(S['g1CaptiveS'][1], S['pushedCaptiveS'][1])} s); none through the nip; none back out; "
             "10/10 disabled feeds stay in the pocket", "Any pipe through the nip, or any pipe out of the inlet",
             f"{frac(S['g1'])} seated, captive {S['g1CaptiveS'][0]}-{S['g1CaptiveS'][1]} s, peak rise {S['g1RiseMm']} mm; pushed {frac(S['pushed']['2.0'])}; disabled {frac(S['receiveIso']['control: robot disabled (rollers back-drivable)'])}",
             "pass inside the G0 band"),
        gate("G2", "eject into trough", "Stand-offs 0, 25, 50, 75, 100 mm x loaded bus 9.0, 10.5, 12.0 V x 10 ejects = 150; record tip departure speed and nip current",
             f">= 147/150 captured and still supported 2 s after a 120 mm withdrawal; every cell >= 9/10; nip never stalls; departure {S['g2DepartureMps'][0]:.1f}-{S['g2DepartureMps'][1]:.1f} m/s",
             "Any cell at 7/10 or below, or a stall at 9 V", f"{frac(S['g2'])} captured at 9-13 V over 40-160 N/mm, departure {S['g2DepartureMps'][0]}-{S['g2DepartureMps'][1]} m/s, slowest {S['g2MinNipMps']} m/s in the nip; 125 mm stand-off {frac(S['standOff125'])}",
             "pass; the assumed 0.9 per stage efficiency is what this gate measures"),
        gate("G3", "custody sled and drive-away", "1, 3 and 8 g for 50 ms, forward and rearward, 10 each, nip on hold; repeat 3 g and 8 g disabled; 10 receive-and-go runs leaving 50 ms after the flap switch at 3 m/s2",
             "60/60 powered retained; 40/40 disabled retained; 10/10 receive-and-go seated (beam blocked) within 0.5 s of stopping", "Any loss at 3 g or below",
             f"powered {frac(S['g3'])}, disabled {frac(S['g3Disabled'])}; receive and go {frac(S['go'])}, seated within {S['goSeatAfterStopS'][1]} s; without the flap {frac(S['goNoFlap'])}",
             "pass with the flap"),
        gate("G4", "end to end, two trips", "20 consecutive two-trip cycles: feed, 5 m carry ending in 1.5 m/s2 braking, eject at sector A, return, feed, eject at sector B; record the lateral dock error at each eject",
             f">= 19/20 cycles with both pipes credited and no hand recovery; every lateral dock error within +/-{LATERAL_LIMIT:.0f} mm", None,
             f"not simulated end to end; same-section control {S['secondTrip']['outcomes'].get('captured and supported 2 s after withdrawal; first pipe still in the trough', 0)}/{S['secondTrip']['n']} captured",
             "unknown: depends on the drivetrain's lateral dock accuracy, which this package does not model"),
        gate("G5", "sensing and interlock", "30 seated pipes, 10 displaced 10 mm toward the flap, 10 removed; 10 lip stowed and 10 deployed; range sensor at 0, 50, 100, 150 and 400 mm stand-off from the trough mock, 10 readings each; 8 g pulses on 30 seated pipes",
             "Every displaced or missing pipe flagged within 50 ms; every lip state correct; every range reading within +/-15 mm; no eject command accepted at 150 mm or beyond; no false loss through the pulses",
             "Any eject command reaching the nip with the lip stowed, the pipe unseated or the stand-off beyond 150 mm",
             "not simulated: beam window X 296-333 mm for the pipe centre and the 255.6 mm zone margin are geometry", "unknown"),
        gate("G6", "lip stop and frontal hit", "Static: pipe on the tip, measure sag. Drop the pipe 60 mm onto the land 20 times. Push the tip rearward with 300 N, 50 times; then drive the bench cart into a wall at 1 m/s, 5 times",
             f"Sag <= 3 mm (model {SC['L2']['value']:.2f} mm); the lip folds behind the bumper face and returns within 1 mm every time; no crack; deploy within 1.0 s of release (model {struct['lip']['deployS']} s)",
             "Any permanent set over 1 mm or any crack", f"landing {SC['L3']['value']:.1f} MPa, frontal {SC['L5']['value']:.1f} MPa against 62 MPa yield; rigid-stop control {SC['L5-control']['value']:.0f} MPa", "pass"),
        gate("G7", "custody with the power off", "Nip drive unplugged from its controller. (a) Push a seated pipe through the nip with a force gauge, 5 times. (b) Stand the article "
             "nose-down (floor vertical, 1 g into the nip) with a pipe seated for 60 s, 5 times. (c) 8 g, 50 ms pulses into the nip, 20 times. Repeat (c) with the "
             "controller powered, disabled, in brake idle mode",
             f"(a) every push-through force >= {PUSH_PASS_N} N (model {BK[soft]['peakForceN']} N at {soft} N/mm, {BK['80']['peakForceN']} N at 80); (b) 5/5 stay seated; "
             "(c) 20/20 and 20/20 retained, pipe back on the seat (beam blocked) within 0.5 s",
             f"Any push-through force below {PUSH_FAIL_N} N (twice the {STOP_LOAD_N} N stop load), any pipe out nose-down, or any loss in (c)",
             POWER_OFF_TEXT, "pass inside the G0 band; the force in (a) is also the G0 stiffness cross-check"),
    ],
    measurementsFirst=["G0: installed tread stiffness and the three friction coefficients", "PVC restitution on UHMW and on the tread", "Drive current through the squeeze at 9 V"],
    decision="G0-G3 and G5-G7 pass: carry B'x into detailed design. G1, G3 or G7 fails inside the G0 band: keep the guarded-pan baseline A. G2 fails: revisit the lip "
             "or the ratio before A. G4 is a whole-robot gate and runs with the drivetrain."))

choice = write(f"{REL}/iteration-choice.json", dict(
    schema="red2/iteration-choice@1", choice="preliminary design Fixed pocket 1.0 of iteration 002; builders in iterations/002/design/preliminary-pocket (no numbered cutover)",
    reason="This design closes or bench-gates each open item of the first draft. The guarded-pan baseline A (iterations/002/design/preliminary and pan) stays on file as the "
           "fallback if bench gate G1 or G3 fails; it is not being developed.",
    cutoverTrigger="When the owner accepts Fixed pocket 1.0 as the effective preliminary design: open the detailed design for B'x and carry the chassis, modules and "
                   "electronics item by item as the premises in chassis-carry-forward.json allow."))

OBJ_RECEIVE = "TELEOP: take one station-issued CORAL from the red station"
OBJ_DELIVER = "Release into the intended L1 trough sector A, clear the wall and leave the pipe supported after withdrawal"
narrative = dict(
    source=pin("kickoff/selected-role.md"),
    read="Read end to end on 2026-11-22: LEAVE, two TELEOP station-to-L1 cycles in sectors A and B, custody, trough release and "
         "support, park, the endgame choice and the empty-start fallback.",
    objectives=[
        dict(objective="AUTO: leave the starting line with the whole occupied robot (3 points)", disposition="out-of-scope",
             basis="A field-route objective for the empty robot; this package reserves the stowed starting configuration (886.7 mm tall, lip "
                   "inside the perimeter) and deploys the lip at match start (lip-deploy) but places no starting-line fixture."),
        dict(objective=OBJ_RECEIVE, disposition="represented",
             basis=f"Cases {RECEIVE_IDS[0]} to {RECEIVE_IDS[-1]} follow the receive-mission trial from the registered station chute to the carry seat; G1 receive trials "
                   f"{frac(S['g1'])}; premises station-receive-face, station-feed-speed and receive-captive-time."),
        dict(objective="Carry it to the red reef while crossing peer routes", disposition="represented",
             basis="The drive-away and brake cases of the receive-mission trial (3 m/s2), the 6 m/s2 receive-and-go runs and the sled trials at 1-8 g both ways; "
                   "weak-pack voltage premise. A frontal hit on the lip is the sprung-stop case service-lip-over-travel. Peer routing itself is a field-rehearsal question."),
        dict(objective="Sustain a deliverable load: keep custody when accelerating, braking or contacted, and detect displacement or loss",
             disposition="represented", basis=f"Sled trials: powered {frac(S['g3'])}, disabled {frac(S['g3Disabled'])} retained; the seated-pipe beam, flap-closed switch and "
                   f"lip-deployed switch are selected and placed. Power off: case {CARRY_IDS[0]} follows the custody-power-off-hit trial; {frac(CU['powerOffRequired'])} required "
                   "stop and hit cases retained on the squeeze alone. Physical retention and detection are bench gates G3, G5 and G7."),
        dict(objective=OBJ_DELIVER, disposition="represented",
             basis=f"Cases {EJECT_IDS[0]} to {EJECT_IDS[-1]} follow the eject-nominal trial into the placed trough; G2 ejects {frac(S['g2'])} captured and supported 2 s after a "
                   "120 mm withdrawal at every 0-100 mm stand-off, with latch-failed, disabled, lip-removed, short-lip and 125 mm controls."),
        dict(objective="Return for a second station CORAL and score it in sector B with the first pipe as an obstacle", disposition="represented",
             basis="The cycle resets (empty pocket, lip stays deployed). The first pipe at its rest pose is a fixture of every eject case; second-trip.json gives the "
                   f"lateral budget (+/-{LATERAL_LIMIT:.0f} mm, premise lateral-dock-error) and the same-section control. The sector divider is not modelled; G4 measures the dock scatter."),
        dict(objective="PARK: reserve the south lane, stow, reach the zone and stop", disposition="represented",
             basis="Case withdraw-park; the lip stays deployed (it has no in-match stow), which the park envelope and R105 premise both include."),
        dict(objective="Endgame alternatives: shallow or deep CAGE", disposition="out-of-scope", basis="The selected role parks."),
        dict(objective="Fallback: empty-start leave-and-park for 5 points", disposition="represented",
             basis="The same robot with no pipe aboard; lip-deploy and withdraw-park need no pipe."),
        dict(objective="Partner and station dependence: Red 1 station change and station depletion", disposition="out-of-scope",
             basis="An alliance-chain comparison owned by conceptual design; it changes no package volume."),
    ])

# support and load path of every placed member, from the registered mounts
support = {}
for mnt in mounts:
    for n in ([mnt["member"]] if mnt.get("member") else by_group[mnt["group"]]):
        if n not in support:
            support[n] = f"On {mnt['on']} at ({mnt['pointMm'][0]:.0f}, {mnt['pointMm'][1]:.0f}, {mnt['pointMm'][2]:.0f}) mm: {mnt['basis']}"
for n in by_group["pipe"]:
    support[n] = "Moved by contact (tray, flap, held or driven rollers, lip) or free, along the receive-mission, custody-power-off-hit and eject-nominal trial traces"
for g_ in ("breaker service hand", "plug service hand"):
    for n in by_group[g_]:
        support[n] = "Unmounted service allowance (a hand, not a robot part): no mount by design; drawn at the end of its reach"
for g_ in FIXTURE_GROUPS:
    for n in by_group[g_]:
        support[n] = "Field fixture: placed in the robot datum at the shown dock, not carried by the robot"
uncovered = [dict(id="bench-" + g_["id"], condition=f"Bench gate {g_['id']} ({g_['name']}): {g_['test']}",
                  reason=f"Physical verification, not required to accept the preliminary design. Pass: {g_['pass_']}") for g_ in read(bench["file"])["gates"]]
uncovered.append(dict(id="second-trip-same-section", condition="Second delivery with the first pipe in the same trough section",
                      reason="Fails in the trials (control). Excluded by the lateral dock premise; G4 measures the scatter"))
# ------------------------------------------------------------------ recorded trials in the one motion list
# Each replay plays the solved trace itself (every recorded sample at its stride), in the robot frame the checked cases use.
REPLAYS = f"{REL}/trial-replays.json"
I12 = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]
t_tr = lambda dx, dz=0.0: [1, 0, 0, round(dx, 3), 0, 1, 0, 0, 0, 0, 1, round(dz, 3)]


def t_rot(origin, deg):
    c_, s_ = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    ox_, oz_ = origin
    return [c_, 0, s_, ox_ - (c_ * ox_ + s_ * oz_), 0, 1, 0, 0, -s_, 0, c_, oz_ - (-s_ * ox_ + c_ * oz_)]


def go_travel(rec, tm):
    """Robot travel (mm) along its commanded drive-away profile at trial time tm: accelerate, cruise, brake to rest."""
    a_, ta, tc = rec["go"]["aMps2"], rec["go"]["accelS"], rec["go"]["cruiseS"]
    u = min(max(tm - rec["goAtS"], 0.0), 2 * ta + tc)
    if u <= ta:
        d = 0.5 * a_ * u * u
    elif u <= ta + tc:
        d = 0.5 * a_ * ta * ta + a_ * ta * (u - ta)
    else:
        v = u - ta - tc
        d = 0.5 * a_ * ta * ta + a_ * ta * tc + a_ * ta * v - 0.5 * a_ * v * v
    return d * 1e3


def replay(id_, label, pid, stride, objective, basis, shown, frame, marks, recorded, display):
    rec = play(pid)
    rows = rec["trace"][::stride] + ([rec["trace"][-1]] if (len(rec["trace"]) - 1) % stride else [])
    frames = [frame(rec, q) for q in rows]
    names_ = sorted({g_ for f_ in frames for g_ in f_["groups"]})
    events = []
    for label_, when in marks(rec):
        k = next((i for i, q in enumerate(rows) if q[0] >= when), len(rows) - 1)
        if not events or k > events[-1]["frame"]:
            events.append(dict(frame=k, label=label_))
    bodies = {n: "robot" for n in robot_bodies}
    bodies.update(shown)
    return dict(id=id_, label=label, basis=basis, objective=objective, evidence=[dict(trials_pin, role=f"solved trace {pid} ({pb[pid]['title']})")],
                recorded=recorded, display=display, time=[q[0] for q in rows], measure=[f_["measure"] for f_ in frames], events=events, follow="robot",
                bodies=bodies, groups={g_: [f_["groups"][g_] for f_ in frames] for g_ in names_})


pipe_at = lambda x_, z_: t_tr(x_ - SEAT[0], z_ - SEAT[1])
MODEL_NOTE = ("Planar compliant-contact trial from field-trials.json, played sample by sample in the robot frame. It is evidence playback: the layout "
              "check does not run it and the checked cases above cover its clearance.")
replay_set = dict(schema="agent-frc/review-replays@1", replays=[
    replay("replay-receive", "Recorded trial: station receive and drive away", "receive-mission", 3, OBJ_RECEIVE,
           "The mission receive: the pipe is released from rest on the station chute, drops into the inlet, passes the check flap and is captive; the robot "
           "leaves 50 ms after the flap closes at 3 m/s2, brakes, and the pipe seats. " + MODEL_NOTE,
           {pipe: "pipe", flap: "flap", **{n: "station" for n in STATION_SET}},
           lambda rec, q: dict(groups=dict(robot=I12, pipe=pipe_at(q[1], q[2]), flap=t_rot(FL, -q[4]), station=t_tr(-go_travel(rec, q[0]))),
                               measure=f"Trial time {q[0]:.3f} s · pipe centre X {q[1]:.0f}, Z {q[2]:.0f} mm · flap {q[4]:.0f} deg open · pipe speed {q[6]:.2f} m/s"),
           lambda rec: [("Released on the station chute", 0.0), ("Passes the check flap", rec["passedFlapS"]), ("Flap closed: captive", rec["flapClosedS"]),
                        ("Robot leaves the station", rec["goAtS"]), ("Robot stopped at the reef", rec["stopAtS"]), ("Seated", rec["seatedByS"])],
           ["Pipe centre path", "Check flap angle"],
           ["The station receding along the commanded drive-away profile (3 m/s2 for 0.5 s, cruise 0.5 s, brake)", "Pipe spin is not shown"]),
    replay("replay-carry-power-off", "Recorded trial: hit while carrying with the main power off", "custody-power-off-hit", 2, OBJ_DELIVER,
           "Custody with the main power off: an 8 g, 50 ms hit drives the seated pipe into the unpowered nip; the tread squeeze stops it and returns it "
           "to the seat. " + MODEL_NOTE,
           {pipe: "pipe", flap: "flap"},
           lambda rec, q: dict(groups=dict(robot=I12, pipe=pipe_at(q[1], q[2]), flap=t_rot(FL, -q[4])),
                               measure=f"Trial time {q[0]:.3f} s · pipe {q[1] - SEAT[0]:+.2f} mm from its seat along X · pipe speed {q[6]:.2f} m/s"),
           lambda rec: [("Seated, main power off", 0.0), ("Back on the seat", rec["trace"][-1][0])],
           ["Pipe centre path", "Check flap angle"], ["The robot is held still; the hit acts on the pipe as the trial's frame acceleration"]),
    replay("replay-eject", "Recorded trial: eject into the trough and withdraw", "eject-nominal", 4, OBJ_DELIVER,
           "The nominal eject at 12 V and 50 mm stand-off: the nip drives the pipe over the lower roller and down the deployed lip, it drops into the "
           "trough and comes to rest, and the robot withdraws 120 mm. " + MODEL_NOTE,
           {pipe: "pipe", **{n: "reef" for n in REEF_SET}, first_pipe: "reef"},
           lambda rec, q: dict(groups=dict(robot=I12, pipe=pipe_at(q[1] - q[5], q[2]), reef=t_tr(-q[5])),
                               measure=f"Trial time {q[0]:.3f} s · pipe centre X {q[1] - q[5]:.0f}, Z {q[2]:.0f} mm · pipe speed {q[6]:.2f} m/s"
                                       + (f" · robot withdrawn {-q[5]:.0f} mm" if q[5] else "")),
           lambda rec: [("Nip starts at 12 V", 0.0), ("Leaves the lip tip", rec["departure"]["t"]),
                        ("Robot withdraws", next(q[0] for q in rec["trace"] if q[5] != 0.0)), ("Supported 2 s after the withdrawal", rec["trace"][-1][0])],
           ["Pipe centre path", "Robot withdrawal (shown as the trough and first pipe receding)"],
           ["The first scored pipe rests one sector away and takes no part in this trial", "Pipe spin is not shown"]),
])
replay_path = ROOT / REPLAYS
if replay_path.exists():
    raise FileExistsError(REPLAYS)
replay_path.write_text(json.dumps(replay_set, separators=(",", ":"), allow_nan=False) + "\n", encoding="utf-8")
replays_pin = pin(REPLAYS)

presentation = dict(
    replays=[replays_pin], alliance="red",
    title="Fixed pocket, held roller nip and exit lip", context="Red 2 · Iteration 002 · B′x preliminary design",
    architecture="Red 2 B′x · fixed pocket, two-roller nip on a powered-hold planetary drive and deploy-once exit lip on four-module swerve",
    focus="CORAL pipe: the receive and eject cases follow two solved trials, from the station chute to the trough. Start with receive-01 and step through, "
          "or play the three recorded trials at the end of the motion list; then read the bench gates under the uncovered cases.",
    notes=["What it is: the Red 2 iteration 002 B'x preliminary blockout, Fixed pocket 1.0, on the carried-forward four-module swerve chassis. It supersedes the first draft: "
           "the self-locking worm is replaced by a planetary drive with a powered hold, the receive starts on the registered station chute, the robot leaves "
           "when the flap closes, the lip stop is sprung, and four sensors are selected and placed.",
           "Trial-driven: every receive and eject case is a straight stroke between two poses of a solved free-piece trace, within " + f"{CHORD_TOL:.0f} mm of it{TOL_NOTE}" + "; each frame shows "
           "the trial time. The lip deploy and the park drive are prescribed. The trials are a planar compliant-contact model: stiffness, friction and "
           "restitution are swept, not measured.",
           "Simplified: the chassis, module housings and electronics are oriented-box unions or supplier-bounds boxes; the steer sweeps are 4 mm-stepped revolved "
           "envelopes; the rollers are their rigid cores; the gearbox is a planning envelope.",
           "Attachment: every robot part is drawn within the 2 mm check reserve of the member that carries it, with three listed exceptions of under "
           "7 mm whose brackets are not drawn: the electronics stand 4-7 mm above the equipment sheets on standoffs, and the nip gearbox, latch servo and "
           "lip switch stand 4.5 mm off the side plates. The posts and side plates land on the two pocket landing shelves.",
           "Not robot parts: the two orange hands are unmounted service allowances, drawn at the end of their reach. The sand-coloured station and "
           "reef are FIRST's official field parts standing on the carpet, and the gold pipe in the trough is the first scored pipe. The checker lists every "
           "fixture in every case, so the station and the reef appear together although the robot is at one of them at a time; hide the field parts to see "
           "the robot alone. The three recorded-trial entries at the end of the motion list show only the fixture each trial used.",
           "Field: the station and reef solids are the official parts as FIRST drew them, placed at the two docks. The check uses the parts the mission "
           "touches or passes (barrier, panels, chute plate, guard, side plates and uprights; trough lip, floors, risers and the two branches over the "
           "trough); fasteners, ribs and the rest of both assemblies are left out. The trials use the same parts' section, measured on the solids.",
           "Not modelled: pipe yaw and end contact, the chute plate's slots, lip and hood flex, peers. Sizing is by hand "
           "formula (structure-checks.json), not finite elements.",
           "Power off: the carry case replays a hit with the main power off. The tread squeeze alone holds the pipe; the limits are in the retention review. "
           + BRAKE_REQUIREMENT,
           "Bench gates G0-G7 are recorded with pass and fail numbers and are all open; no bench article exists. The guarded-pan baseline A is the fallback if "
           "G1, G3 or G7 fails.",
           "Rules: read from the final 2025 manual; the project register is the kickoff V0 packet and the V0 archive could not be re-read on 2026-11-22."],
    components=components, required=required, loadMembers=load_members, loadPaths=support,
    runningAdditions=[dict(name="one-colour bumpers", kg=[7.5, 7.5], members=["Bumper ring"]), dict(name="battery", kg=[6.17, 6.17], members=["Dressed battery"]),
                      dict(name="carried CORAL", kg=[0.816466, 0.816466], members=["CORAL pipe"])],
    runningBasis="Carried CORAL 0.816466 kg plus the R103-excluded battery 6.17 kg and one-colour bumpers 7.5 kg",
    caseKinds={c["id"]: "nominal" for c in cases}, uncoveredCases=uncovered, caseTimelines=timelines, openingCase=RECEIVE_IDS[0],
    loadPathsSource=load_paths, selectedInterface=carry, retention=retention, findings=findings, rulings=ruling,
    benchGates=bench, iterationChoice=choice, fieldTrials=trials_pin, structureChecks=struct_pin, secondTrip=second_trip)
CONTROL_CASE = next(c for c in RECEIVE_IDS if "-pocket" in c)
plan = dict(schema="agent-frc/preliminary-layout@1", issue=ISSUE, design=design, step=step, motion=motion, datum=datum,
            groups=groups, mounts=mounts, stages=stages, missingInputs=[],
            control=dict(case=CONTROL_CASE, displaced="Hood (removable)", moving=pipe, offsetMm=[0.0, 0.0, -12.0],
                         basis="The removable hood seated 12 mm low (a missing tab or a sagging 1/16 in sheet) over the 10 mm receive gap"),
            cycle=cycle, **({"view": VIEW} if VIEW else {}), budgets=budgets, premises=premises,
            attachment=dict(gapMm=3.5),      # the official reef stands 3.175 mm above the carpet datum on its base plates
            presentation=presentation, narrativeReview=narrative)
write(f"{REL}/layout.json", plan)
has = lambda *keys: [c for c in cycle if any(k in c for k in keys)]
bt = lambda measure, pass_, fail, unit=None: dict(benchTest=dict(measure=measure, **{"pass": pass_}, fail=fail, **({"unit": unit} if unit else {})))
risks = [
    dict(id="bench-G0", risk="Installed tread stiffness is unmeasured; receive fails at 10 N/mm and is marginal at 20, and the drive stalls at 240 N/mm",
         **bt("Force to press a pipe 6 mm into each installed roller", f"{BAND[0] * 6:.0f} to {BAND[1] * 6:.0f}", "below 120 or above 1440", "N")),
    dict(id="bench-G1", risk="Station receive: captive time, pass-through and escape on the real article",
         **bt("Powered feeds captive within 0.50 s of passing the flap, out of 40; disabled feeds kept, out of 10", "40 and 10", "any pipe through the nip or out of the inlet")),
    dict(id="bench-G2", risk="Eject into the trough across stand-off and bus voltage; gearbox efficiency is assumed",
         **bt("Ejects captured and supported 2 s after withdrawal, out of 150 (5 stand-offs x 3 voltages x 10)", ">= 147, every cell >= 9/10", "any cell <= 7/10 or a stall at 9 V")),
    dict(id="bench-G3", risk="Carry custody under 1-8 g pulses and when leaving the station on the flap signal",
         **bt("Retained out of 60 powered and 40 disabled pulses; receive-and-go runs seated within 0.5 s, out of 10", "60, 40 and 10", "any loss at 3 g or below")),
    dict(id="bench-G4", risk="Two-trip cycle end to end; lateral dock scatter at the two sectors",
         **bt("Two-trip cycles with both pipes credited, out of 20; lateral dock error", f">= 19; within {LATERAL_LIMIT:.0f}", f"< 19 or any error beyond {end_gap / 2:.1f}", "cycles; mm")),
    dict(id="bench-G5", risk="Seated beam, flap and lip switches and the reef range interlock on real hardware",
         **bt("Displaced or missing pipes flagged within 50 ms; range error; eject commands accepted at 150 mm or beyond", "all flagged; within 15; 0", "any missed loss or any eject accepted beyond 150 mm", "mm")),
    dict(id="bench-G6", risk="Lip sag, landing and frontal hit on the sprung stop",
         **bt("Tip sag under the pipe; permanent set after 50 pushes of 300 N and 5 wall hits at 1 m/s", "<= 3; <= 1", "set over 1 or any crack", "mm")),
    dict(id="bench-G7", risk="Custody with the power off rests on the installed tread squeeze, which is unmeasured",
         **bt("Force to push a seated pipe through the unpowered nip; pipes kept nose-down for 60 s out of 5; kept through 8 g, 50 ms pulses out of 20",
              f">= {PUSH_PASS_N}; 5; 20", f"< {PUSH_FAIL_N}, or any pipe lost", "N; count; count")),
    dict(id="power-weak-pack", risk=f"Loaded bus voltage on a weak pack while driving away is {weak_min:.2f} V against the 9.0 V floor: "
                                    f"{weak_min - 9.0:.2f} V of margin, on a lumped pack model",
         **bt("Lowest bus voltage logged over ten 3 m/s2 launches with the nip on hold, on a pack at the weak-condition resistance",
              ">= 9.0 on every launch", "below 9.0 on any launch", "V")),
    dict(id="mass-headroom", risk=f"Heavy inspection estimate {total[1]:.2f} kg leaves {mass["rule"]["limitKg"] - total[1]:.2f} kg under the {mass["rule"]["limitKg"]:.2f} kg limit, "
                                  "with 3.4 kg of joints and 5.5 kg of electrical parts carried as allowances",
         **bt("Inspection weight of the finished robot without bumpers and battery", f"<= {mass['rule']['limitKg']:.2f}", f"> {mass['rule']['limitKg']:.2f}", "kg")),
    dict(id="PD17-power-off-custody", risk="With the worm removed nothing locks the nip when the robot is disabled or the main power is off",
         closedBy=f"Trials show the tread squeeze alone holds: {frac(CU['powerOffRequired'])} required stop and hit cases retained with the power off, checked case "
                  f"{CARRY_IDS[0]}, premise power-off-custody; brake idle mode is set (design-findings.json PD17). The installed squeeze is bench gate G7"),
    dict(id="second-trip-same-section", risk="A second delivery into the section that already holds the first pipe fails in the trials",
         closedBy=f"Premise lateral-dock-error holds each dock within {LATERAL_LIMIT:.0f} mm so the pipes land in different sectors (second-trip.json); the dock scatter itself is bench gate G4"),
    dict(id="PD3-worm", risk="The first draft needed a self-locking worm that held 4.10 N m, with no part selected",
         closedBy="Replaced by a NEO 550 on a MAXPlanetary 36:1 with a powered hold; trials show the lock is not needed (design-findings.json PD3/PD6)"),
    dict(id="PD11-seat-time", risk="The first draft seat time moved from 0.15 to 0.69 s with an unmeasured contact stiffness",
         closedBy="The robot leaves on the flap-closed signal; seat time no longer holds it (design-findings.json PD11)"),
    dict(id="PD7-lip", risk="The first draft lip stop and tip load were unsized and the lip leads the bumper by 24.5 mm",
         closedBy="Sprung deployed stop sized in structure-checks.json L1-L9; the over-travel swing is checked case service-lip-over-travel"),
    dict(id="PD12-plates", risk="The first draft side-plate landing and bolts were unsized", closedBy="Sized in structure-checks.json P1-P3 and S1-S2 (steel shafts)"),
    dict(id="PD9-g412", risk="The first draft left the G412 ruling open with no zone sensing",
         closedBy="Ruling and reef range interlock in rules-rulings.json; premise g412-eject-interlock"),
    dict(id="PD14-station", risk="The first draft had no station fixture and started the receive at an energy estimate",
         closedBy="The station chute is a registered fixture and the receive trial starts on it"),
]
write(f"{REL}/missions.json", dict(
    missionCases=[
        dict(case=MISSION["delivered"], kind="operating-case", accomplishes="delivered", objective=OBJ_DELIVER),
        dict(case=MISSION["ejectLoaded"], kind="operating-case", accomplishes="delivered", objective=OBJ_DELIVER),
        dict(case=MISSION["acquired"], kind="operating-case", accomplishes="acquired", objective=OBJ_RECEIVE)],
    cycle=dict(
        acquire=dict(cases=has("-station-chute", "-drop", "-inlet", "-pocket", "-captive")),
        retain=dict(cases=has("-drive-away", "-brake", "-seat", "-power-off-hit")),
        transfer=dict(cases=has("-nip", "-lip")),
        deliver=dict(cases=has("-flight", "-trough")),
        reset=dict(notApplicable="The pocket is fixed and the lip stays deployed: after the eject the robot is ready for the next station receive with no reset motion"),
        endgame=dict(notApplicable="The selected role parks; it does not climb"),
        park=dict(cases=["withdraw-park"]),
        service=dict(cases=[c["id"] for c in cases if c["stage"] == "service"])),
    risks=risks))
print(len(cases), "cases:", n_rec, "receive,", n_cu, "carry,", n_ej, "eject; mass", round(total[1], 3), "weak bus", round(weak_min, 3), "captive", captive)
