"""Write the motion study, budgets, premises and layout plan for the iteration 002 preliminary package.

    python -B write_package.py PROJECT_ROOT PLUGIN_SCRIPTS

Reads iterations/002/design/preliminary/{design.json, layout.step, layout-blocks.json} and writes the
remaining project records beside them. Every pinned file is project-relative.
"""
import hashlib
import itertools
import json
import math
import shutil
import sys
from pathlib import Path

ROOT = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(Path(sys.argv[2]).resolve()))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from showcase_contacts import load_solids, box_gap  # noqa: E402
from layout_occ import aabb, distance  # noqa: E402

REL = "iterations/002/design/preliminary"
OUT = ROOT / REL
ISSUE = "red2-002-preliminary-layout-A"
GAP = 2.0


CHASSIS = "artifacts/resume-20261121/screws-place-01/four-seated-screws.step"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pin(rel):
    return {"file": rel, "sha256": sha(ROOT / rel)}


def write(rel, obj):
    path = ROOT / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise FileExistsError(rel)
    path.write_text(json.dumps(obj, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return pin(rel)


def read(rel):
    return json.loads((ROOT / rel).read_text(encoding="utf-8-sig"))


def copy_evidence(src, rel, expected):
    if sha(src) != expected:
        raise ValueError("evidence digest changed: " + str(src))
    dest = ROOT / rel
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists():
        raise FileExistsError(rel)
    shutil.copyfile(src, dest)
    return pin(rel)


blocks = read(f"{REL}/layout-blocks.json")
poses = blocks["poses"]
design = pin(f"{REL}/design.json")
step = pin(f"{REL}/layout.step")
datum = dict(name="Iteration 002 robot frame = taps-02 root: chassis centre on the factory-tire floor (Z = 0), "
                  "+X toward the dump (reef) face, +Y port, +Z up, millimetres",
             robotToStepMm=[1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])
by_group = {}
for b in blocks["blocks"]:
    by_group.setdefault(b["group"], []).append(b["name"])
roles = {b["group"]: b["role"] for b in blocks["blocks"]}
groups = {g: dict(role=roles[g], members=m) for g, m in by_group.items()}

# ------------------------------------------------------------------ motions
PIV = poses["pivotMm"]
def rot(origin, axis, deg):
    return dict(kind="rotation", originMm=list(origin), axis=list(axis), travelDeg=deg)
def tr(v):
    return dict(kind="translation", travelMm=[float(x) for x in v])
def sub(a, b):
    return [a[0] - b[0], 0.0, a[1] - b[1]]

pipe = "CORAL pipe"; pan = "Guarded pan"
P0, P1, PS = poses["pipe"]["stationExitXZ"], poses["pipe"]["chuteEndXZ"], poses["pipe"]["seatStowXZ"]
Q = (PS[0] + 16.9, PS[1] + 81.6)              # above the seat along the stowed floor, clear of the panel edge
TIP = poses["tipDeg"]
tip = rot(PIV, (0, 1, 0), TIP)
HOLD = 0.05                                   # stow placement band (operating-intent custodyCriteria.stopPlacement)
hold = rot(PIV, (0, 1, 0), HOLD)
tip_rest = rot(PIV, (0, 1, 0), TIP - HOLD)
a = math.radians(2.0)
dump_dir = (math.cos(a), -math.sin(a))         # floor direction at sim theta -2 deg
du = poses["pipe"]["circumradiusMm"] + poses["pipe"]["seatGapMm"] + 340.0 - (poses["pipe"]["seatFlatXZ"][0] - PIV[0])
release = tr([dump_dir[0] * du, 0.0, dump_dir[1] * du])
sweeps = by_group["steer sweeps"]
axes = {f"Module {k} steer sweep": v["axis"] for k, v in poses["steer"].items()}
steer = lambda deg: {n: [rot([axes[n][0], axes[n][1], 0.0], (0, 0, 1), deg)] for n in sweeps}
cassette = by_group["pan cassette"]

cases = [
    dict(id="receive-chute", maxDepth=11, stage="operating", minGapMm=GAP,
         basis="Station-issued pipe drops from the station chute exit and slides down feed panels 2 and 3 to the panel 3 edge; "
               "the rolling chute contact is partitioned 3.5 mm above the panel surface",
         initialPose={pipe: [tr(sub(P0, PS))]}, moves={pipe: [tr(sub(P1, P0))]}),
    dict(id="receive-over-edge", stage="operating", minGapMm=GAP,
         basis="The pipe clears the panel 3 front edge into the open top of the stowed pan pocket",
         initialPose={pipe: [tr(sub(P1, PS))]}, moves={pipe: [tr(sub(Q, P1))]}),
    dict(id="receive-seat", stage="operating", minGapMm=GAP,
         basis="The pipe settles down the stowed floor onto the slotted rear lip; seated contact partitioned at 3.0 mm",
         initialPose={pipe: [tr(sub(Q, PS))]}, moves={pipe: [tr(sub(PS, Q))]}),
    dict(id="loaded-carry", stage="operating", minGapMm=GAP,
         basis="Loaded carry: the pan holds stow within its +/-0.05 deg placement band (operating-intent stopPlacement) and here "
               "opens to the band edge with the seated pipe. Steering needs no motion: each steer sweep is a static revolved "
               "envelope that already contains every azimuth. Peer-event openings (3.9-51.5 deg, study-06) lie inside the tip sweep "
               "and are reviewed in retention-review.json",
         moves={pan: [hold], pipe: [hold]}),
    dict(id="tip-to-dump", maxDepth=11, stage="operating", minGapMm=GAP,
         basis="Docked at the L1 trough, the pan tips from the stow band edge to the commanded dump (-2 deg, 72 deg in all) with the seated pipe",
         initialPose={pan: [hold], pipe: [hold]}, moves={pan: [tip_rest], pipe: [tip_rest]}),
    dict(id="release", maxDepth=11, stage="operating", minGapMm=GAP,
         basis="At dump the pipe slides off the floor past the front edge and leaves the robot",
         initialPose={pan: [hold, tip_rest], pipe: [hold, tip_rest]}, moves={pipe: [release]}),
    dict(id="reset-to-stow", maxDepth=11, stage="operating", minGapMm=GAP,
         basis="Empty pan returns 72 deg to stow while the robot is parked at the trough; the released pipe stays ahead",
         initialPose={pan: [hold, tip_rest], pipe: [hold, tip_rest, release]}, moves={pan: [rot(PIV, (0, 1, 0), -TIP)]}),
    dict(id="park", stage="park", minGapMm=GAP,
         basis="The robot withdraws 300 mm from the trough and parks with the pan stowed; in the robot frame the scored pipe "
               "recedes ahead. Modules re-azimuth inside their static revolved envelopes",
         initialPose={pipe: [hold, tip_rest, release]}, moves={pipe: [tr([300.0, 0.0, 0.0])]}),
    dict(id="service-unload-pipe", stage="service", minGapMm=GAP,
         basis="Lift an unscored pipe straight out of the stowed pan pocket by hand before service",
         moves={pipe: [tr([0, 0, 440.0])]}, extract=[pipe]),
    dict(id="service-pan-cassette-lift", maxDepth=11, stage="service", minGapMm=GAP,
         basis="Unbolt the four cradle end clips and the knee-strut feet and lift the cassette with the stowed pan vertically clear",
         removed={pipe: "service-unload-pipe"},
         moves={n: [tr([0, 0, 470.0])] for n in cassette + [pan]}, extract=cassette + [pan]),
    dict(id="service-battery-lift", stage="service", minGapMm=GAP,
         basis="After unplugging, lift the dressed battery 250 mm through the centre gap between the equipment sheets",
         removed={pipe: "service-unload-pipe"}, moves={"Dressed battery": [tr([0, 0, 250.0])]}),
    dict(id="service-battery-rear-carry", stage="service", minGapMm=GAP,
         basis="Continue from the 250 mm lift rearward under feed panel 1 and over the rear bumper until clear of the robot",
         removed={pipe: "service-unload-pipe"}, initialPose={"Dressed battery": [tr([0, 0, 250.0])]},
         moves={"Dressed battery": [tr([-420.0, 0, 0])]}),
    dict(id="service-plug-disconnect", stage="service", minGapMm=GAP,
         basis="A hand reaches down to grip the SB50 pair from above",
         removed={pipe: "service-unload-pipe"}, moves={"Plug grip hand": [tr([0, 0, 199.1 - 455.0])]}),
    dict(id="service-breaker-reset", stage="service", minGapMm=GAP,
         basis="A hand reaches the CB185 reset button outboard of the port stringer extension",
         removed={pipe: "service-unload-pipe"}, moves={"Breaker reset hand": [tr([0, 0, 257.2 - 600.0])]}),
]
motion = write(f"{REL}/motion.json", dict(schema="agent-frc/mechanical-motion@1", step=step,
                                          layoutDesignSha256=design["sha256"], layoutDatum=datum, cases=cases))

operating_basis = ("2.0 mm reserve between distinct reserves: 0.5 mm for the +/-0.05 deg stow placement at the 560 mm pan radius, "
                   "1.5 mm fabrication and working-load deflection; every mated face is partitioned at 2.1 mm")
service_basis = "2.0 mm clearance for hand, tool and withdrawal paths around conservative envelopes; mated faces partitioned at 2.1 mm"
park_basis = "2.0 mm packaging reserve between parked reserves, the same partition as operation"
stages = {}
for c in cases:
    moving = sorted({g for g, m in by_group.items() if set(m) & set(c["moves"])})
    carried = {"pipe": "pan"} if c["id"] in ("loaded-carry", "tip-to-dump") else {}
    basis = {"operating": operating_basis, "park": park_basis, "service": service_basis}[c["stage"]]
    stages[c["id"]] = dict(purpose=c["stage"], movingGroups=moving, carriedWith=carried, allowanceBasis=basis)
cycle = [c["id"] for c in cases if c["stage"] != "service"]

# ------------------------------------------------------------------ mounts (every installed reserve)
by_name = {b["name"]: b for b in blocks["blocks"]}
def clamp_point(block, host):
    bb, hb = by_name[block]["boundsMm"], by_name[host]["boundsMm"]
    c = [(bb[i] + bb[i + 3]) / 2 for i in range(3)]
    return [round(min(max(c[i], hb[i]), hb[i + 3]), 3) for i in range(3)]
def nearest_host(prefix, block):
    bb_ = by_name[block]["boundsMm"]; c = [(bb_[i] + bb_[i + 3]) / 2 for i in range(3)]
    def gap(n):
        b = by_name[n]["boundsMm"]
        return sum(max(b[i] - c[i], 0, c[i] - b[i + 3]) ** 2 for i in range(3))
    return min((n for n in by_group["structure"] if n.startswith(prefix)), key=gap)


def host_at(prefix, point):
    for n in by_group["structure"]:
        if n.startswith(prefix):
            b = by_name[n]["boundsMm"]
            if all(b[i] - 1e-6 <= point[i] <= b[i + 3] + 1e-6 for i in range(3)):
                return n
    raise ValueError("no host at " + prefix + str(point))
mounts = [
    dict(group="pan", on="Pan cassette", pointMm=PIV, basis="Pan axis through the REV-21-1915 hex bearings in both carriers (Y +/-195)"),
    dict(group="pan cassette", on=host_at("Chassis pan trusses and stringers", [110.0, 232.475, 480.0]), pointMm=[110.0, 232.475, 480.0],
         basis="Cradle 110 port end clip on the upper pan stringer inboard face (pan/geometry.json attachments)"),
    dict(group="upper support", on=host_at("Chassis pan trusses and stringers", [100.6, 242.0, 480.0]), pointMm=[100.6, 242.0, 480.0],
         basis="Port stringer extension butt splice to the upper pan stringer (X 100.475; datum 0.125 mm inside the stringer reserve)"),
    dict(group="inlet", on=next(n for n in by_group["upper support"] if by_name[n]["boundsMm"][1] <= 242.0 <= by_name[n]["boundsMm"][4]),
         pointMm=[-242.5, 242.0, 489.5],
         basis="Port inlet mounting foot 1 on the stringer extension top"),
    dict(group="battery", on="Battery tray and retention", pointMm=[-150.0, 0.0, 29.9], basis="Battery case bottom on the tray"),
    dict(group="battery retention", on=host_at("Chassis sills, decks and crossmembers", [-210.0, 110.0, 111.13]), pointMm=[-210.0, 110.0, 111.13], basis="Port rear hanger under inboard sill 1"),
    dict(group="bumpers", on=host_at("Chassis perimeter", [370.0, 0.0, 150.0]), pointMm=[370.0, 0.0, 150.0], basis="Front bumper backer on the front perimeter face (mount undesigned)"),
]
for k, v in poses["steer"].items():
    mounts.append(dict(group="steer sweeps", on=f"Module {k} housing", pointMm=[v["axis"][0], v["axis"][1], 100.0],
                       basis="Steer axis through the A035XPO turret bearing"))
for n in by_group["module housings"]:
    if "part" not in n:
        mounts.append(dict(group="module housings", on=nearest_host("Chassis cassettes and socket rails", n), pointMm=clamp_point(n, nearest_host("Chassis cassettes and socket rails", n)),
                           basis="Six 10-32 frame screws clamping the 50.8 mm socket rail between the module plates (nearest host point to the housing centre)"))
for g in ("electronics", "wiring"):
    for n in by_group[g]:
        mounts.append(dict(group=g, on=nearest_host("Chassis sills, decks and crossmembers", n), pointMm=clamp_point(n, nearest_host("Chassis sills, decks and crossmembers", n)),
                           basis="Proposed attachment: the host-envelope point nearest the reserve centre (bracket undesigned)"))

# ------------------------------------------------------------------ budgets
mass = read("artifacts/resume-20261121/mass-01/mass-study.json")
line = {r["id"]: r["kg"] for r in mass["ledger"]["lines"]}
growth = {r["item"]: r["kg"] for r in mass["ledger"]["growth"]}
g = lambda *ids: sum(growth[i] for i in ids)
fab = by_group["pan"] + by_group["pan cassette"] + by_group["upper support"] + by_group["inlet"]
ledger = [
    ("sds-modules", by_group["steer sweeps"] + by_group["module housings"], line["modules"], g("modules"),
     "Catalog 6.0 lb SDS MK4i L2 NEO x 4, wiring included; 3 % item growth (mass-01)"),
    ("chassis-fabricated", by_group["structure"], line["chassis-fabricated"], g("chassis-fabricated"),
     "146 taps-02 solids by measured CAD volume at 2700 kg/m3; 2 % growth (mass-01)"),
    ("chassis-purchased-hardware", [], line["chassis-screws"] + line["taps02-clamp-screws"] + line["axle-spacer"],
     g("chassis-screws", "taps02-clamp-screws", "axle-spacer"),
     "116 McMaster chassis screws, 4 seated clamp screws and the steel spacer (inside the chassis reserve); 5-20 % growth"),
    ("pan-package-fabricated", fab, line["pan-fabricated"], g("pan-fabricated"),
     "r3 moving pan, cassette, upper support and fixed inlet by measured volume; 2 % growth"),
    ("pan-package-purchased", [], line["pan-purchased-sourced"] + line["pan-purchased-estimated"] + line["pan-fasteners"],
     g("pan-purchased-sourced", "pan-purchased-estimated", "pan-fasteners", "pan-open-articles"),
     "6 sourced REV articles, 11 estimated purchases including the 0.200 kg clutch, pan fasteners, plus 0.15 kg open articles"),
    ("joining-allowance", [], line["joining-allowance"], 0.0, "Named allowance: unresolved joints, splices and backing (mass-01, single layer)"),
    ("electrical-allowance", by_group["electronics"] + by_group["wiring"], line["electrical-allowance"], 0.0,
     "Named allowance: controllers, PDH, roboRIO, radio, breaker, robot-side plug, harness and mounts (mass-01)"),
    ("battery-tray-retention", by_group["battery retention"], 0.35, 0.0,
     "Named allowance carved from the mass-01 1.5 kg hardware-and-finish allowance so AF7 names it; not yet designed"),
    ("permanent-bumper-mounts", [], 0.30, 0.0,
     "Named allowance carved from the mass-01 hardware-and-finish allowance (AF7); R103 counts permanent mounts"),
    ("hardware-finish-remainder", [], line["hardware-finish-allowance"] - 0.65, 0.0,
     "Named allowance: the remaining unspecified installed hardware and finish"),
    ("bumpers-excluded", by_group["bumpers"], 0.0, 0.0, "R103 excludes bumpers; the 7.5 kg one-colour set is carried in the running mass"),
    ("battery-excluded", by_group["battery"], 0.0, 0.0,
     "R103 excludes the battery, battery-side SB50, lugs, insulation and up to 12 in of lead per leg; 6.17 kg carried in the running mass"),
]
masses = [dict(id=k, members=m, kg=[lo, lo + up], basis=b) for k, m, lo, up, b in ledger]
total = [0.0, 0.0]
for r in masses:
    total = [x + y for x, y in zip(total, r["kg"])]
payload = [0.816466 + 6.17 + 7.5] * 2
running = [x + y for x, y in zip(total, payload)]

SOURCES = ["drive", "steer", "pan", "controls"]
docked = dict(drive=[0, 10], steer=[0, 4], pan=[0, 30], controls=[3, 6.5])
moving = dict(drive=[0, 52], steer=[0, 20], pan=[0, 7.45], controls=[3, 6.5])
stage_loads = {"receive-chute": docked, "receive-over-edge": docked, "receive-seat": docked, "loaded-carry": moving,
               "tip-to-dump": docked, "release": docked, "reset-to-stow": docked,
               "park": dict(moving, pan=[0, 4.26])}
pan_stage = {"receive-chute", "receive-over-edge", "receive-seat", "tip-to-dump", "release", "reset-to-stow"}
packs = {"nominal": ([11.8, 12.4], [0.025, 0.027]), "weak": ([11.8, 11.8], [0.032, 0.032])}
commissioned, electrical = {}, {}
for stage in cycle:
    for cond, (voc, res) in packs.items():
        key = f"{stage}-{cond}"
        commissioned[key] = dict(stage=stage, batteryCondition=cond, sources=SOURCES,
                                 duty=("Hold or move the pan under its 30 A limit while the chassis is docked" if stage in pan_stage
                                       else "Drive the loaded robot at up to 1.70 m/s cruise with module re-azimuth"),
                                 requiredMotionMinV=9.0 if stage in pan_stage else 6.0)
        electrical[key] = dict(
            loadedKg=running, batterySideA=stage_loads[stage], openCircuitV=voc, resistanceOhm=res,
            basis=("Battery-side source ranges. Drive 0-52 A covers the 60.07 A adverse replay bus total less pan and controls "
                   "(limited-field-four-terminals lowestBus); steer 0-20 A from the 0.225 N m scrub-limited motor torque at 189 rad/s "
                   "per module (four modules, timed-bus 9.5 A phase peak); pan bounded by its 30 A phase limit (docked) or the 7.45 A "
                   "loaded-driving peak; controls 3-6.5 A for the 68 W auxiliary budget. Pack: fresh 12.4 V / 25 mOhm and screened "
                   "11.8 V / 27 mOhm (nominal), weak 11.8 V / 32 mOhm (experiments/six-wheel/conditioned-match/supply.py)."))
limits = write(f"{REL}/budget-limits.json", dict(
    basis="R103 115 lb (2025 kickoff V0 pp. 76/86/112 per inspection-mass-552 ruleBasis; season rules-v0.json has no R103 entry) "
          "and the project 9 V branch floor used by every pan and field replay (pan-stop-study-05 criteria branch >= 9 V)",
    inspectionLimitKg=mass["rule"]["limitKg"], operatingMinV=9.0,
    massLines=[r["id"] for r in masses], electricalCases=commissioned))
budgets = write(f"{REL}/budgets.json", dict(
    issue=ISSUE, step=step, motion=motion, design=design, requirements=limits,
    runningPayloadKg=payload,
    runningPayloadBasis="Carried CORAL 0.816466 kg plus the R103-excluded battery 6.17 kg and one-colour bumpers 7.5 kg, so the "
                        "running mass for duty includes everything the inspection ledger excludes",
    masses=masses, electricalCases=electrical))
weak_min = min(voc[0] - sum(v[1] for v in stage_loads[s].values()) * res[1]
               for s in cycle for cond, (voc, res) in packs.items() if cond == "weak")

# ------------------------------------------------------------------ static electronics clearance (installed scene)
solids_, _ = load_solids(ROOT / step["file"])
bb = {k: aabb(v) for k, v in solids_.items()}
watch = set(by_group["electronics"] + by_group["wiring"] + by_group["battery"] + by_group["battery retention"])
elec_min = (math.inf, None)
for x, y in itertools.combinations(sorted(solids_), 2):
    if (x in watch or y in watch) and box_gap(bb[x], bb[y]) < 30:
        d = distance(solids_[x], solids_[y])
        if d < elec_min[0]:
            elec_min = (d, [x, y])

# ------------------------------------------------------------------ evidence copies (outside-project custody into the issue)
native = Path("WORKSTATION/Documents/FRC Native/2025/resume-20261121-2025/pan-stop-01")
ev06 = copy_evidence(native / "pan-stop-study-06.json", f"{REL}/evidence/pan-stop-study-06.json",
                     sha(native / "pan-stop-study-06.json"))
p5 = read("artifacts/resume-20261121/pan-stop-01/pan-stop-study-05.json")["replay"]["P5-K5420-limit"]["fourTerminal"]
evfield = copy_evidence(native / "study05" / "P5-K5420-limit.field-bus.json", f"{REL}/evidence/P5-K5420-limit.field-bus.json", p5["sha256"])
field = read(evfield["file"])
ready_after_feed = max(27.03207263530179 - r["readyBeforeCarryS"] - 21.1 for k, r in field["runs"].items() if "coral-A" in k)
cmp1 = read("artifacts/resume-20261121/fixed-path-01/comparison.json")
cmp2 = read("artifacts/resume-20261121/fixed-path-01/comparison-02.json")
arrival = max(v["speedMps"] for v in cmp1["fixedPath"]["receiveScreen"]["arrivals"].values()) if "receiveScreen" in cmp1.get("fixedPath", {}) else None
if arrival is None:
    def find(o):
        if isinstance(o, dict):
            if "receiveScreen" in o:
                return o["receiveScreen"]
            for v in o.values():
                r = find(v)
                if r:
                    return r
        if isinstance(o, list):
            for v in o:
                r = find(v)
                if r:
                    return r
    arrival = max(v["speedMps"] for v in find(cmp1)["arrivals"].values())
selected = "guarded pan" if cmp2["verdict"]["result"] == "FAIL" and cmp2["verdict"]["selected"].startswith("guarded pan") else "fixed path"

measure = write(f"{REL}/evidence/premise-measurements.json", dict(
    schema="red2/premise-measurements@1", issue=ISSUE, designSha256=design["sha256"], stepSha256=step["sha256"],
    receiveFace="front",
    receiveReadyAfterFeedS=ready_after_feed,
    dockSupportMarginMm=p5["minSupportMarginMm"],
    stationArrivalMps=arrival,
    inspectionHeavyKg=total[1],
    weakPackMinBusV=weak_min,
    electronicsMinClearanceMm=elec_min[0],
    selectedArchitecture=selected,
    chassisInterface="002-connected-machining-654-r6",
    panPackageRevision=read("iterations/002/design/pan/geometry.json")["revision"],
    seatedClampScrews="four 91251A542 seated in UNC-2B taps",
    provenance=dict(
        receiveFace="This issue places the inlet mouth on the robot -X face, the face that docks to the station; the directional "
                    "Field Lab names that pickup line the front edge (kickoff/selected-role.md cycle, commissioning.md).",
        receiveReadyAfterFeedS=dict(source=evfield, field="runs[*/coral-A/30].readyBeforeCarryS",
                                    note="ready = carry start 27.032 s - readyBeforeCarryS; feed at 21.1 s; r3 P5-K5420 30 A replay, worst case"),
        dockSupportMarginMm=dict(source=pin("artifacts/resume-20261121/pan-stop-01/pan-stop-study-05.json"),
                                 field="replay.P5-K5420-limit.fourTerminal.minSupportMarginMm",
                                 note="selected r3 proposal, powered at 30 A, 12 cases at the 12 mm dock stand-off"),
        stationArrivalMps=dict(source=pin("artifacts/resume-20261121/fixed-path-01/comparison.json"),
                               field="receiveScreen.arrivals.adverse.speedMps", note="energy estimate, adverse admission"),
        inspectionHeavyKg=dict(source=budgets, field="sum of masses[].kg upper", note="equals the budget receipt upper estimate"),
        weakPackMinBusV=dict(source=budgets, field="min over weak cases of Voc_low - sum(upper source A) x R_high"),
        electronicsMinClearanceMm=dict(source=step, pair=elec_min[1], note="installed scene, every electronics, wiring, battery and retention reserve against every other reserve"),
        chassisInterface=dict(source=pin("iterations/002/design/cad/fusion-authority.json"), field="registered revision",
                              note="the 658-solid STEP of this issue is that revision plus the four seated screws"),
        panPackageRevision=dict(source=pin("iterations/002/design/pan/geometry.json"), field="revision"),
        seatedClampScrews=dict(source=pin("artifacts/resume-20261121/screws-place-01/native-receipt.json"), field="status",
                               note="four seated purchased screws placed; mechanical review pending"),
        selectedArchitecture=dict(source=pin("artifacts/resume-20261121/fixed-path-01/comparison-02.json"),
                                  field="verdict", value=cmp2["verdict"]))))

premise_specs = [
    ("station-receive-face", "conceptual-design", "face", {"equals": "front"}, ["inlet", "pan", "pipe"],
     "Both TELEOP station receives (18-153 s) at the red station, front approach", "receiveFace",
     pin("kickoff/selected-role.md"), "Selected station-fed L1 role: the robot docks its pickup face to the station chute"),
    ("station-receive-time", "conceptual-design", "s", {"range": [0.0, 5.932]}, ["inlet", "pan", "pipe"],
     "Station feed at 21.1 s to scheduled carry start at 27.032 s (coral-A), adverse mass and battery", "receiveReadyAfterFeedS",
     pin("iterations/002/design/strategy/pan-route-12mm.json"), "Receive must finish (pan ready) before the scheduled carry start"),
    ("dock-support-margin", "conceptual-design", "mm", {"range": [1.828296, 1000.0]}, ["pan", "pipe", "steer sweeps"],
     "Twelve L1 placements, sectors A and B, nominal and adverse, at the 12 mm earlier dock stand-off", "dockSupportMarginMm",
     pin("iterations/002/design/strategy/limited-field-12mm-executed.json"),
     "The 12 mm stand-off accepted on its 1.828296 mm post-contact support baseline; physical dock error must stay inside it"),
    ("station-arrival-speed", "conceptual-design", "m/s", {"range": [0.0, 2.5]}, ["inlet", "pan", "pipe"],
     "Pipe arrival at the receiver from the station chute, adverse admission", "stationArrivalMps",
     pin("artifacts/resume-20261121/fixed-path-01/comparison.json"),
     "The pan receive replays are accepted for station arrivals up to about 2.5 m/s"),
    ("inspection-mass", "preliminary-design", "kg", {"range": [0.0, mass["rule"]["limitKg"]]},
     sorted(g_ for g_ in groups if groups[g_]["role"] not in ("payload", "service")),
     "R103 heavy single-contingency estimate of the inspected configuration", "inspectionHeavyKg",
     pin("artifacts/resume-20261121/mass-01/mass-study.json"), "Heavy estimate at or under 115 lb"),
    ("weak-pack-voltage", "preliminary-design", "V", {"range": [9.0, 13.0]}, ["pan", "steer sweeps", "electronics", "battery"],
     "Every cycle stage on the weak 11.8 V / 32 mOhm pack at the conservative simultaneous current", "weakPackMinBusV",
     pin("iterations/002/design/strategy/drive-envelope.json"), "Lumped bus stays at or above the 9 V project branch floor"),
    ("electronics-layout", "preliminary-design", "mm", {"range": [2.0, 1000.0]}, ["electronics", "wiring", "battery", "battery retention"],
     "552-r5 electronics placements, battery gauge and wiring reserves inside the r3 upper support", "electronicsMinClearanceMm",
     pin("iterations/002/design/chassis/inset-electrical-package.json"), "Every electrical reserve keeps the 2 mm package reserve"),
    ("chassis-interface", "preliminary-design", "revision", {"equals": "002-connected-machining-654-r6"},
     ["structure", "module housings", "steer sweeps", "bumpers"],
     "Carried-forward iteration 002 chassis as a revisable premise of this layout, not a fixed boundary",
     "chassisInterface", pin("iterations/002/design/cad/fusion-authority.json"),
     "Retained on its evidence (six converged issues, occupied-shape closeout, seated screws). Replace it if the evaluation "
     "shows the heavy R103 estimate cannot keep its margin without structural savings (mass), if the bumper or battery "
     "packaging needs a different perimeter (17.25 mm effective rear cutback), or if a better scoring architecture needs "
     "other module stations or upper-support landings."),
    ("pan-package-geometry", "preliminary-design", "revision", {"equals": "002-pan-r3"}, ["pan", "pan cassette", "inlet", "upper support"],
     "Guarded pan r3 proposal geometry (pivot, stow 70 deg, dump -2 deg, stops, inlet) as a revisable premise",
     "panPackageRevision", pin("iterations/002/design/pan/geometry.json"),
     "Retained on the twelve-case r3 replays and the failed fixed-path comparison. Replace the stow and receive geometry if "
     "the retention review (R1 open face, R2 power-off loss at 3 g) needs a retainer, latch or deeper pocket, if the "
     "physical dock test reopens the 1.828 mm support margin, or if mass needs the P3 inlet reduction."),
    ("seated-clamp-screws", "preliminary-design", "fit", {"equals": "four 91251A542 seated in UNC-2B taps"}, ["structure"],
     "Four tapped nutbar holes and seated 1/4-20 x 1 in screws on the inset brace clamps as a revisable premise",
     "seatedClampScrews", pin("artifacts/resume-20261121/screws-place-01/native-receipt.json"),
     "Retained as seated purchased screws; preload, thread phase and service travel remain open. Replace them with the chassis "
     "if the chassis premise is replaced, or if the load and maintenance review rejects the clamp."),
    ("pan-selection", "preliminary-design", "architecture", {"equals": "guarded pan"}, ["pan", "pan cassette", "inlet"],
     "Selected scoring architecture after the matched fixed-path comparison (12 cases)", "selectedArchitecture",
     pin("artifacts/resume-20261121/fixed-path-01/comparison-02.json"),
     "Detail the guarded pan; a fixed path loses at least one adverse pipe"),
]
premises = []
for key, phase, unit, crit, ifaces, scenario, field_, source, basis in premise_specs:
    acc = write(f"{REL}/premises/{key}-accepted.json", dict(id=key, scenario=scenario, unit=unit, criterion=crit, phase=phase,
                                                             interfaces=ifaces, source=source, basis=basis))
    value = read(measure["file"])[field_]
    cur = write(f"{REL}/premises/{key}-current.json", dict(id=key, scenario=scenario, unit=unit, value=value, issue=ISSUE,
                                                           designSha256=design["sha256"], stepSha256=step["sha256"],
                                                           source=measure, valueField=field_))
    premises.append(dict(accepted=acc, current=cur))

# ------------------------------------------------------------------ presentation, load paths and carry-forward record
CATEGORY = {"structure": ("frame", "6061-T6 aluminium tube and plate (taps-02 bodies)"),
            "bumpers": ("bumper", "4 mm backer, 69.5 mm PE foam, 0.5 mm cover (project rear-facet record)"),
            "steer sweeps": ("wheel", "SDS MK4i turret: billet wheel, nitrile tread, aluminium mounts (supplier)"),
            "module housings": ("drive", "SDS MK4i housing with two NEOs and CANcoder (supplier)"),
            "upper support": ("structure", "6061-T6 tube (r3 stringer extensions and inlet truss)"),
            "pan cassette": ("mechanism", "6061-T6 cradles and plates, REV bearings, UltraPlanetary and NEO 550 (r3)"),
            "pan": ("receiver", "5052-H32 sheet with 6061-T6 stringers and plates (r3)"),
            "inlet": ("guide", "5052-H32 sheet and 6061-T6 tube (r3 fixed inlet)"),
            "wiring": ("electrical", "Copper conductor and insulation reserve"),
            "battery retention": ("plate", "Aluminium tray and hangers (planning reserve)"),
            "battery": ("battery", "Sealed lead-acid FRC battery with leads (excluded by R103)"),
            "pipe": ("game-piece", "PVC CORAL pipe (field element)"),
            "breaker service hand": ("reserved-space", "Clearance gauge: no robot material"),
            "plug service hand": ("reserved-space", "Clearance gauge: no robot material")}
ELEC = {"Power distribution hub": ("power", "REV PDH (supplier)"),
        "Main 120 A breaker": ("power", "Eaton CB185 120 A breaker (supplier)"),
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
housings = [n for n in by_group["module housings"] if "part" not in n]
ctrl = [n for n in by_group["electronics"] if n.startswith(("Port controller", "Starboard controller"))]
required = {
    "wheels": sweeps, "drive actuators": housings, "steer actuators": housings,
    "control computer": ["roboRIO 2"], "power distribution": ["Power distribution hub"],
    "main disconnect": ["Main 120 A breaker"], "battery supply": ["Dressed battery", "Chassis battery plug and bracket"],
    "comms": ["VH-109 radio"],
    "pan pivot": ["Pan cassette", "Guarded pan"], "pan drive": ["Pan cassette", "Pan controller (SPARK MAX)"],
    "pan stops": ["Pan cassette"], "pan encoder": [], "pan clutch": ["Pan cassette"],
}
load_members = {
    "drive": housings + [n for n in ctrl if n.split()[2] in ("1", "3")],
    "steer": housings + sweeps + [n for n in ctrl if n.split()[2] in ("2", "4")],
    "pan": ["Pan cassette", "Guarded pan", "Pan controller (SPARK MAX)"],
    "controls": ["roboRIO 2", "VH-109 radio", "Power distribution hub"],
}
load_paths = write(f"{REL}/load-paths.json", dict(
    schema="red2/preliminary-load-paths@1", issue=ISSUE, step=step,
    paths=[
        dict(load="Guarded pan with seated pipe", kind="support",
             chain=["Guarded pan socket post stacks (Y +/-164..188)", "1/2 in hex drive shaft (port) and headed idler stub (starboard)",
                    "REV-21-1915 flanged hex bearings in both carriers (Y +/-195)", "bearing carriers and gearbox plate",
                    "carrier landing clips 110 and 250", "upper cradles X110 and X250", "four 3/4 in angle cradle end clips",
                    "upper pan stringers +/-1", "pan truss members 0-5 (plugged butt nodes M9)", "truss landings and captive nutbars",
                    "pan crossmembers 110/250", "inboard sills and structural perimeter"],
             reserves=["Guarded pan", "Pan cassette", "Chassis pan trusses and stringers", "Chassis sills, decks and crossmembers", "Chassis perimeter"],
             absent=["Shaft and stub sections inside the 2.1 mm journal partitions are not separate solids",
                     "Plugged butt node capacity (M9) and the rivet-nut pull-out rating (1.5 kN, unsourced)"]),
        dict(load="Guarded pan", kind="drive",
             chain=["NEO 550 case on the UltraPlanetary motor plate", "UltraPlanetary 54.83:1 stages", "required slip clutch (16.5-18 N m)",
                    "UltraPlanetary output face on the port gearbox plate (Y 216)", "1/2 in hex through the port bearing",
                    "port socket post hex cut", "pan cheek"],
             reserves=["Pan cassette", "Guarded pan"],
             provisional=["Slip clutch: required but not designed or placed; reserved only inside the gearbox envelope",
                          "Absolute encoder for the -2 deg soft limit: not designed or placed",
                          "Reverse-limit cam and roller-lever switch: r3 sizing only; cam-to-carrier 1.85 mm, bearing flange unmodelled",
                          "UltraPlanetary stage stack and NEO 550 lengths unsourced (envelope ends at Y 350)"]),
        dict(load="Guarded pan", kind="stops",
             chain=["cheek stop lug stacks -> 6 mm stow pads and shims -> stow stop brackets on the X110 cradle",
                    "lower-flange strike blocks -> 18 mm soft-stop pads -> caps, posts and beams -> keeper plates under the X250 cradle",
                    "closing torsion spring: driver collar on the idler stub, fixed leg on the starboard carrier shoulder screw"],
             reserves=["Pan cassette"], absent=["Bench pad stiffness and loss (pan README open item 4)"]),
        dict(load="CORAL pipe", kind="support",
             chain=["pipe on the floor and slotted rear lip (partitioned 2.1 mm)", "the Guarded pan support path"],
             reserves=["CORAL pipe", "Guarded pan"], absent=["No retainer for the carry hop under peer events (study-06)"]),
        dict(load="Each SDS module wheel", kind="support",
             chain=["wheel and nitrile tread", "wheel mounts A/B and shoulder axle", "A035XPO turret bearing", "module plates",
                    "six 10-32 x 2.5 in frame screws through the 50.8 mm socket rail", "socket rails and cassettes",
                    "frame supports and sleeves", "structural perimeter"],
             reserves=["Module steer sweeps", "Module housings", "Chassis cassettes and socket rails", "Chassis perimeter"]),
        dict(load="Each SDS module wheel", kind="drive",
             chain=["drive NEO -> 14T/50T stage -> intermediate shaft -> bevel 1545 -> wheel (supplier L2 6.746:1)",
                    "steer NEO -> 14T/50T -> steering belt 320-5M-15 -> base pulley -> turret"],
             reserves=["Module housings", "Module steer sweeps"],
             absent=["Gear mesh and belt wrap are supplier interfaces inside the housing partition"]),
        dict(load="Dressed battery", kind="support",
             chain=["battery case on the tray", "end keepers", "four hangers", "inboard sill undersides"],
             reserves=["Dressed battery", "Battery tray and retention", "Chassis sills, decks and crossmembers"],
             absent=["Tray, hangers, keepers and top hold-down are undesigned (planning reserve)"]),
        dict(load="Electronics", kind="support", chain=["devices on standoffs", "equipment sheets", "deck keys, pedestals and tabs"],
             reserves=by_group["electronics"], absent=["Device brackets and standoffs"]),
        dict(load="Bumper ring", kind="support", chain=["bumper backer", "permanent bumper mounts", "structural perimeter"],
             reserves=["Bumper ring", "Chassis perimeter"], absent=["The 002 bumper mounts are not designed (R103 counts them)"]),
    ]))
carry = write(f"{REL}/chassis-carry-forward.json", dict(
    schema="red2/selected-interface@1", issue=ISSUE, decision="revisable premise",
    ownerGuidance="The preliminary design must find what will actually work and inform the Fusion design; existing Fusion CAD "
                  "does not constrain it. Each carried item below is a premise with its evidence and its replacement trigger.",
    premises=["chassis-interface", "pan-package-geometry", "seated-clamp-screws"],
    replaceIf=dict(
        chassis="A mass evaluation that cannot hold the R103 margin without structural savings, a bumper or battery packaging result "
                "that needs a different perimeter or corner, or a scoring architecture that needs other module stations or landings",
        panPackage="A retention result needing a retainer, latch or different stow and receive geometry; a physical dock test "
                   "outside the 1.828 mm support margin; or a mass result needing the inlet or cassette lightened",
        tapsAndScrews="Replacement of the chassis, or a load and maintenance review that rejects the clamp"),
    interface="Iteration 002 chassis: 146 fabricated bodies, four SDS MK4i L2 NEO modules and four seated 91251A542 clamp screws",
    reason="The layout carries the existing 002 chassis forward as a revisable premise rather than starting greenfield. Six issued chassis "
           "revisions converged on it (cad/issue-001 to issue-006, registered master 002-connected-machining-654-r6); the "
           "occupied-shape replay closed with all eight retained bodies in the same occupied form; and the four purchased "
           "screws are seated in the tapped nutbars on the selected STEP.",
    evidence=[pin("iterations/002/design/cad/fusion-authority.json"), pin("iterations/002/design/cad/issue-006/lineage.json"),
              pin("artifacts/resume-20261121/seven-retained-8-closeout.json"),
              pin("artifacts/resume-20261121/screws-place-01/native-receipt.json"), pin(CHASSIS)],
    reopen=["The bumper outline: the lower cassette corners set a 17.25 mm rear cutback, not the recorded 57.8 mm chamfer",
            "Any change at the four module stations, the upper pan stringers or the pan-truss landings",
            "Perimeter strike stress (149-241 MPa, open) or module-screw loads beyond their review"]))
# ------------------------------------------------------------------ purpose and rationale for every placed ID
def explain(name, group):
    k = name.split()[1] if name.startswith("Module ") else None
    if name.startswith(("Chassis perimeter", "Chassis cassettes", "Chassis sills", "Chassis pan trusses")):
        base = name.split(" part ")[0]
        what = {"Chassis perimeter": ("Closes the frame perimeter that carries the bumpers and reacts strikes into the wheel stations.",
                                      "The bumper ring and the frame-perimeter rules bound the robot; strike loads must reach the module stations."),
                "Chassis cassettes and socket rails": ("Holds the four module cassettes; the module plates clamp the 50.8 mm socket rails.",
                                                       "Four-module swerve needs a rigid, serviceable seat at each corner."),
                "Chassis sills, decks and crossmembers": ("Ties front to rear through the sills, spans the pan crossmembers, seats the clamp screws and carries the two equipment sheets.",
                                                          "The two-sill architecture hangs the battery low and central, reaches the pan loads into both sills and leaves the battery lift gap between the decks."),
                "Chassis pan trusses and stringers": ("Raises the upper pan stringers to Z 470-490 for the cradles and stringer extensions.",
                                                      "The pan pivot at Z 580 needs support stringers on both sides, clear of the electronics below.")}[base]
        if base != name:
            return (what[0] + " (separated piece after partition)", what[1])
        return what
    if group == "steer sweeps":
        return (f"Occupies the module {k} wheel and turret through any steering angle and puts the tread on the floor at Z = 0.",
                "Four-module swerve is the project drivetrain; the full 360 deg steer sweep must stay clear of the frame, "
                "bumpers, leads and battery while the robot re-azimuths during the loaded carry.")
    if group == "module housings":
        if "part" in name:
            return (f"Bounds a separated piece of the module {k} housing (CANcoder cap, belt run or gear stage) left after partition.",
                    "The fixed supplier housing must be reserved wherever it is not inside the steer sweep or the chassis reserve.")
        return (f"Holds the module {k} drive and steer NEOs, gears, belt, CANcoder and plates on the socket rail.",
                "The SDS MK4i L2 corner is the selected purchased module; its motors sit inboard and govern the lead and "
                "crossmember space under the equipment sheet.")
    if group == "upper support":
        return ("Extends the upper pan stringers rearward and lands the inlet truss on the perimeter.",
                "The fixed inlet sits behind the pan pivot and above the battery path, so it needs its own stringer extensions "
                "and truss legs reaching the frame.")
    if name == "Pan cassette":
        return ("Carries the pan bearings, drive gearbox and NEO 550, both stops, the closing spring and the inhibit cam and switch, "
                "and lifts out as one unit for service.",
                "The guarded pan needs a removable support on the X110 and X250 cradles so its drive, stops and journals can be "
                "serviced without disturbing the chassis.")
    if name == "Guarded pan":
        return ("Receives the station pipe at 70 deg stow, carries it and tips 72 deg to release it into the L1 trough.",
                "The matched fixed-path comparison lost an adverse pipe in every variant, so the role relies on a powered rotating "
                "pan with positive stops (pan-selection premise).")
    if name.startswith("Inlet guides"):
        return ("Guides the station pipe laterally and supports the chute from the stringer extensions.",
                "The pipe arrives at about 2.5 m/s from the station chute; side guides keep its 301.6 mm length square to the "
                "pan opening.")
    if name.startswith("Inlet chute crossmember"):
        return ("Supports the feed panels at their joint.", "The chute panels are thin sheet and need a cross support under the "
                "rolling pipe at the panel break.")
    if name.startswith("Inlet chute panels"):
        return ("Forms the rolling surface that brings the station pipe down into the stowed pan.",
                "A passive chute lets the pan stay at stow during receive, so the receive needs no extra actuator.")
    if name == "Bumper ring":
        return ("Protects the frame perimeter in contact with field elements and robots.",
                "Bumpers are required on the frame perimeter; the ring follows the actual frame plan hull so its backing "
                "clears the lower cassette corners.")
    if name == "Power distribution hub":
        return ("Distributes battery power to the nine motor controllers and the control system through branch breakers.",
                "Every motor and controller load in the voltage budget draws through it.")
    if name == "roboRIO 2":
        return ("Runs the robot program, the pan limits and the drive control.", "A control computer is required to operate the robot.")
    if name == "VH-109 radio":
        return ("Links the robot to the driver station and field.", "Required robot communications.")
    if name == "Main 120 A breaker":
        return ("Disconnects and protects the main battery circuit.", "Required main disconnect; its reset button needs the hand path "
                "checked in service-breaker-reset.")
    if "controller" in name and name.startswith(("Port", "Starboard")):
        which = "drive" if name.split()[2] in ("1", "3") else "steer"
        return (f"Commands one module {which} NEO.", "Eight SPARK MAX controllers run the four drive and four steer motors.")
    if name.startswith("Pan controller"):
        return ("Commands the pan NEO 550 under its 30 A limit with the reverse-limit inhibit input.",
                "The opening-only drive inhibit at -3.5 deg is wired to this controller, independent of the roboRIO.")
    if name.startswith("Chassis battery plug"):
        return ("Mates the battery SB50 to the robot-side main leads.", "The battery must disconnect at a plug the plug-grip hand can reach.")
    if name == "Main power loom":
        return ("Reserves the main lead route from the SB50 to the breaker and PDH.",
                "The main leads must avoid the battery lift, the plug grip and the inlet truss legs.")
    if name.startswith("Branch and CAN trunk"):
        return ("Reserves the branch power and CAN route from the PDH to the controllers.",
                "Branch and CAN wiring must cross the centre gap between the equipment sheets above the pan crossmembers.")
    if name.endswith("lead reserve"):
        return (f"Reserves the module {k} motor and CANcoder leads between the module top and the equipment sheet.",
                "Module leads must leave the housing under the sheet without entering the steer sweep or the crossmembers.")
    if name == "Battery tray and retention":
        return ("Supports and retains the battery case below the sills.",
                "The battery sits low and central for CG and must be retained against strikes yet lift out through the centre gap.")
    if name == "Dressed battery":
        return ("Supplies the robot; its case, leads and battery-side SB50 move together in service.",
                "The pack is the voltage source of the budget and its lift and rearward carry set the service path.")
    if name == "Breaker reset hand":
        return ("Clearance gauge for a hand reaching the main breaker reset from above.",
                "Resetting the breaker between matches must not require removing the stringer extension or the inlet.")
    if name == "Plug grip hand":
        return ("Clearance gauge for a hand gripping the SB50 pair to disconnect the battery.",
                "The battery swap begins with this disconnect; the gauge keeps the space above the plugs free of leads.")
    if name == "CORAL pipe":
        return ("The finite game piece followed from the station chute through the seat, tip and release.",
                "The role scores two station pipes in the L1 trough; every receive, carry and release clearance is checked "
                "against the actual pipe length and diameter.")
    raise KeyError(name)


for b in blocks["blocks"]:
    purpose, rationale = explain(b["name"], b["group"])
    components[b["name"]].update(purpose=purpose, rationale=rationale)

# ------------------------------------------------------------------ narrative pre-read and pan retention review
narrative = dict(
    source=pin("kickoff/selected-role.md"),
    read="Read end to end on 2026-11-21: the opening LEAVE, two TELEOP station-to-L1 cycles in sectors A and B, custody, "
         "trough release and support, park, the endgame choice and the empty-start fallback. commissioning.md restates "
         "the two-pipe objective for iteration 002.",
    objectives=[
        dict(objective="AUTO: leave the starting line with the whole occupied robot (3 points)", disposition="out-of-scope",
             basis="A field-route objective for the empty robot; this package reserves the stowed starting configuration "
                   "(940.5 mm tall, perimeter 2919.8 mm) but has no field fixture. Replay the opening-line study with the 17.25 mm "
                   "rear cutback in the field rehearsal."),
        dict(objective="TELEOP: take one station-issued CORAL from the red station (front approach)", disposition="represented",
             basis="Cases receive-chute, receive-over-edge and receive-seat; premises station-receive-face, station-receive-time "
                   "and station-arrival-speed."),
        dict(objective="Carry it to the red reef while crossing peer routes", disposition="represented",
             basis="Case loaded-carry (seated pipe in the pan hold band; steer covered by the static revolved envelopes); weak-pack voltage premise; peer traffic is field-side."),
        dict(objective="Sustain a deliverable load: keep custody when accelerating, braking or contacted, and detect displacement or loss",
             disposition="open",
             basis="retention-review.json: no positive restraint on the open face at stow; powered events lift the pipe 24-114 ms "
                   "(study-06); unpowered holding loses it at 3 g opening contact (study-05). No seated-pipe sensor or camera "
                   "view is placed in this layout."),
        dict(objective="Release into the intended L1 trough sector A, clear the wall and leave the pipe supported after withdrawal",
             disposition="open",
             basis="Cases tip-to-dump, release and reset-to-stow carry the pipe out past the pan edge, and the dock-support-margin "
                   "premise holds the replay support; the reef wall and trough fixture are not placed, so whole-robot wall "
                   "clearance at the 12 mm stand-off is not checked in this package."),
        dict(objective="Return for a second station CORAL and score it in sector B with the first pipe as an obstacle",
             disposition="represented",
             basis="The same ordered cycle repeats from reset-to-stow; sector B timing and the first-pipe obstacle are covered by "
                   "the twelve-case replays behind the premises."),
        dict(objective="PARK: reserve the south lane, stow, reach the zone and stop", disposition="represented",
             basis="Case park (withdraw 300 mm from the scored pipe, pan stowed); the lane reservation is field-side."),
        dict(objective="Endgame alternatives: shallow or deep CAGE", disposition="out-of-scope",
             basis="The selected role parks; the occupied receiver has no demonstrated climb route (selected-role.md)."),
        dict(objective="Fallback: empty-start leave-and-park for 5 points", disposition="represented",
             basis="The stowed starting configuration and the park case are the same robot with no pipe aboard."),
        dict(objective="Partner and station dependence: Red 1 station change and station depletion", disposition="out-of-scope",
             basis="An alliance-chain and field-supply comparison owned by conceptual design; it changes no package volume."),
    ])

# Gravity in the pan frame. u runs along the floor toward the front edge, w is the floor normal into the pocket.
def frame(theta):
    t = math.radians(theta)
    return (math.cos(t), math.sin(t)), (-math.sin(t), math.cos(t))
supports = [(70.0, "stow: station receive and loaded carry", ["rear lip (-u)", "floor (-w)", "cheeks (+/-Y)"]),
            (34.0, "mid-tip", ["rear lip (-u)", "floor (-w)", "cheeks (+/-Y)"]),
            (0.0, "lip unloads: gravity along the floor reaches zero", ["floor (-w)", "cheeks (+/-Y)", "friction only along u"]),
            (-2.0, "commanded dump: release toward the open front edge", ["floor (-w)", "cheeks (+/-Y)"]),
            (-5.0, "soft stop: unpowered rest after an opening event", ["floor (-w)", "cheeks (+/-Y)"])]
rows = []
for th, label, contacts in supports:
    u, w = frame(th)
    gu, gw = -u[1], -w[1]            # components of (0, -1) in g
    rows.append(dict(thetaDeg=th, stage=label, gravityAlongFloorG=gu, gravityIntoFloorG=-gw, pushBack=contacts,
                     exitDirection=("toward the lip; retained" if gu < -1e-9 else "toward the open front edge" if gu > 1e-9 else "none")))
u70, w70 = frame(70.0)
liftoff = 9.80665 * w70[1] / (-w70[0])     # forward chassis acceleration that lifts the pipe off the floor at stow
sweeps = {}
pipe_s = solids_["CORAL pipe"]
from layout_occ import moved as _moved
for label, dvec in (("open face (+w, rear and up)", (w70[0], 0.0, w70[1])), ("up the floor (+u)", (u70[0], 0.0, u70[1]))):
    first = None
    for travel in range(0, 401, 2):
        shape = _moved(pipe_s, tuple(travel * c for c in dvec))
        b2 = aabb(shape)
        hits = [(distance(shape, solids_[n]), n) for n in solids_ if n != "CORAL pipe" and box_gap(b2, bb[n]) < 1.0]
        hits = [h for h in hits if h[0] <= 1e-6]
        if hits:
            first = dict(travelMm=travel, obstruction=sorted(h[1] for h in hits))
            break
    sweeps[label] = first or dict(travelMm=None, obstruction=[], note="free beyond 400 mm: the pipe leaves the pan")
retention = write(f"{REL}/retention-review.json", dict(
    schema="red2/tipping-retention-review@1", issue=ISSUE, step=step,
    method="Static gravity projection into the pan frame at each change of support, rigid sweep of the whole seated pipe "
           "from its stow seat toward each opening against every other reserve, and the existing free-piece replays for "
           "dynamic capture, power loss and peer contact. Separate from the prescribed (carriedWith) animation.",
    supportChanges=rows,
    stowLiftoffForwardAccelMps2=liftoff,
    sweepsFromStowSeat=sweeps,
    restraints=["Rear lip (slotted, |Y| 96..161) and floor carry the pipe at stow; cheeks bound it in Y with 10 mm side play",
                "Powered stow hold at 30 A on the 5420 N m/rad stow stop with the 1.5 N m closing spring",
                "Unpowered: the closing spring holds station impact and 2 g opening contact (pan-stop-study-05 H5-o2g50 kept)",
                f"Open face: a rigid sweep of the seated pipe along +w meets {', '.join(sweeps['open face (+w, rear and up)']['obstruction'])} "
                f"after {sweeps['open face (+w, rear and up)']['travelMm']} mm, so the chute stops a rearward escape; the path up the floor (+u) is free"],
    evidence=dict(unpowered=pin("artifacts/resume-20261121/pan-stop-01/pan-stop-study-05.json"), powered=ev06),
    powerOffEscapeControl=dict(
        case="H5-o3g50-spring-unpowered (pan-stop-study-05)",
        result="lost: a 3 g, 50 ms opening contact with power removed opens the pan and the pipe leaves before scoring",
        worstLoadedAngleDeg=-5.0,
        note="With the pan open to its -5 deg soft stop, gravity points 0.087 g along the floor through the open front edge"),
    findings=[
        dict(id="R1", status="open",
             text=f"No positive restraint on the open face at stow. A forward chassis acceleration above {liftoff:.2f} m/s2 "
                  "lifts the pipe off the floor; study-06 shows it lift and re-seat for 24-114 ms in every powered event "
                  "(3 g to the front strike). Custody was kept powered, but the continuous-support test is interrupted."),
        dict(id="R2", status="open",
             text="With power removed the pipe is retained only to 2 g opening contact; 3 g, 4 g, 5 g and the front strike lose it "
                  "(study-05 holds). Add an article retainer or a latched stow and retest, or accept powered custody only."),
        dict(id="R3", status="open", text="No measured PVC-on-pan friction or seated-pipe sensing; release angle and slip rely on the replay model."),
    ]))

corrections = write(f"{REL}/modelling-corrections.json", dict(
    schema="red2/modelling-corrections@1", issue=ISSUE,
    corrections=[
        dict(id="MC1", item="Module steer sweeps",
             was="A wheel cylinder at the parked azimuth plus coaxial steer cylinders, labelled a full 360 deg sweep and rotated 90 deg in loaded-carry and park",
             now="A true solid of revolution of the turret radial profile about each steer axis, static in every case",
             reason="The rotated blocks were not the envelope their label claimed, and rotating a near-axisymmetric block against housings "
                    "4 mm away forced deep exact refinement (25 CPU-minutes in loaded-carry) without adding occupied volume"),
        dict(id="MC2", item="CORAL pipe installed pose",
             was="The STEP held the pipe at the station exit while the motion study assumed it at the stow seat",
             now="The STEP holds the pipe at its stow seat; receive cases offset it to the station exit",
             reason="carriedWith requires the payload and carrier to share the same initial pose; the first fail-fast run caught the double offset (receive-chute, 7695 mm3 with feed panel 1)"),
        dict(id="MC3", item="Chassis reserve",
             was="One fused block of 150 per-body boxes spanning the robot",
             now="Perimeter, lower cassettes, pan trusses and stringers, and sills/decks/crossmembers as separate partitioned reserves",
             reason="Box culling could not clear any moving pair against a whole-robot bounding box; same per-body boxes and allowances"),
        dict(id="MC4", item="Retention sweep", was="Computed from the station-exit pipe pose", now="Computed from the stow seat",
             reason="Follows MC2"),
    ]))
presentation = dict(components=components, required=required, loadMembers=load_members,
                    provisional={"pan encoder": "Not designed or placed: absolute encoder for the -2 deg soft limit",
                                 "pan clutch": "Required slip clutch not designed; only the gearbox envelope reserves it",
                                 "pan cam and switch": "r3 sizing only, inside the cassette cam-sweep reserve"},
                    loadPaths=load_paths, selectedInterface=carry, retention=retention, corrections=corrections)

plan = dict(schema="agent-frc/preliminary-layout@1", issue=ISSUE, design=design, step=step, motion=motion, datum=datum,
            groups=groups, mounts=mounts, stages=stages, missingInputs=[],
            control=dict(case="tip-to-dump", displaced="Breaker reset hand", moving=pan, offsetMm=[560.0, -274.0, 120.0],
                         basis="A service hand left over the pan arc between the clear stow and dump endpoints"),
            cycle=cycle, view=f"{REL}/package-motion.html", budgets=budgets, premises=premises,
            presentation=presentation, narrativeReview=narrative)
write(f"{REL}/layout.json", plan)
print("weak min", round(weak_min, 4), "heavy", round(total[1], 3), "running", [round(x, 3) for x in running],
      "elec", round(elec_min[0], 3), elec_min[1], "ready", round(ready_after_feed, 4), "arrival", arrival, selected)
