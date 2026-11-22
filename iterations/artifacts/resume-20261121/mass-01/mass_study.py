"""Offline whole-robot R103 mass study for Red 2 iteration 002 (read-only on sources; writes only to OUT).

Usage: python -I mass_study.py WORKTREE OUT
"""
import hashlib
import json
import sys
from pathlib import Path

WT = Path(sys.argv[1])
OUT = Path(sys.argv[2])
D = WT / "iterations/002/design"
LB = 0.45359237
LIMIT = 115 * LB


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def load(p):
    return json.loads(Path(p).read_bytes())


src = {
    "mass-plan": D / "pan/mass-plan.json",
    "inset-connected-mass": D / "chassis/inset-connected-mass.json",
    "inspection-mass-552": D / "chassis/inspection-mass-552.json",
    "inset-upper-named-mass": D / "chassis/inset-upper-named-mass.json",
    "inset-stock-check": D / "chassis/inset-stock-check.json",
    "inset-chassis-shop": D / "chassis/inset-chassis-shop.json",
    "inset-structure-clock": D / "chassis/inset-structure-clock.json",
    "inset-deck-register": D / "chassis/inset-deck-register.json",
    "inset-key-backing": D / "chassis/inset-key-backing.json",
    "inset-cassette-decks": D / "chassis/inset-cassette-decks.json",
    "inset-saddle-all-contacts": D / "chassis/inset-saddle-all-contacts.json",
    "inset-pan-truss": D / "chassis/inset-pan-truss.json",
    "pan-structure": D / "pan/structure.json",
    "pan-geometry": D / "pan/geometry.json",
    "inset_connected_mass.py": D / "chassis/inset_connected_mass.py",
    "pan_mass_plan.py": D / "pan/pan_mass_plan.py",
    "issue-006-saved-census": WT / "artifacts/resume-20261121/issue-006-saved-census.json",
}
pins = {k: sha(v) for k, v in src.items()}
mp = load(src["mass-plan"])
icm = load(src["inset-connected-mass"])
est = load(src["inspection-mass-552"])
stock = load(src["inset-stock-check"])
geo = load(src["pan-geometry"])
census = load(src["issue-006-saved-census"])
L = {row["item"]: row["kg"] for row in mp["heavyEstimate"]["lines"]}
ub = mp["upperBottomUp"]


def line(prefix):
    hits = [v for k, v in L.items() if k.startswith(prefix)]
    assert len(hits) == 1, prefix
    return hits[0]


modules = line("catalog four SDS")
chassis = line("taps-02 measured chassis")
screws = line("planned chassis screws")
joining = line("remaining fixed joining")
elec = line("electrical budget remainder")
hw = line("unspecified installed hardware")
spacer = line("steel axle-spacer")
taps = line("outstanding chassis item")
upper = line("r3 pan")
unweighed = line("previous allowance for unweighed")
hc_j, hc_u, hc_l = line("high case: additional"), line("high case: upper"), line("high case: loom")
assert abs(sum(L.values()) - mp["heavyEstimate"]["fixedRobotKg"]) < 1e-9

# Census check: the 654-body native issue has 146 non-module bodies and no pan, electronics, battery, bumpers or screws.
non_module = [b for b in census["bodies"] if not b["path"].startswith("Matched SDS")]
assert len(non_module) == 146

pan_fab = ub["movingFabricatedKg"] + ub["upperSupportFabricatedKg"] + ub["fixedInletFabricatedKg"]
sourced = sum(p["kg"] for p in ub["purchased"] if p["basis"] == "sourced")
estimated_p = sum(p["kg"] for p in ub["purchased"] if p["basis"] == "estimated")
pan_hw = ub["fixedFastenersKg"] + ub["movingJoiningHardwareKg"] + ub["movingOnAxisKg"]
assert abs(pan_fab + sourced + estimated_p + pan_hw - upper) < 1e-9

ledger = [
    # id, kg, class, source line, note
    ("modules", modules, "catalog", "SDS MK4i L2 NEO, 6.0 lb each published approximate; tread, motors and encoder ordered separately"),
    ("chassis-fabricated", chassis, "measured-cad", "146 taps-02 non-module solids x 2700 kg/m3, includes 1.504 kg equipment sheets, 1.178 kg pedestals and 0.908 kg pan truss"),
    ("chassis-screws", screws, "measured-supplier-solid", "116 planned McMaster screws, supplier solids at 7.86e-6 kg/mm3"),
    ("taps02-clamp-screws", taps, "estimated", "4 x brace clamp screws pending in screws-01"),
    ("axle-spacer", spacer, "measured-supplier-solid", "steel spacer correction plus 1 g"),
    ("pan-fabricated", pan_fab, "measured-cad", "r3 moving, upper support and fixed inlet fabricated volumes"),
    ("pan-purchased-sourced", sourced, "catalog", "6 REV articles with supplier masses"),
    ("pan-purchased-estimated", estimated_p, "estimated", "11 unweighed purchased articles, including the 0.200 kg required clutch"),
    ("pan-fasteners", pan_hw, "estimated", "count x unit estimate: 86 #10-32 screws, 48 rivets, 50 rivet nuts, moving joining and on-axis hardware"),
    ("joining-allowance", joining, "allowance", "residual 8.087 - 4.684 kg of the retired cross-frame joining allowance (unresolved joints, splices, backing)"),
    ("electrical-allowance", elec, "allowance", "residual 7.000 - 1.504 kg electrical budget (controllers, PDH, roboRIO, radio, breaker, cables, harness, mounting)"),
    ("hardware-finish-allowance", hw, "allowance", "unspecified installed hardware and finish"),
]
stacked = [
    ("pan-unweighed-residual", unweighed, "stacked", "8.2 kg upper allocation minus its named items. The 8.2 kg allocation is superseded by the bottom-up pan, so this is a second upper allowance."),
    ("high-case-joints", hc_j, "stacked", "third joining layer on top of the 3.403 kg joining allowance and the 1.5 kg hardware allowance"),
    ("high-case-upper", hc_u, "stacked", "upper contingency defined against the 8.2 kg allocation; stacks with the 0.693 kg residual and the estimated pan rows"),
    ("high-case-loom", hc_l, "stacked", "loom contingency; the 5.496 kg electrical remainder already names harness and branch extensions"),
]
base_kg = sum(r[1] for r in ledger)
stacked_kg = sum(r[1] for r in stacked)
assert abs(base_kg + stacked_kg - mp["heavyEstimate"]["fixedRobotKg"]) < 1e-9

# One item-specific growth layer (no blanket stacks). Percentages are project planning choices, stated openly.
growth_rate = {"catalog": 0.03, "measured-cad": 0.02, "measured-supplier-solid": 0.05, "estimated": None}
growth = []
for rid, kg, cls, _ in ledger:
    if cls == "allowance":
        continue
    if rid == "pan-purchased-estimated":
        rate = 0.50
    elif rid in ("pan-fasteners", "taps02-clamp-screws"):
        rate = 0.20
    elif rid == "pan-purchased-sourced":
        rate = 0.05
    else:
        rate = growth_rate[cls]
    growth.append(dict(item=rid, baseKg=kg, rate=rate, kg=kg * rate))
open_pan = 0.15  # unitemised pan articles named in panOutstanding (diaphragm, shim, end cap, cartridge screws, shaft retention, wear surfaces, containment)
growth.append(dict(item="pan-open-articles", baseKg=None, rate=None, kg=open_pan,
                   basis="allowance for panOutstanding articles that the bottom-up does not itemise"))
growth_kg = sum(g["kg"] for g in growth)

nominal = base_kg
honest = base_kg + growth_kg
plan_heavy = base_kg + stacked_kg

# Candidate reductions: savedKg = [nominal saving, conservative saving]
cands = [
    dict(id="A1", name="Retire the stacked contingencies and carry one item-specific growth layer", kind="accounting",
         savedKg=[stacked_kg - growth_kg, stacked_kg - growth_kg], structureRisk="none", roleRisk="none",
         shopEffort="none", native=False, replay=False,
         note="Removes 0.693 + 2.000 kg of second and third layers and adds %.3f kg of item growth. Keep the 2 kg as an adverse field-mass sensitivity in the drive simulation if wanted; it is not an R103 line." % growth_kg),
    dict(id="C2", name="Pocket both equipment sheets about 30 % outside the key openings, device lands and screw rows", kind="physical",
         savedKg=[0.45, 0.30], structureRisk="medium", roleRisk="low", shopEffort="low (router/waterjet program)", native=True, replay=False,
         note="Sheets are in the deck-key load path: worst flat bearing 109.5 MPa of 120 at the (110, -95) key; deck-keyed ring cases 111-112 MPa. Pockets must keep a solid ligament around every key and the shear path between keys. Rerun the deck register and device-mount checks on the pocketed sheet."),
    dict(id="C3", name="Pocket the six keyed pedestals outside the key, nutbar and screw lands", kind="physical",
         savedKg=[0.30, 0.20], structureRisk="low-medium", roleRisk="none", shopEffort="medium (6 milled pockets)", native=True, replay=False,
         note="The two rear pedestals are 0.372 kg each and about 83 % solid. Recheck the deck register, brace clamp paths and key backing."),
    dict(id="P3", name="Fixed inlet posts to 3/4 x 3/4 x 1/16 in tube with a designed post-to-spine attachment", kind="physical",
         savedKg=[0.25, 0.18], structureRisk="low-medium", roleRisk="low", shopEffort="medium (new attachment detail)", native=True, replay=False,
         note="Post lateral load 94 N, 24.2 N m at the foot (about 41 MPa in the tube). Contact faces stay; the conservative end allows 70 g for the attachment. Fixed part, so no moving-property change."),
    dict(id="F1", name="Grade 5 titanium for the 68 1/4-20 x 5/8 deck-retaining screws (91251A539)", kind="physical",
         savedKg=[0.176, 0.15], structureRisk="low", roleRisk="none", shopEffort="low (purchase; anti-seize)", native=False, replay=False,
         note="Keys carry the deck shear; these screws retain. 5.94 g steel to about 3.35 g titanium each. Check preload and thread engagement in the tapped 6061 backing, and gall resistance."),
    dict(id="E1", name="Itemise the 5.496 kg electrical remainder from the selected articles and a routed harness", kind="accounting",
         savedKg=[0.0, 0.0], structureRisk="none", roleRisk="none", shopEffort="none", native=False, replay=False,
         note="Not credited. 8 SPARK MAX + PDH are 1.423 kg sourced; the remaining 4.07 kg covers the ninth controller, roboRIO 2, radio, breaker, SB50 robot side, main and branch cables, CAN, sensors and mounts. A routed-harness bill is likely to release mass, but nothing in the project sources it yet."),
    dict(id="J1", name="Itemise the 3.403 kg joining allowance against the modeled 146-body chassis", kind="accounting",
         savedKg=[0.0, 0.0], structureRisk="none", roleRisk="none", shopEffort="none", native=False, replay=False,
         note="Not credited. The chassis bodies and 116 screws are modeled; the residual must still name battery retention, permanent bumper mounts, truss-node gussets and rivets, nuts and washers. Until named it stays in full."),
    dict(id="X1", name="Drop the slip clutch by qualifying the 26 N m output route", kind="physical",
         savedKg=[0.20, 0.0], structureRisk="high (gearbox)", roleRisk="medium", shopEffort="low", native=False, replay=True,
         note="Not credited. Needs a measured inhibit latency of about 1.5 ms or less; the owner decision makes the clutch mandatory until then."),
    dict(id="W1", name="Same-vendor wheel or tread change on the SDS MK4i", kind="physical",
         savedKg=[0.04, 0.0], structureRisk="none", roleRisk="low", shopEffort="low", native=True, replay=True,
         note="Not credited. The billet wheel with tread is 0.55 lb; tread choices differ by tens of grams and change the loaded radius, so the drive clock would need a replay. Falcon and Kraken variants are heavier."),
    dict(id="R1", name="Thin the pan crossmembers (2 x 1 x 1/8 in) or the sills to 1/16 in wall", kind="physical",
         savedKg=[0.5, 0.0], structureRisk="high", roleRisk="medium", shopEffort="medium", native=True, replay=False,
         note="Rejected. The crossmember roof bearing is already 68.6 MPa and would roughly double past the 120 MPa working limit; the sills carry the wheel saddles (124 MPa ring peaks) and sleeves."),
    dict(id="R2", name="Lighten the perimeter, socket rails, cassettes or wheel saddles", kind="physical",
         savedKg=[0.0, 0.0], structureRisk="high", roleRisk="high", shopEffort="-", native=True, replay=False,
         note="Excluded. The perimeter bumper cases are open at 149-241 MPa, the cassette rare-wall case is 122.6 MPa and the saddle ring peak is 124.0 MPa, all over the 120 MPa working limit."),
    dict(id="R3", name="Further moving-pan lightening (skins, webs, cradles, soft-stop bar)", kind="physical",
         savedKg=[0.15, 0.0], structureRisk="high", roleRisk="high", shopEffort="medium", native=True, replay=True,
         note="Rejected. Off-post +0.07, leg B +0.14 and keeper +0.42 margins govern, and any moving change reopens the 12-case replay."),
]

package = ["A1", "C2", "C3", "P3"]
reserve = ["F1"]
byid = {c["id"]: c for c in cands}


def apply(base, ids, idx):
    return base - sum(byid[i]["savedKg"][idx] for i in ids if byid[i]["kind"] == "physical")


phys = [i for i in package if byid[i]["kind"] == "physical"]
pkg_nom = honest - sum(byid[i]["savedKg"][0] for i in phys)
pkg_cons = honest - sum(byid[i]["savedKg"][1] for i in phys)
pkg_res_nom = pkg_nom - byid["F1"]["savedKg"][0]
pkg_res_cons = pkg_cons - byid["F1"]["savedKg"][1]
plan_phys_only = plan_heavy - sum(byid[i]["savedKg"][0] for i in phys + reserve)

result = dict(
    schema="red2/mass-study@1",
    scope="Offline planning study. Nothing was edited in the worktree, nothing ran in Fusion and nothing is a scale weight.",
    inputs={k: dict(path=str(v.relative_to(WT)).replace("\\", "/"), sha256=pins[k]) for k, v in src.items()},
    rule=dict(id="R103", limitKg=LIMIT, basis=est["ruleBasis"],
              included="Robot structure and every mechanism in the inspected configuration; robot-side leads; permanently attached bumper mounts; battery tray and retention.",
              excluded="Bumpers; battery with its battery-side connector, lugs, fasteners, insulation and at most 12 in per leg; the game piece.",
              ruleArtifactGap="kickoff/rules-v0.json in the season project has no R103 entry. The basis above is the inspection-mass-552 ruleBasis (2025 V0, pp. 76/86/112). Confirm against the season's current manual revision."),
    exclusionCheck=dict(batteryInHeavy=False, bumpersInHeavy=False, gamePieceInHeavy=False,
                        note="The 52.800 kg heavy line set excludes the 6.17 kg battery, the 7.5 kg bumpers and the 0.816 kg piece. Possible small overlap: the electrical budget names 'main leads'; if those include the excluded battery-side 12 in per leg (est. under 0.1 kg) they are counted twice. Not credited."),
    ledger=dict(
        lines=[dict(id=r[0], kg=r[1], basis=r[2], note=r[3]) for r in ledger],
        stackedLayers=[dict(id=r[0], kg=r[1], basis=r[2], note=r[3]) for r in stacked],
        totalsByBasis={b: sum(r[1] for r in ledger if r[2] == b) for b in sorted({r[2] for r in ledger})},
        growth=growth, growthKg=growth_kg,
        nominalKg=nominal, nominalMarginKg=LIMIT - nominal,
        honestSingleContingencyKg=honest, honestMarginKg=LIMIT - honest, honestMarginPct=100 * (LIMIT - honest) / LIMIT,
        planHeavyKg=plan_heavy, planMarginKg=LIMIT - plan_heavy,
    ),
    accountingFindings=[
        dict(id="AF1", severity="high", kg=unweighed + hc_u,
             text="Two upper contingencies are stacked on an itemised pan: the 0.693 kg residual of the retired 8.2 kg allocation and the 0.5 kg high-case upper. The 0.693 kg was 8.2 kg minus named items and has no remaining basis once the bottom-up replaces the allocation (mass-plan already admits 0.165 kg overlap)."),
        dict(id="AF2", severity="high", kg=hc_j,
             text="Joining is covered three times: 3.403 kg residual joining allowance, 1.5 kg hardware and finish, and the 1.25 kg high case, on a chassis whose 146 bodies and 116 screws are modeled."),
        dict(id="AF3", severity="medium", kg=hc_l,
             text="The 0.25 kg loom high case stacks on the electrical remainder, which already names harness and branch extensions."),
        dict(id="AF4", severity="high", kg=0.75,
             text="C2 and C3 credits vanish if inset_connected_mass.py is rerun: electricalAllowanceAfterSheetsKg = 7 - sheets and remainingFixedJoiningAllowanceKg = 8.087 - backers (which include the pedestals). Freeze both allowances as absolute values, or itemise them, before crediting either pocket."),
        dict(id="AF5", severity="low", kg=None,
             text="README 'versusAllocation +1.074 kg' understates upper growth. The 0.908 kg chassis pan truss was also debited to the 8.2 kg allocation, so the upper is 9.274 + 0.908 = 10.182 kg against 8.2 kg (+1.982 kg)."),
        dict(id="AF6", severity="low", kg=None,
             text="commissioning.md still quotes the 552 figures (54.063 kg, 1.900 kg over R103); the r3 mass-plan heavy is 52.800 kg."),
        dict(id="AF7", severity="medium", kg=None,
             text="Battery tray and retention and permanent bumper mounts count toward R103 but appear in no line by name. They are assumed inside the joining or hardware allowances and must be named when those are itemised."),
    ],
    candidates=cands,
    package=dict(
        selected=package, reserve=reserve,
        heavyKg=[pkg_nom, pkg_cons], marginKg=[LIMIT - pkg_nom, LIMIT - pkg_cons],
        withReserveHeavyKg=[pkg_res_nom, pkg_res_cons], withReserveMarginKg=[LIMIT - pkg_res_nom, LIMIT - pkg_res_cons],
        targetMarginKg=1.5,
        meetsTargetNominal=LIMIT - pkg_nom >= 1.5, meetsTargetConservative=LIMIT - pkg_cons >= 1.5,
        meetsTargetWithReserveConservative=LIMIT - pkg_res_cons >= 1.5,
        stackedLedgerPhysicalOnlyHeavyKg=plan_phys_only, stackedLedgerPhysicalOnlyMarginKg=LIMIT - plan_phys_only,
        verdict=None,
        conditions=["Freeze the joining and electrical allowances as absolute kilograms (AF4).",
                    "Pocket C2 and C3 in native Fusion on a new issue; reopen, census, STEP and re-measure.",
                    "Rerun the deck register, key backing and brace clamp checks on the pocketed sheets and pedestals.",
                    "Design and check the P3 post-to-spine attachment and the inlet portal end clips.",
                    "Weigh the clutch, encoder and other 11 estimated pan purchases at procurement."],
    ),
    fieldMass=dict(
        replayedBandKg=[66.5495, 68.5495], executedRouteKg=67.21,
        packageFieldKg=[pkg_nom + 7.5 + 6.17 + 0.816466, nominal - sum(byid[i]["savedKg"][0] for i in phys) + 7.5 + 6.17 + 0.816466],
        note="The lighter robot falls below the replayed field-mass band; replay limited-field at the light end (traction and pipe-contact cases).",
    ),
)
nm, cm = LIMIT - pkg_nom, LIMIT - pkg_cons
rm = LIMIT - pkg_res_cons
result["package"]["verdict"] = (
    "Meets 1.5 kg at the nominal savings (%.3f kg). At the conservative savings it gives %.3f kg; adding the titanium reserve F1 gives %.3f kg. "
    "On the stacked r3 ledger no non-harmful physical package reaches 1.5 kg (best %.3f kg)." % (nm, cm, rm, LIMIT - plan_phys_only))

# assembly-accounting mapping (signed changes on the honest baseline)
REV = mp["inputs"]["external/taps-02/four-tapped-holes.step"]
data = dict(
    inventory=[r[0] for r in ledger] + ["equipment-sheets", "keyed-pedestals", "inlet-posts", "deck-screws"],
    limitKg=LIMIT,
    baseline=[nominal, honest],
    stackedBaseline=[nominal, plan_heavy],
    delta={c["id"]: [-c["savedKg"][0], -c["savedKg"][1]] for c in cands if c["kind"] == "physical" and c["savedKg"][0] > 0},
)
(OUT / "mass-study-data.json").write_text(json.dumps(data, indent=1) + "\n", encoding="utf-8")
dsha = sha(OUT / "mass-study-data.json")
swap = {"C2": ("equipment-sheets", "equipment-sheets-pocketed"), "C3": ("keyed-pedestals", "keyed-pedestals-pocketed"),
        "P3": ("inlet-posts", "inlet-posts-tube"), "F1": ("deck-screws", "deck-screws-titanium")}


def mapping(baseline_ptr, selected):
    inst = [dict(id=i, presence="modeled", disposition="make") for i in data["inventory"]]
    inst += [dict(id=new, presence="candidate", disposition="make") for _, new in swap.values()]
    return {
        "schema": "agent-frc/assembly-accounting@1", "revision": REV, "configuration": "red2-002 taps-02 + pan r3",
        "scope": "partial", "omissions": ["electronics, harness, battery retention and bumper mounts are allowances, not instances"],
        "sources": [{"id": "study", "file": "mass-study-data.json", "sha256": dsha}],
        "exports": [{"id": "ledger", "source": "study", "pointer": "/inventory"}],
        "instances": inst, "articles": [],
        "assignments": [dict(export="ledger", occurrence=i, role="installed", instance=i) for i in data["inventory"]],
        "changes": [dict(id=k, configuration="red2-002 taps-02 + pan r3", revision=REV, remove=[a], add=[b],
                         delta=dict(source="study", pointer="/delta/" + k, basis="estimated", rangeKg=data["delta"][k]))
                    for k, (a, b) in swap.items()],
        "selected": selected,
        "mass": {"unit": "kg", "configuration": "red2-002 taps-02 + pan r3", "revision": REV,
                 "baseline": dict(source="study", pointer=baseline_ptr, basis="estimated", rangeKg=data[baseline_ptr[1:]]),
                 "limitKg": LIMIT, "limitEvidence": {"source": "study", "pointer": "/limitKg"},
                 "omissions": ["no scale weight; allowances unitemised"]},
    }


(OUT / "mapping-honest-package.json").write_text(json.dumps(mapping("/baseline", ["C2", "C3", "P3"]), indent=1) + "\n", encoding="utf-8")
(OUT / "mapping-honest-reserve.json").write_text(json.dumps(mapping("/baseline", ["C2", "C3", "P3", "F1"]), indent=1) + "\n", encoding="utf-8")
(OUT / "mapping-stacked-all.json").write_text(json.dumps(mapping("/stackedBaseline", ["C2", "C3", "P3", "F1"]), indent=1) + "\n", encoding="utf-8")
(OUT / "mass-study.json").write_text(json.dumps(result, indent=1) + "\n", encoding="utf-8")
print(json.dumps(dict(nominal=nominal, honest=honest, growth=growth_kg, plan=plan_heavy, margins=[LIMIT - nominal, LIMIT - honest, LIMIT - plan_heavy],
                      pkg=[pkg_nom, pkg_cons, nm, cm], reserve=[pkg_res_nom, pkg_res_cons, LIMIT - pkg_res_nom, rm],
                      stackedPhys=[plan_phys_only, LIMIT - plan_phys_only], verdict=result["package"]["verdict"],
                      byBasis=result["ledger"]["totalsByBasis"]), indent=1))
