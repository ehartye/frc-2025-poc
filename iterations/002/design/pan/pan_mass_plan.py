"""Whole-robot R103 mass plan for pan revision r3 (offline; writes mass-plan.json).

r3 re-bases the heavy estimate on the accepted taps-02 issue, not on the 552-r5 allocation. Every line is
itemised: the catalog modules, the 146 measured taps-02 chassis bodies, the chassis allowances carried from
inset-connected-mass.json, the bottom-up pan (measured fabricated volumes, itemised purchased articles,
fasteners and rivet nuts), the previous unweighed-parts allowance and the itemised 2 kg high case.
Lightening proposals are applied only where they are in the CAD of this package and their structure is checked.

Run from the project root after pan_geometry.py and the roster:
  python -I iterations/002/design/pan/pan_mass_plan.py PLUGIN TAPS02_STEP
"""
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
LB = 0.45359237
R103 = 115 * LB
NAMED = "iterations/002/design/chassis/inset-upper-named-mass.json"

# Purchased-article masses (kg). "sourced" rows carry the supplier catalog mass registered in chassis/inset-upper-named-mass.json
# (URL given); "estimated" rows are engineering estimates with no supplier or scale value yet and must be weighed at procurement.
PURCHASED = [
    ("REV-21-1651 NEO 550 with leads", 0.142, "sourced"), ("REV-41-1601 3:1 stage", 0.0355, "sourced"), ("REV-41-1602 4:1 stage", 0.034, "sourced"),
    ("REV-41-1603 5:1 stage", 0.036, "sourced"), ("REV-41-1607 550 input plate", 0.0335, "sourced"), ("REV-41-1615 V2 output cartridge", 0.0665, "sourced"),
    ("REV-41-1608 13T pinion", 0.005, "estimated"), ("REV-41-1620 1/2 in hex output adapter", 0.025, "estimated"), ("9 x M3x8 drive screws", 0.009, "estimated"),
    ("2 x REV-21-1915 flanged 1/2 in hex bearing", 0.040, "estimated"), ("2 x 6655K17 thrust stack, stationary halves", 0.026, "estimated"),
    ("port 1/2 in hex drive shaft", 0.020, "estimated"), ("2 stow pads + 2 soft-stop pads (r3: 18 mm soft-stop pads)", 0.026, "estimated"),
    ("closing spring stationary half + #10 shoulder screw", 0.012, "estimated"), ("pan position encoder, limit input and dressed lead", 0.060, "estimated"),
    # r3 additions
    ("1/2 in hex friction slip clutch, 16.5-18 N m (required), with gearbox stand-off", 0.200, "estimated"),
    ("reverse-limit roller switch, 2 x #4-40 screws and lead", 0.012, "estimated"),
]
FASTENER_KG = {"#10-32 screw": 0.0055, "rivet": 0.0008, "rivet nut": 0.0025}
HIGH_CASE = [("additional joints (inspection-mass-552 2 kg high case)", 1.25), ("upper (same high case)", 0.50), ("loom (same high case)", 0.25)]


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    plugin, step = Path(sys.argv[1]), Path(sys.argv[2])
    sys.path.insert(0, str(plugin / "scripts"))
    from showcase_contacts import load_solids
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    est = json.loads((ROOT / "iterations/002/design/chassis/inspection-mass-552.json").read_bytes())
    icm = json.loads((ROOT / "iterations/002/design/chassis/inset-connected-mass.json").read_bytes())
    named = json.loads((ROOT / NAMED).read_bytes())
    geo = json.loads((HERE / "geometry.json").read_bytes())
    roster = json.loads((HERE / "target-roster.json").read_bytes())
    urls = {a["id"]: a["source"] for a in named["currentCatalogArticles"]}
    ids = {}
    solids, _ = load_solids(step, ids)
    chassis_kg, n, modules = 0.0, 0, 0
    for k, s in solids.items():
        if ids[k]["occurrencePath"][1].startswith("Matched SDS"):
            modules += 1
            continue
        g = GProp_GProps(); BRepGProp.VolumeProperties_s(s, g); chassis_kg += g.Mass() * 2.70e-6; n += 1
    coh = est["currentCohortsKg"]
    stock_552 = sum(v for grp in icm["allocation"].values() for v in grp["stockGroups"].values())
    screws_kg = sum(n_ * icm["sourceScrewKg"][k] for grp in icm["allocation"].values() for k, n_ in grp["screws"].items())

    parts = geo["parts"]; rec = geo["reconciliation"]
    moving_fab = sum(p["massKg"] for p in parts if p["moving"])
    moving_hw = sum(h["grams"] for h in rec["offAxisHardware"]) / 1000
    moving = moving_fab + moving_hw + rec["onAxisAddedKg"]
    support = sum(p["massKg"] for p in parts if p["group"] == "Pan upper support")
    inlet = sum(p["massKg"] for p in parts if p["group"] == "Fixed inlet")
    purchased = sum(m for _, m, _ in PURCHASED)
    fixed_occ = [o for o in roster["occurrences"] if o["kind"] == "purchased" and not o["path"].startswith("Guarded pan")]
    rivnut = lambda o: o["article"].startswith("#10-32 flanged blind rivet nut")
    screws = sum(1 for o in fixed_occ if o["article"].startswith("#10-32") and "screw" in o["article"] and not rivnut(o))
    rivets = sum(1 for o in fixed_occ if "structural blind rivet" in o["article"])
    rivnuts = sum(1 for o in fixed_occ if rivnut(o))
    fasteners = screws * FASTENER_KG["#10-32 screw"] + rivets * FASTENER_KG["rivet"] + rivnuts * FASTENER_KG["rivet nut"]
    upper = moving + support + inlet + purchased + fasteners
    applied_inlet = [p for p in parts if p.get("lightening")]

    lines = [
        ("catalog four SDS L2 modules (inspection-mass-552 currentCohorts)", coh["catalogFourSDSModules"], "catalog"),
        (f"taps-02 measured chassis bodies ({n} non-module solids at 2700 kg/m3; includes the 0.908 kg pan trusses and the 1.504 kg equipment sheets)", chassis_kg, "measured, taps-02 STEP"),
        ("planned chassis screws (116, inset-connected-mass sourceScrewKg)", screws_kg, "measured supplier screw solids"),
        ("remaining fixed joining allowance (inset-connected-mass)", icm["remainingFixedJoiningAllowanceKg"], "allowance"),
        ("electrical budget remainder after the measured sheets (7 kg budget)", icm["electricalAllowanceAfterSheetsKg"], "allowance"),
        ("unspecified installed hardware and finish (inspection-mass-552)", coh["unspecifiedInstalledHardwareAndFinish"], "allowance"),
        ("steel axle-spacer correction and 1 g barrel allowance (inset-connected-mass)", icm["sourceCorrectionSteelKg"] + 0.001, "measured + allowance"),
        ("outstanding chassis item: taps-02 seated purchased-screw phase, 4 x #10-32 (pending in the taps-02 receipt; not in the 116)", 4 * icm["sourceScrewKg"]["91251A354"], "estimated"),
        ("r3 pan, upper support, inlet truss and drive bottom-up (upperBottomUp)", upper, "measured volumes + itemised purchases"),
        ("previous allowance for unweighed pan articles (inset-connected-mass panRemainingForUnweighedArticlesKg), kept in full", icm["panRemainingForUnweighedArticlesKg"], "allowance"),
    ] + [(f"high case: {a}", m, "allowance") for a, m in HIGH_CASE]
    heavy = sum(m for _, m, _ in lines)
    proposals = [
        dict(id="P1", area="fixed inlet", applied=bool([p for p in applied_inlet if "inlet guide" in p["id"]]), kgSaved=0.196,
             change="Inlet guides 3 mm to 2 mm 5052-H32; contact faces kept, spines and post tips moved 1 mm inboard to stay seated", review="structure.json inletLightening"),
        dict(id="P2", area="fixed inlet", applied=bool([p for p in applied_inlet if "feed panel" in p["id"]]), kgSaved=0.144,
             change="Feed panels 2.0 to 1.6 mm 5052-H32; top (contact) faces kept, crossmember tops raised to stay seated", review="structure.json inletLightening"),
        dict(id="P3", area="fixed inlet", applied=False, kgSaved=0.25, change="Inlet posts to 3/4 x 3/4 x 1/16 in tube",
             review="NOT applied: the post-to-spine attachment is not designed, so the structure cannot be checked"),
        dict(id="C2", area="electrical budget", applied=False, kgSaved=0.45, change="Pocket both equipment sheets about 30 %",
             review="NOT applied: chassis package, needs deck stiffness and device-mount checks and a chassis CAD issue"),
        dict(id="C3", area="chassis", applied=False, kgSaved=0.30, change="Pocket the six keyed pedestals outside the key, nutbar and screw lands",
             review="NOT applied: chassis package, needs the deck register and brace clamp paths rechecked"),
        dict(id="C4", area="chassis", applied=False, kgSaved=0.0, change="Do not lighten the structural perimeter (bumper cases already 149-241 MPa)", review="excluded"),
        dict(id="S1", area="upper support", applied=True, kgSaved=None, change="Lightening windows in both bearing carriers and the gearbox plate", review="in the measured support mass"),
        dict(id="S2", area="moving pan", applied=True, kgSaved=None, change="Lightening window in all three socket-post plates (r3)", review="in the measured moving mass"),
    ]
    applied_saved = sum(p["kgSaved"] for p in proposals if p["applied"] and p["kgSaved"])
    unapplied = sum(p["kgSaved"] for p in proposals if not p["applied"] and p["kgSaved"])
    margin = R103 - heavy
    overlap = sum(m for a, m, b in PURCHASED if b == "estimated" and any(w in a for w in ("pinion", "adapter", "M3x8", "1915", "6655K17", "encoder")))
    out = dict(
        schema="red2/pan-mass-plan@2", revision=geo["revision"],
        inputs={"iterations/002/design/chassis/inspection-mass-552.json": sha(ROOT / "iterations/002/design/chassis/inspection-mass-552.json"),
                "iterations/002/design/chassis/inset-connected-mass.json": sha(ROOT / "iterations/002/design/chassis/inset-connected-mass.json"),
                NAMED: sha(ROOT / NAMED),
                "iterations/002/design/pan/geometry.json": sha(HERE / "geometry.json"), "iterations/002/design/pan/target-roster.json": sha(HERE / "target-roster.json"),
                "external/taps-02/four-tapped-holes.step": sha(step), "iterations/002/design/pan/pan_mass_plan.py": sha(Path(__file__))},
        rule=dict(id="R103", limitKg=R103, basis="inspection-mass-552.json ruleBasis (2025 kickoff V0); excludes battery, bumpers and game piece"),
        basis=dict(issue="taps-02 (four-tapped-holes.f3d e426e576..., STEP 166c5567...)", replaces="r2 used inset-connected-mass heavy bare, which is on the 552-r5 allocation",
                   measuredChassisKg=chassis_kg, measuredChassisBodies=n, moduleSolidsExcluded=modules, stockGroupsOn552Kg=stock_552, taps02Minus552Kg=chassis_kg - stock_552),
        heavyEstimate=dict(lines=[dict(item=a, kg=m, basis=b) for a, m, b in lines], fixedRobotKg=heavy, R103MarginKg=margin,
                           marginPercentOfLimit=100 * margin / R103,
                           withBumpersKg=heavy + coh["removableBumpersOneColor"], R408MarginKg=135 * LB - (heavy + coh["removableBumpersOneColor"]),
                           verdict=("NEGATIVE: the heavy estimate is over R103 by %.3f kg" % -margin) if margin < 0 else
                                   ("THIN: %.3f kg is below 1 %% of the limit" % margin if margin < 0.01 * R103 else "%.3f kg margin" % margin),
                           conservatism=f"The kept 0.693 kg unweighed-article allowance overlaps {overlap:.3f} kg of the itemised estimated purchases (pinion, adapter, M3 screws, bearings, thrust stacks, encoder); both are carried"),
        highCase=[dict(item=a, kg=m) for a, m in HIGH_CASE],
        upperBottomUp=dict(movingPanKg=moving, movingFabricatedKg=moving_fab, movingJoiningHardwareKg=moving_hw, movingOnAxisKg=rec["onAxisAddedKg"],
                           upperSupportFabricatedKg=support, fixedInletFabricatedKg=inlet, purchasedKg=purchased,
                           purchased=[dict(item=a, kg=m, basis=b, source=(urls.get(a.split()[0]) if b == "sourced" else "none: estimate, weigh at procurement")) for a, m, b in PURCHASED],
                           sourcedCount=sum(1 for _, _, b in PURCHASED if b == "sourced"), estimatedCount=sum(1 for _, _, b in PURCHASED if b == "estimated"),
                           fixedFastenersKg=fasteners, counts=dict(numberTen32Screws=screws, blindRivetsFixed=rivets, rivetNutsFixed=rivnuts),
                           versusAllocationKg=upper - coh["plannedUpperAndDrive"]),
        lightening=dict(proposals=proposals, appliedKg=applied_saved, notAppliedKg=unapplied,
                        marginIfAllRemainingAppliedKg=margin + unapplied,
                        note="Applied rows are in this package's CAD and checked; the rest need their own design (P3) or a chassis issue (C2, C3) and are not credited"),
        scope="Planning estimate. Measured volumes use commissioned densities; purchased masses marked 'estimated' must be weighed. It is not a scale weight.")
    (HERE / "mass-plan.json").write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8")
    print(json.dumps(dict(chassis=chassis_kg, n=n, upper=upper, heavy=heavy, margin=margin, pct=100 * margin / R103, applied=applied_saved, unapplied=unapplied,
                          lines=[(a[:60], round(m, 4)) for a, m, _ in lines], screws=screws, rivets=rivets, rivnuts=rivnuts, moving=moving), indent=1))


if __name__ == "__main__":
    main()
