"""Generate target-roster.json (agent-frc/article-functions@1) for pan revision r3 from geometry.json.

Run from the project root after pan_geometry.py:
  python -I iterations/002/design/pan/pan_roster.py
Validate with the plugin: python scripts/article_functions.py iterations/002/design/pan/target-roster.json
"""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
F = {
 "support-cradle": "Bridge the upper pan stringers at X110 or X250. Carry journal, drive-reaction, stop and soft-stop loads to the stringers at the four upper pickup datums",
 "cradle-attach": "Fix one cradle end to an Upper pan stringer inboard face beside a truss node",
 "journal-carrier": "Hold a flanged 1/2 in hex bearing on the pan axis at Y=+/-195 and land on both cradle tops",
 "carrier-attach": "Fix a bearing carrier or the gearbox plate to a cradle top",
 "axial-strut": "Carry the starboard thrust (685 N guide case) from the carrier at Z=560 to the starboard stringer as axial strut force",
 "drive-mount": "React the pan gearbox housing torque at the drive datum Y=216 and land on both cradles",
 "journal": "Support the pan hex radially on the pivot axis (205, y, 580); ~446 N peak radial per side",
 "thrust": "Locate the pan axially on the starboard (idler) side only: 396 N pipe peak, 685 N guide case",
 "drive": "Supply pan torque at 54.83:1 (NEO 550; 5.23/3.62/2.90 stages) under the 30 A phase limit",
 "drive-shaft": "Transmit gearbox output torque through the port bearing into the driven hex socket with axial slip",
 "drive-fastening": "Retain the gearbox housing and output adapter (supplier screws)",
 "carry-floor": "Carry the article on the 430 x 322 mm floor through receive, carry and tip (carried-forward profile)",
 "side-guide": "Guide the article laterally (riveted sheet-and-tube cheek ladder) and present the soft-stop strike",
 "root-moment": "Carry the cheek root moment (685 N x 145 mm at the post root) from the socket posts across the pan (pivot bulkhead and 1/4 in end clips bolted through both legs)",
 "stop-strike": "Present the stow-stop striking face at the M1 contact (separate block, screwed and pinned through the cheek skin)",
 "rear-wall": "Retain the article at stow (96 mm slot)",
 "pivot-socket": "Couple the pan to the drive or idler 1/2 in hex and carry the cheek root moment (socket post)",
 "drive-protection": "Limit the gearbox output torque to 16.5-18 N m (above the 15.4 N m normal-duty cap) in every drive fault and bumper strike",
 "drive-inhibit": "Inhibit opening-direction drive output below theta = -3.5 deg; never cut closing output; never latch (controller reverse-limit input, coast)",
 "idler-shaft": "Carry the starboard side of the pan through its bearing, thrust stack and closing spring",
 "pan-joint": "Join the floor and lips to the cheeks without welding (T6/H32 tempers retained)",
 "pan-fastening": "Retain a mechanically joined pan joint (blind rivet or flat-head screw)",
 "feed-surface": "Deliver the station-fed article from the feed to the pan floor (sim feed polyline)",
 "inlet-guide": "Constrain the article laterally on the fixed inlet (94 N fixed-inlet load)",
 "inlet-structure": "Tie the fixed inlet guides and panels together",
 "inlet-support": "Carry the fixed inlet to the stringer extensions",
 "inlet-truss": "Carry the stringer extensions and inlet loads to the perimeter through the new rear truss node",
 "truss-node": "Transfer member force between an existing truss member top and the upper stringer (plugged butt node, M9)",
 "stop-bracket": "Locate the stow stop pad seat and react the +q stop load into the X110 cradle (design 2 x 296 N per pad, rear bumper strike)",
 "stop-backing": "Bond the stow pad and set its seat with the shim stack (+/-0.05 deg at stow)",
 "stop-pad": "Passive unilateral high-loss stow stop: 300 N/mm and 790 N s/m per pad at a 100 mm lever",
 "soft-stop": "Passive open-direction stop at theta=-5 deg; absorbs the F1-F3 drive faults with the inhibit late and the front bumper strike (structure.json softStopStructure)",
 "soft-stop-pad": "High-loss soft-stop pad, 18 mm, ~250 N/mm per pad at a 90 mm lever",
 "closing-preload": "Passive 1.5 N m closing torque at stow, linear to zero at theta=-2 deg",
 "spring-mount": "Locate and drive the closing torsion spring on the idler stub",
 "joint-fastening": "Retain a bolted upper-support or inlet-truss joint",
 "blind-thread": "Provide a #10-32 thread in a closed 1/16 in tube wall where no nut can be reached (blind rivet nut)",
}


def function(pid, group):
    if group == "Guarded pan":
        if "reverse-limit cam" in pid: return "drive-inhibit"
        if "floor clip" in pid or "root clip" in pid: return "pan-joint"
        if "bulkhead" in pid: return "root-moment"
        if "stop lug" in pid: return "stop-strike"
        if "floor" in pid: return "carry-floor"
        if "cheek" in pid: return "side-guide"
        if "rear lip" in pid: return "rear-wall"
        if "socket post" in pid: return "pivot-socket"
        if "stub hex" in pid: return "idler-shaft"
    if group == "Fixed inlet":
        if "feed panel" in pid: return "feed-surface"
        if "inlet guide" in pid: return "inlet-guide"
        if "post" in pid or "foot" in pid: return "inlet-support"
        return "inlet-structure"
    if pid.startswith("Upper cradle"): return "support-cradle"
    if "end clip" in pid: return "cradle-attach"
    if "bearing carrier" in pid: return "journal-carrier"
    if "landing clip" in pid: return "carrier-attach"
    if "knee strut" in pid: return "axial-strut"
    if "gearbox plate" in pid: return "drive-mount"
    if "stow stop bracket" in pid: return "stop-bracket"
    if "pad backing" in pid: return "stop-backing"
    if "soft stop" in pid: return "soft-stop"
    if "inlet truss" in pid or "stringer extension" in pid or "stringer splice" in pid: return "inlet-truss"
    if "Truss node plug" in pid: return "truss-node"
    raise KeyError(pid)


def main():
    geo = json.loads((HERE / "geometry.json").read_bytes())
    occ = []
    for p in geo["parts"]:
        occ.append(dict(path=f'{p["group"]}:1+{p["id"]}:1', article=p["id"], revision=geo["revision"],
                        configuration="fabricated " + p["material"] + (" (" + p["stock"] + ")" if p.get("stock") else ""),
                        function=function(p["id"], p["group"]), kind="fabricated", quantity=1, source=p["source"],
                        origin="retained-from-001" if p["status"].startswith("retained") else "new-002", buildInRecipe=p["build"], note=p["status"]))

    def buy(path, article, rev, conf, func, origin, note, source):
        occ.append(dict(path=path, article=article, revision=rev, configuration=conf, function=func, kind="purchased", quantity=1,
                        source=source, origin=origin, buildInRecipe=False, note=note))
    s001 = "cad/red2-split-drop/pan_drive.py (001 registration)"
    for side in ("port", "starboard"):
        buy(f"Pan upper support:1+REV-21-1915 {side} bearing:1", "REV-21-1915", "supplier STEP per 001; confirm current revision",
            "flanged 1/2 in hex bearing, 1.125 in OD", "journal", "retained-from-001", "radial only; seat datum", s001)
    for where in ("inboard", "outboard"):
        buy(f"Pan upper support:1+6655K17 starboard {where} thrust stack:1", "6655K17", "confirm", "ball thrust bearing stack for a 1/2 in shaft",
            "thrust", "retained-from-001", "axial location on the idler side only (r1)", "cad/red2-split-drop/pan-fixed-bearing-seats.json")
    for sku, conf in (("REV-21-1651", "NEO 550 motor"), ("REV-41-1607", "UltraPlanetary NEO 550 input plate"), ("REV-41-1608", "NEO 550 input pinion"),
                      ("REV-41-1603", "5:1 cartridge (68/13)"), ("REV-41-1602", "4:1 cartridge (76/21)"), ("REV-41-1601", "3:1 cartridge (84/29)"),
                      ("REV-41-1615", "V2 output cartridge"), ("REV-41-1620", "output adapter")):
        buy(f"Pan upper support:1+Pan drive:1+{sku}:1", sku, "supplier STEP per 001; confirm", conf, "drive", "retained-from-001",
            "stage order 5:1, 4:1, 3:1 from NEO 550 to V2 output", s001)
    for i in range(1, 7):
        buy(f"Pan upper support:1+Pan drive:1+M3x8 housing screw {i}:1", "M3 x 8 mm socket head screw (001 source 2800-0003-0008)", "confirm",
            "32 mm bolt circle", "drive-fastening", "retained-from-001", "", s001)
    for i in range(1, 4):
        buy(f"Pan upper support:1+Pan drive:1+M3x8 adapter screw {i}:1", "M3 x 8 mm socket head screw (001 source 2800-0003-0008)", "confirm",
            "output adapter flange", "drive-fastening", "retained-from-001", "", s001)
    buy("Guarded pan:1+Port drive hex shaft:1", "1/2 in hex shaft stock", "unselected", "cut to length after drive checked placement; slip fit in the socket",
        "drive-shaft", "retained-from-001", "", s001)
    for side in ("Port", "Starboard"):
        buy(f"Pan upper support:1+{side} stow stop pad:1", "high-loss viscoelastic pad, 20 mm dia x 6 mm (ether polyurethane or Sorbothane class, loss factor ~0.5)",
            "unselected", "bench-measure 300 N/mm and 790 N s/m per pad; restitution <= ~0.4", "stop-pad", "new-002", "seat on the backing", "pan-stop-01/pan-stop-study.json")
        buy(f"Pan upper support:1+{side} soft stop pad:1", "high-loss viscoelastic pad, 18 mm dia x 18 mm (same class; r3 taller pad)", "unselected",
            "~250 N/mm per pad; qualified to 0.31 strain (F3 with the inhibit 5 ms late, structure.json softStop)", "soft-stop-pad", "new-002", "seat on the soft stop cap", "structure.json softStop")
    spring = json.loads((HERE / "structure.json").read_bytes())["closingSpring"]["candidate"] if (HERE / "structure.json").exists() else {}
    buy("Guarded pan:1+Closing torsion spring:1", "music-wire torsion spring ASTM A228, %.1f mm wire, %.0f mm ID, %.2f body coils, 20 mm tangential legs" %
        (spring.get("wireMm", 2.5), spring.get("insideDiameterMm", 18.0), spring.get("bodyCoils", 5.0)), "unselected (made to print or catalog within +/-10 % rate)",
        "1.19 N m/rad, 1.5 N m at stow, free at theta=-2 deg", "closing-preload", "new-002", "OD-3 candidate for the procurement record", "structure.json closingSpring")
    for item, conf in (("1/2 in hex-bore shaft collar (thrust collar)", "Y -202..-208 on the idler stub"),
                       ("1/2 in hex-bore shaft collar, notched (spring driver and retainer)", "Y -223..-229"),
                       ("16 mm OD x 1/2 in hex arbor sleeve, acetal", "spring arbor Y -208..-223.4")):
        buy(f"Guarded pan:1+{item}:1", item, "unselected", conf, "spring-mount", "new-002", "", "structure.json closingSpring")
    buy("Pan upper support:1+Spring fixed-leg shoulder screw:1", "#10 shoulder screw, 17 mm shoulder", "unselected", "starboard carrier outboard face",
        "spring-mount", "new-002", "", "structure.json closingSpring")
    rivet = "3/16 in (4.8 mm) structural blind rivet, aluminum body, rated single shear >= 1.5 kN and tensile >= 2.0 kN"
    def rivets(prefix, host, n, head):
        for i in range(1, n + 1):
            buy(f"{prefix}:1+{host} rivet {i}:1", rivet + (", 100 deg countersunk into dimpled sheet (no knife-edge countersink)" if head == "csk" else ", dome head"), "unselected",
                "grip per joint; retains " + host, "pan-fastening" if prefix == "Guarded pan" else "joint-fastening", "new-002", "", "structure.json")
    SHCS = "#10-32 UNF alloy steel socket head cap screw"
    RIVNUT = "#10-32 flanged blind rivet nut for a 1/16 in 6061-T6 wall (published pull-out >= 1.5 kN required; structure.json fastenerJoints.rivetNut)"

    def screws(prefix, host, n, article):
        for i in range(1, n + 1):
            buy(f"{prefix}:1+{host} screw {i}:1", article, "unselected", "length per attachment design; retains " + host,
                "pan-fastening" if prefix == "Guarded pan" else "joint-fastening", "new-002", "", "docs/attachment-design.md (plugin)")

    def rivnuts(prefix, host, n, wall):
        for i in range(1, n + 1):
            buy(f"{prefix}:1+{host} rivet nut {i}:1", RIVNUT, "unselected", "set in the " + wall + "; blind side inaccessible",
                "blind-thread", "new-002", "", "structure.json fastenerJoints")
    parts = {p["id"]: p for p in geo["parts"]}
    for pid in parts:
        if "floor clip" in pid:
            rivets("Guarded pan", pid, 1, "dome"); rivets("Guarded pan", pid + " tab", 1, "dome")
        if "lip root clip" in pid:
            rivets("Guarded pan", pid, 4, "csk")
    for side in ("Port", "Starboard"):
        rivets("Guarded pan", f"{side} cheek skin to upper stringer", 11, "dome")
        rivets("Guarded pan", f"{side} cheek skin to lower stringers", 10, "dome")
        rivets("Guarded pan", f"{side} cheek skin to webs", 6, "dome")
        rivets("Guarded pan", f"{side} lower stringer end plugs", 2, "dome")
        rivets("Guarded pan", f"{side} cheek strike plug", 2, "dome")
        screws("Guarded pan", f"{side} lip to stop lug plate", 2, "#6-32 flat head screw into the tapped 7/8 in stop lug plate")
        screws("Guarded pan", f"{side} skin and stop lug stack", 2, "#10-32 100 deg flat head screw through the 7/8 in plate and skin lobe into the tapped 1/2 in outboard plate")
        for i in (1, 2):
            buy(f"Guarded pan:1+{side} stop lug stack dowel {i}:1", "4 mm x 30 mm hardened steel dowel pin", "unselected", "press fit in the 7/8 in plate, slip in the skin lobe and outboard plate",
                "pan-fastening", "new-002", "", "structure.json stopLugBlock")
        screws("Guarded pan", f"{side} skin to socket post", 4, "#10-32 100 deg flat head screw into the tapped post inner plate")
        screws("Guarded pan", f"{side} socket post stack", 3, "#10-32 100 deg flat head screw, flush in the outer plate, into the tapped post inner plate")
        screws("Guarded pan", f"{side} bulkhead end clip leg B", 3, SHCS + " through the clip and skin into the tapped post inner plate (9 mm engagement)")
        screws("Guarded pan", f"{side} bulkhead end clip leg A", 3, SHCS + " through the clip and bulkhead with a nylon-insert locknut on the open forward face")
        screws("Guarded pan", f"{side} post to lower stringer end plugs", 2, SHCS + " into the tapped end plug")
    buy("Pan upper support:1+Pan drive:1+Output torque limiter:1", "1/2 in hex friction slip clutch, adjustable, set 16.5-18 N m", "unselected; REQUIRED (the capacity route needs a measured inhibit latency <= 1.5 ms)",
        "between REV-41-1620 and the pan hex; the gearbox plate stands off by the clutch length; needs its own drive checked placement", "drive-protection", "new-002", "structure.json gearbox", "structure.json gearbox")
    buy("Pan upper support:1+Pan reverse-limit switch:1", "snap-action roller-lever limit switch, SPDT, subminiature", "unselected",
        "normally-closed wiring to the pan controller reverse-limit input; operating point inside the cam's 3.5 mm rise; trip-to-neutral <= 5 ms bench measured",
        "drive-inhibit", "new-002", "seat on the port carrier outboard face", "structure.json driveInhibit")
    for i in (1, 2):
        buy(f"Pan upper support:1+Pan reverse-limit switch screw {i}:1", "#4-40 socket head cap screw", "unselected", "into the tapped port carrier",
            "joint-fastening", "new-002", "", "structure.json driveInhibit")
    buy("Pan upper support:1+Pan reverse-limit switch lead:1", "two-conductor sensor lead to the controller data-port limit input, strain relieved on the carrier", "unselected",
        "dressed clear of the cam and the pan sweep", "drive-inhibit", "new-002", "", "structure.json driveInhibit")
    buy("Guarded pan:1+Port limit-cam retaining collar:1", "1/2 in hex-bore shaft collar", "unselected", "Y 203.2..209.5 outboard of the cam on the port drive hex",
        "drive-inhibit", "new-002", "", "geometry.json deferredSeats")
    for pid, p in parts.items():
        g = p["group"]
        if g == "Guarded pan":
            continue                                  # moving-pan fasteners are listed above (r2 also gave the bulkhead clips 8 extra rows here)
        if "end clip" in pid:
            screws(g, pid, 4, SHCS + " into a rivet nut"); rivnuts(g, pid, 4, "cradle side wall (2) and stringer inboard wall (2)")
        elif "landing clip" in pid:
            screws(g, pid, 1, SHCS + " into the tapped carrier or gearbox plate"); screws(g, pid + " cradle", 1, SHCS + " into a rivet nut"); rivnuts(g, pid, 1, "cradle top wall")
        elif "knee strut head clip" in pid: screws(g, pid, 3, SHCS + " into the tapped carrier plate (2) and the tapped strut plug (1)")
        elif "stow stop bracket" in pid:
            screws(g, pid, 2, SHCS + " into a rivet nut"); rivnuts(g, pid, 2, "X110 cradle rear wall")
        elif "soft stop beam" in pid:
            screws(g, pid + " aft", 1, "#10-32 100 deg flat head screw, flush in the beam top, through the keeper plate with a nylon-insert locknut")
            screws(g, pid + " forward", 1, SHCS + " through the beam and keeper plate with a nylon-insert locknut")
        elif "soft stop post plug" in pid or "inlet truss" in pid and "plug" in pid or "knee strut plug" in pid:
            screws(g, pid, 1, SHCS + " into the tapped plug"); rivets(g, pid, 2, "dome")
        elif "Truss node plug" in pid:
            screws(g, pid, 1, SHCS + " into the tapped plug"); rivets(g, pid, 4, "dome")
        elif "stringer splice" in pid:
            screws(g, pid, 4, SHCS + " into a rivet nut"); rivnuts(g, pid, 4, "stringer and extension side walls")
        elif "landing pad" in pid:
            screws(g, pid, 2, SHCS + " into a rivet nut"); rivnuts(g, pid, 2, "perimeter tube top wall")
        elif "mounting foot" in pid:
            screws(g, pid, 2, SHCS + " into a rivet nut"); rivnuts(g, pid, 2, "stringer extension top wall")
    used = {o["function"] for o in occ}
    counts = dict(total=len(occ), fabricated=sum(o["kind"] == "fabricated" for o in occ), purchased=sum(o["kind"] == "purchased" for o in occ),
                  retainedFrom001=sum(o["origin"] == "retained-from-001" for o in occ), new002=sum(o["origin"] == "new-002" for o in occ),
                  builtByRecipe=sum(o["buildInRecipe"] for o in occ),
                  rivets=sum(1 for o in occ if "structural blind rivet" in o["article"]), rivetNuts=sum(1 for o in occ if o["article"].startswith("#10-32 flanged blind rivet nut")),
                  screws=sum(1 for o in occ if "screw" in o["article"] and o["kind"] == "purchased"))
    roster = dict(schema="agent-frc/article-functions@1", authorityId="red2-two-sill-robot",
                  configuration="002 guarded pan and upper support on taps-02 chassis (pan-build r3 proposal)",
                  architecture="Single-pivot guarded gravity pan with riveted sheet-and-tube cheeks, socket posts and a pivot bulkhead (no welds), on bolted 1 in X110/X250 upper cradles over the two-sill pan truss with a rear inlet truss node; starboard axial location with knee struts; passive high-loss stow stop on separate lug plate stacks, soft stop at -5 deg bolted through a keeper under the X250 cradle, a -3.5 deg opening-direction drive inhibit (limit cam and switch), a mandatory output slip clutch, and a torsion closing spring",
                  status="Pre-CAD target roster r3 for review. It is not frozen until its SHA-256 is recorded under operatingIntent.articleFunctions in the approved commissioning plan.",
                  functions={k: v for k, v in F.items() if k in used}, occurrences=occ, counts=counts,
                  geometrySha256=hashlib.sha256((HERE / "geometry.json").read_bytes()).hexdigest())
    (HERE / "target-roster.json").write_text(json.dumps(roster, indent=1) + "\n", encoding="utf-8")
    write_fabrication_plan(geo)
    print(counts)


def write_fabrication_plan(geo):
    """frc-fabrication-plan/v1 for the r3 parts that are not a plain 2D cut or saw-and-drill: the bent lip root clips and the
    waterjet hex bores. Every other fabricated article is a 2D waterjet/router profile, a saw cut or a drilled stock length."""
    gsha = hashlib.sha256((HERE / "geometry.json").read_bytes()).hexdigest()
    ref = dict(id="red2-pan-geometry-002-pan-r3", revision=3, sha256=gsha)        # pins iterations/002/design/pan/geometry.json
    ev = [dict(id="geometry-r3", kind="design-record", description="Pan r3 offline geometry (sections, stock and bounds per part)",
               locator="iterations/002/design/pan/geometry.json", revision="002-pan-r3", sha256=gsha, recordedOn="2026-11-21", reviewDue="2026-12-21")]

    def op(i, process, machine, material, env, dev, unresolved):
        return dict(id=i, processId=process, machineClass=machine, materialId=material, setupEnvelopeMm=env, envelopeBasis="stock-and-fixture-XYZ-no-rotation",
                    allowedDeviationMm=dev, fixtureIds=None, operatorCapabilityId=None, basis="drawing-requirement", evidenceRefs=["geometry-r3"], unresolvedInputs=unresolved)
    parts = [
        dict(id="lip-root-clip", description="Port and starboard lip root clips: 1/16 in 5052-H32 strip, 65 mm long, one 95.2 deg bend (geometry.json 'Port/Starboard lip root clip')",
             designRef=ref, routeCoverage="partial",
             routes=[dict(id="shear-and-hand-brake", description="Shear the strip, bend 95.2 deg on a hand bending (finger) brake, drill and dimple for the rivets",
                          operations=[op("shear-blank", "sheet-shearing", "sheet-shear", "5052-H32-sheet-1.5875mm", [120.0, 40.0, 10.0], 0.25, []),
                                      op("bend-95.2", "press-brake-bending", "hand-bending-brake", "5052-H32-sheet-1.5875mm", [120.0, 40.0, 30.0], 0.22,
                                         ["bend radius and springback allowance for 5052-H32 at 1/16 in; check the 95.2 deg with a protractor gauge (0.22 mm = 1 deg over the 12.7 mm leg)"]),
                                      op("drill-and-dimple", "drilling", "drill-press", "5052-H32-sheet-1.5875mm", [120.0, 40.0, 30.0], 0.2, ["dimple die for the 100 deg rivet heads"])])],
             unresolvedInputs=["school hand-brake capacity for 65 mm of 1/16 in 5052-H32 is not registered"]),
        dict(id="waterjet-hex-plates", description="Socket-post plates (3/8, 3/8, 3/16 in) and the limit cam (1/8 in) with the 1/2 in hex cut in the same 2D waterjet profile; replaces the r2 broached bar",
             designRef=ref, routeCoverage="partial",
             routes=[dict(id="waterjet-and-gauge", description="Abrasive waterjet profile including the 12.72 mm hex, deburr, hand-fit the hex to a 1/2 in hex go gauge, drill and tap the screw holes",
                          operations=[op("waterjet-profile", "abrasive-waterjet-cutting", "abrasive-waterjet", "6061-T6-plate-9.525mm", [60.0, 190.0, 9.525], 0.1,
                                         ["waterjet taper on 3/8 in plate; cut the hex undersize and file to the gauge if the taper exceeds 0.1 mm"]),
                                      op("tap-holes", "tapping", "drill-press", "6061-T6-plate-9.525mm", [60.0, 190.0, 30.0], 0.1, [])])],
             unresolvedInputs=["school waterjet access is not registered; the same hex can be cut by a qualified outside waterjet service"]),
    ]
    plan = {"schema": "frc-fabrication-plan/v1", "id": "red2-pan-r3-special-routes", "revision": 3,
            "assumption": "all-required-tools-assumed-for-design-only", "evidence": ev, "parts": parts}
    (HERE / "fabrication-plan.json").write_text(json.dumps(plan, indent=1) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
