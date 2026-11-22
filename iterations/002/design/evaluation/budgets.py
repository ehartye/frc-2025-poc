"""Mass margin, loaded voltage, governing loads and mechanism time for the four architectures.

A (guarded pan r3) is taken from recorded files. B, C and D are bottom-up ESTIMATES from
stock dimensions and catalogue-class masses; each line names its basis and every estimate
carries a 20 % growth layer (the mass study used 2-50 % item growth). Nothing is weighed.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sources import load  # noqa: E402

G = 9.80665
PC, AL = 1200.0, 2700.0   # polycarbonate, 6061 density kg/m^3
TUBE_1x1x116 = (0.0254 ** 2 - 0.022225 ** 2) * AL   # kg/m
HEX_HOLLOW = 0.30                                   # kg/m, hollow 1/2 in aluminium hex (class value)

# Purchased drive identical to the pan's NEO 550 + UltraPlanetary rows in pan/mass-plan.json
def pan_drive_kg(plan):
    p = {r["item"]: r["kg"] for r in plan["upperBottomUp"]["purchased"]}
    keys = ["REV-21-1651 NEO 550 with leads", "REV-41-1601 3:1 stage", "REV-41-1602 4:1 stage",
            "REV-41-1607 550 input plate", "REV-41-1615 V2 output cartridge", "REV-41-1608 13T pinion",
            "REV-41-1620 1/2 in hex output adapter"]
    return sum(p[k] for k in keys)


def estimate_B(drive):
    L, W = 0.62, 0.322   # pocket+tray run, retained 322 mm floor width
    lines = [
        ("tray floor, 1/8 in polycarbonate", W * L * 0.003175 * PC),
        ("two side walls, 1/8 in polycarbonate, 130 mm", 2 * L * 0.13 * 0.003175 * PC),
        ("hood, 1/16 in polycarbonate", W * 0.25 * 0.0016 * PC),
        ("two nip side plates, 1/4 in 6061, 150x200, 50 % pocketed", 2 * 0.15 * 0.20 * 0.00635 * AL * 0.5),
        ("fence, 1x1x1/16 tube + pad", 0.34 * TUBE_1x1x116 + 0.02),
        ("two roller shafts, hollow 1/2 in hex, 360 mm", 2 * 0.36 * HEX_HOLLOW),
        ("eight 2 in compliant wheels", 8 * 0.025),
        ("four flanged hex bearings", 4 * 0.02),
        ("roller coupling belt and pulleys", 0.08),
        ("NEO 550 + 12:1 UltraPlanetary (pan rows)", drive),
        ("seated-pipe beam break and lead", 0.03),
        ("rear check flap and spring", 0.08),
        ("station inlet guides, polycarbonate (replaces 2.24 kg fixed inlet)", 0.80),
        ("four 1x1x1/16 uprights 300 mm + gussets", 4 * 0.30 * TUBE_1x1x116 + 0.10),
        ("rivets, screws, nuts", 0.35),
    ]
    return lines


def estimate_C(B):
    drop = {"two roller shafts, hollow 1/2 in hex, 360 mm", "eight 2 in compliant wheels", "four flanged hex bearings",
            "roller coupling belt and pulleys", "NEO 550 + 12:1 UltraPlanetary (pan rows)",
            "two nip side plates, 1/4 in 6061, 150x200, 50 % pocketed"}
    lines = [l for l in B if l[0] not in drop]
    lines += [("spring gate, latch, reef-contact plunger and linkage", 0.55),
              ("gate side plates", 0.20)]
    return lines


def estimate_D(B, drive):
    keep = {"station inlet guides, polycarbonate (replaces 2.24 kg fixed inlet)", "tray floor, 1/8 in polycarbonate",
            "two side walls, 1/8 in polycarbonate, 130 mm", "rivets, screws, nuts", "seated-pipe beam break and lead"}
    lines = [l for l in B if l[0] in keep]
    lines += [
        ("end-effector plates, rollers, wheels, bearings (moving)", 0.95),
        ("end-effector roller drive NEO 550 + gearbox (moving)", drive),
        ("pivot shaft, bearings, hard stops", 0.35),
        ("pivot drive NEO + 2-stage MAXPlanetary + chain", 0.75),
        ("two fixed pivot towers, 1x1x1/16 + gussets", 0.70),
        ("pivot encoder and leads", 0.06),
    ]
    return lines


def run():
    ms = load("massStudy")
    plan = load("panMassPlan")
    L = {r["id"]: r["kg"] for r in ms["ledger"]["lines"]}
    gr = {r["item"]: r["kg"] for r in ms["ledger"]["growth"]}
    limit = ms["rule"]["limitKg"]
    honest = ms["ledger"]["honestSingleContingencyKg"]
    pan_keys = ["pan-fabricated", "pan-purchased-sourced", "pan-purchased-estimated", "pan-fasteners"]
    A_upper = sum(L[k] for k in pan_keys) + sum(gr[k] for k in pan_keys) + gr["pan-open-articles"]
    pan_truss = 0.908 * 1.02   # chassis pan trusses inside chassis-fabricated (mass-plan line note), 2 % growth
    base = honest - A_upper - pan_truss   # robot without the scoring upper and its chassis trusses
    drive = pan_drive_kg(plan)
    est = {}
    est["B"] = estimate_B(drive)
    est["C"] = estimate_C(est["B"])
    est["D"] = estimate_D(est["B"], drive)
    est["B'x"] = est["B"] + [("one-time deployed exit lip: polycarbonate lip, hinge, spring and start latch", 0.30)]
    mass = {"A": dict(upperKg=A_upper + pan_truss, basis="mass-study pan lines + growth + 0.908 kg chassis pan trusses",
                      heavyKg=honest, marginKg=limit - honest, kind="recorded (offline, CAD volume + catalogue)")}
    for k, lines in est.items():
        nom = sum(v for _, v in lines)
        upper = nom * 1.20
        mass[k] = dict(upperNominalKg=nom, upperKg=upper, heavyKg=base + upper, marginKg=limit - (base + upper),
                       lines=[dict(item=a, kg=round(b, 4)) for a, b in lines], kind="estimate, +20 % growth")
    pkg = ms["package"]
    mass["A"]["withReductionPackageMarginKg"] = pkg["marginKg"]

    # Loaded voltage: same method and source ranges as preliminary/write_package.py (weak 11.8 V / 32 mOhm)
    voc, res = 11.8, 0.032
    docked_base = 10 + 4 + 6.5          # drive, steer, controls upper bounds while docked
    moving_base = 52 + 20 + 6.5
    mech = {"A": (30.0, 7.45), "B": (25.0, 0.0), "B'x": (25.0, 0.0), "C": (0.0, 0.0), "D": (30.0 + 20.0, 5.0 + 5.0)}
    volts = {}
    for k, (dock_a, move_a) in mech.items():
        volts[k] = dict(mechPeakA=dock_a, mechMovingA=move_a,
                        dockedBusV=voc - (docked_base + dock_a) * res,
                        movingBusV=voc - (moving_base + move_a) * res)
    volts["A"]["recordedConservativeImpactV"] = 8.81
    volts["A"]["source"] = "pan-stop-study-06 front strike conservative pairing (8.81 V), simultaneous >= 9.09 V"
    volts["B"]["basis"] = "roller current limit 25 A during the 0.2 s eject only; custody is geometric so carry draws 0 A"
    volts["D"]["basis"] = "arm 30 A + roller 20 A at the dock; 5 + 5 A holding while driving (estimate)"

    # Governing loads
    M = 0.816466266
    loads = {
        "A": dict(text="soft-stop 1710 N per pad (F3, inhibit 5 ms late); front strike 1183 N per pad; gearbox 41.0 N m "
                       "without clutch vs 40 N m stage failure; worst structural margin +0.07 (off-post 685 N)",
                  governingN=1710, source="pan/README.md r3"),
        "B": dict(text="receive about 500 N peak into a low-friction fence face on 25 mm of compliant backing (custody.py; a foam face lets the pipe climb); 8 g pipe load on the nip 64 N; "
                       f"8 g on the fixed {mass['B']['upperKg']:.1f} kg tray {mass['B']['upperKg'] * 8 * G:.0f} N shared by four mounts; "
                       "no moving inertia can back-drive a gearbox",
                  governingN=501),
        "B'x": dict(text="as B; plus reef or peer contact on the deployed lip tip (not sized)", governingN=501),
        "C": dict(text="same receive pad as B; latch sear load from the pipe at 8 g 64 N", governingN=501),
        "D": dict(text="arm about 0.05-0.08 kg m^2 (estimate) under the same strikes: the same stop, clutch and gearbox "
                       "class as A at roughly 0.5-0.8 times its inertia", governingN=None),
    }
    # Mechanism time per trip (receive-to-reset share that the mechanism controls)
    tip = load("panDownselect")["settings"]["tipDurationS"]
    cycle = {"A": dict(mechanismS=2 * tip + 0.3, basis=f"tip {tip} s + return assumed equal + 0.3 s support/settle"),
             "B": dict(mechanismS=0.1 + 0.15 / 1.3 + 0.3, basis="0.1 s spin-up + 150 mm nip transit at 1.3 m/s + 0.3 s support check"),
             "B'x": dict(mechanismS=0.1 + 0.15 / 1.0 + 0.3, basis="0.1 s spin-up + 150 mm at 1.0 m/s + 0.3 s support check"),
             "C": dict(mechanismS=0.6 + 0.3, basis="gravity roll-out from rest about 0.6 s + 0.3 s support check"),
             "D": dict(mechanismS=0.4 + 0.2 + 0.4 + 0.3, basis="0.4 s out, 0.2 s eject, 0.4 s return, 0.3 s check")}
    fab = {"A": dict(fabricated=136, purchased=357, notes="142 rivets, 50 rivet nuts, 136 screws; waterjet plate stacks and hex, "
                                                         "hand-brake 95.2 deg clip, slip clutch, cam and switch (target-roster.json, r3 README)"),
           "B": dict(fabricated=16, purchased=70, notes="router/bandsaw/drill press; polycarbonate and 1x1 tube; COTS rollers and gearbox (estimate)"),
           "B'x": dict(fabricated=19, purchased=80, notes="as B plus the deployed lip, hinge and start latch (estimate)"),
           "C": dict(fabricated=16, purchased=40, notes="as B plus a tuned latch and trigger linkage (estimate)"),
           "D": dict(fabricated=26, purchased=110, notes="arm plates, pivot towers, two drives, hard stops (estimate)")}
    launch = launch_speed_control()
    return dict(limitKg=limit, baseWithoutUpperKg=base, mass=mass, voltage=volts, loads=loads, cycle=cycle,
                fabrication=fab, launchControl=launch)


def launch_speed_control(wheel_d=0.0508, free_rpm=11000.0):
    """Nip surface speed against bus voltage for the NEO 550 (free speed scales with voltage).
    Bands come from delivery.py (every stand-off 0-100 mm under every e, mu): B 1.1-1.5 m/s at the
    0.67 m exit and 0 deg; B'x 0.6-1.5 m/s at 0.55 m and 0 deg. Pipe exit speed versus surface speed
    (slip, wheel compression) is unmeasured."""
    import math
    out = {}
    for ratio in (9, 12, 15):
        free = {v: free_rpm * v / 12.0 / ratio / 60 * math.pi * wheel_d for v in (12.5, 12.0, 11.0, 10.3, 10.0, 9.0)}
        out[f"{ratio}:1"] = {f"{v} V": round(x, 3) for v, x in free.items()}
    bands = {"B": (1.1, 1.5), "B'x": (0.6, 1.5)}
    open_loop = (12.5 - 10.0) / (12.5 + 10.0)      # +/- fraction if duty is fixed
    req = {}
    for k, (lo, hi) in bands.items():
        mid = (lo + hi) / 2
        req[k] = dict(band=[lo, hi], target=mid, halfWidthPct=round(100 * (hi - lo) / (hi + lo), 1),
                      openLoopSpreadPct=round(100 * open_loop, 1),
                      headroomAt10V_12to1=round(out["12:1"]["10.0 V"] / mid, 2),
                      headroomAt9V_12to1=round(out["12:1"]["9.0 V"] / mid, 2))
    req["finding"] = ("12:1 gives 2.44 m/s free at 12 V and 2.09 m/s at 10.3 V. B needs 1.1-1.5 m/s (+/-15 %); "
                      "fixed duty alone spreads about +/-11 % from 10 to 12.5 V before slip and load, so B needs "
                      "closed-loop roller velocity on the NEO 550 encoder (target 1.3 m/s, tolerance +/-5 %) and an "
                      "eject interlock below 9.5 V; headroom at 10 V is 1.6x. B'x (0.6-1.5 m/s, +/-43 %) tolerates "
                      "open-loop duty.")
    return dict(freeSurfaceMps=out, requirements=req)


if __name__ == "__main__":
    import json
    r = run()
    for k, v in r["mass"].items():
        print(k, round(v["upperKg"], 3), round(v["heavyKg"], 3), round(v["marginKg"], 3))
    print(json.dumps(r["voltage"], indent=0))
    print(json.dumps(r["cycle"], indent=0))
