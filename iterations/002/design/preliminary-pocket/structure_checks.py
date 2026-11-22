"""Sizing calculations for the B'x layout-002 structure and lip: hand formulas with the trial loads.

    python -I -B structure_checks.py FIELD_TRIALS.json OUTPUT.json

Every row states its inputs, formula, value, allowable and margin (allowable / value, pass at >= 1.5 unless the row
says otherwise). Loads come from the field trials (rigid-lip, penalty-contact peaks, so they are upper estimates) and
from the stated abuse cases. Material allowables are handbook class values, not test data: polycarbonate yield
62 MPa and E 2.3 GPa, 6061-T6 yield 276 MPa, alloy-steel shaft yield 400 MPa and E 200 GPa, steel pin shear yield
230 MPa. Preliminary sizing only: no fatigue, no stress concentration, no fastener preload study.
"""
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pocket_geometry as g  # noqa: E402

trials = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
S = trials["summary"]
G = 9.80665
M = g.PIPE_MASS
PC = dict(E=2.3e9, yield_=62e6, rho=1200.0)
rows = []


def row(id_, question, inputs, formula, value, allowable, unit, need=1.5, note=None, smaller_is_better=True):
    margin = (allowable / value) if smaller_is_better else (value / allowable)
    rows.append(dict(id=id_, question=question, inputs=inputs, formula=formula, value=round(value, 4), allowable=round(allowable, 4), unit=unit,
                     margin=round(margin, 2), requiredMargin=need, status="pass" if margin >= need else "fail", **({"note": note} if note else {})))
    return margin


# ---------------------------------------------------------------- lip section
LIP_W = 2 * (161.0 - 3.0) / 1e3            # lip width, m (build_layout.py LIP_W)
t = g.LIP_T / 1e3
EI = PC["E"] * LIP_W * t ** 3 / 12
Sx = LIP_W * t ** 2 / 6
root, knee, tip = g.lip_profile()
hinge = g.LIP_HINGE
ramp_len = math.dist(root, knee) / 1e3
arm_tip = (tip[0] - hinge[0]) / 1e3          # horizontal arm of a load at the tip
drop_tip = (hinge[1] - tip[1]) / 1e3         # the tip is this far below the hinge
STOP_ARM = 0.030                             # sprung stop bar bears on the lip 30 mm from the hinge axis
STOP_PRELOAD = 6.0                           # N m about the hinge holding the lip on its seat
STOP_RATE = 4.0 / math.radians(31.2)         # N m/rad: +4 N m at full over-travel
lip_mass = PC["rho"] * LIP_W * t * (ramp_len + g.LIP_LAND / 1e3 + 0.012)
# 1. static pipe on the tip
m_static = M * G * arm_tip + lip_mass * G * arm_tip / 2
row("L1", "Does the sprung deployed stop hold the lip under the pipe standing on the tip?",
    dict(pipeKg=M, tipArmMm=arm_tip * 1e3, lipKg=round(lip_mass, 3), stopPreloadNm=STOP_PRELOAD),
    "moment about the hinge = M g a_tip + m_lip g a_tip / 2, against the stop preload", m_static, STOP_PRELOAD, "N m", need=3.0)
k_tip = 3 * EI / ramp_len ** 3
row("L2", "How far does the 1/8 in polycarbonate lip sag under the pipe at the tip?",
    dict(EI_Nm2=round(EI, 3), rampLengthMm=round(ramp_len * 1e3, 1)), "cantilever over the ramp: delta = M g / (3 EI / L^3)",
    M * G / k_tip * 1e3, 3.0, "mm", need=1.0, note="Allowable is a 3 mm drop of the 550 mm roll-off height; the 20 deg ramp trials (lower, slower) still capture")
# 2. pipe landing on the lip: energy method, trial normal speed back-computed from the rigid-model peak
v_n = S["lipN"] / math.sqrt(2.0e5 * M)
e_land = 0.5 * M * v_n ** 2
a = 0.090
k_a = 3 * EI / a ** 3
x = math.sqrt(2 * e_land / k_a)
f_land = k_a * x
row("L3", "Bending stress when the ejected pipe lands on the lip",
    dict(rigidModelPeakN=S["lipN"], normalSpeedMps=round(v_n, 3), energyJ=round(e_land, 3), loadPointFromStopMm=a * 1e3, peakForceN=round(f_land, 1),
         deflectionMm=round(x * 1e3, 1)),
    "energy into a cantilever: x = sqrt(2 E / k), F = k x, sigma = F a / (b t^2 / 6)", f_land * a / Sx / 1e6, PC["yield_"] / 1e6, "MPa", need=2.0)
theta = e_land / STOP_PRELOAD
row("L4", "Does the landing push the lip off its sprung stop far enough to change the roll-off?",
    dict(energyJ=round(e_land, 3), stopPreloadNm=STOP_PRELOAD), "worst case, all landing energy into the stop: theta = E / preload; tip drop = theta x tip radius",
    theta * math.hypot(arm_tip, drop_tip) * 1e3, 10.0, "mm", need=1.0,
    note="Upper bound: the lip's own bending (L3) absorbs most of it first. 10 mm keeps the tip above the trough lip top (457.2 + 12 mm)")
# 3. frontal push on the tip (wall or peer): the lip rotates on its sprung stop until the bumper carries the hit
travel = (g.LIP_TIP_X - g.BUMPER_FACE) + 10.0
over = math.degrees(math.acos((arm_tip * 1e3 - travel) / math.hypot(arm_tip, drop_tip) / 1e3)) - math.degrees(math.atan2(drop_tip, arm_tip))
m_over = STOP_PRELOAD + STOP_RATE * math.radians(over)
f_front = m_over / drop_tip
sig_front = f_front * math.sin(math.radians(g.LIP_RAMP_DEG)) * ramp_len / Sx
row("L5", "Frontal hit on the lip tip (24.5 mm ahead of the bumper face): stress with the sprung stop yielding",
    dict(tipAheadOfBumperMm=g.LIP_TIP_X - g.BUMPER_FACE, bumperCrushMm=10.0, overTravelDeg=round(over, 1), stopMomentAtOverTravelNm=round(m_over, 2),
         tipForceN=round(f_front, 1), forceArmMm=round(drop_tip * 1e3, 1)),
    "the push acts 37.6 mm below the hinge and turns the lip down against the stop springs; sigma = F sin(30) L_ramp / (b t^2 / 6)",
    sig_front / 1e6, PC["yield_"] / 1e6, "MPa", need=2.0)
# the same hit on a rigid stop (the layout-001 arrangement): the lip must bend the whole travel
f_rigid = travel / 1e3 * k_tip / math.sin(math.radians(g.LIP_RAMP_DEG)) ** 2
row("L5-control", "Control: the same frontal travel on a rigid deployed stop (layout-001)",
    dict(travelMm=travel, tipForceN=round(f_rigid, 0)), "the ramp bends the whole travel: F = d k / sin^2(30); sigma = F sin(30) L / S",
    f_rigid * math.sin(math.radians(g.LIP_RAMP_DEG)) * ramp_len / Sx / 1e6, PC["yield_"] / 1e6, "MPa", need=2.0,
    note="Expected to fail: this is why the stop is sprung")
# 4. stop bar, springs and hinge pins
f_stop = max(S["lipMomentNm"], m_over) / STOP_ARM
pin_area = math.pi * 0.003 ** 2 / 4
row("L6", "Hinge pins (two 3 mm steel pins, double shear) under the largest stop reaction plus the direct lip load",
    dict(rigidModelLipMomentNm=S["lipMomentNm"], stopArmMm=STOP_ARM * 1e3, stopReactionN=round(f_stop, 0), lipLoadN=S["lipN"]),
    "tau = (F_stop + F_lip) / (2 pins x 2 planes x A)", (f_stop + S["lipN"]) / (4 * pin_area) / 1e6, 230.0, "MPa", need=2.0)
row("L7", "Stop springs: preload each of two extension springs must carry at the 30 mm stop arm",
    dict(stopPreloadNm=STOP_PRELOAD, stopArmMm=STOP_ARM * 1e3), "F = preload / arm / 2", STOP_PRELOAD / STOP_ARM / 2, 150.0, "N", need=1.0,
    note="Allowable is a catalogue-class limit for a 10 mm OD music-wire extension spring; select the article in detailed design")
# 5. deploy swing: spring plus gravity, released from the stowed pose
cgx = (root[0] + tip[0]) / 2 - hinge[0]
cgz = (root[1] + tip[1]) / 2 - hinge[1]
r_cg = math.hypot(cgx, cgz) / 1e3
a_cg = math.atan2(cgz, cgx)
I_lip = lip_mass * (r_cg ** 2 + (ramp_len + 0.025) ** 2 / 12)
th, w, tt, dt = math.radians(g.LIP_STOW_DEG), 0.0, 0.0, 1e-5
k_dep, tau0 = 0.05, 0.10                     # deploy torsion spring: 0.10 N m at the stowed pose, falling 0.05 N m/rad
while th < 0.0 and tt < 3.0:
    # deployed-to-current rotation th about +Y (negative = toward stowed); gravity torque about the hinge
    ang = a_cg - th                          # the stow rotation is counter-clockwise in the XZ view
    tq = tau0 - k_dep * (th - math.radians(g.LIP_STOW_DEG)) + lip_mass * G * r_cg * math.cos(ang)
    w += tq / I_lip * dt
    th += w * dt
    tt += dt
row("L8", "Deploy swing: time from latch release to the deployed stop (torsion spring 0.10 N m plus gravity)",
    dict(lipKg=round(lip_mass, 3), inertiaKgm2=round(I_lip, 6), arrivalRadS=round(w, 2), arrivalEnergyJ=round(0.5 * I_lip * w * w, 3)),
    "rigid lip about the hinge, I th'' = spring + m g r cos(angle), integrated from -120 deg", tt, 1.0, "s", need=1.0,
    note="The lip must be down before the first eject, about 10 s into TELEOP; arrival energy goes into the sprung stop")
row("L9", "Deploy arrival: over-travel of the sprung stop when the lip lands on it",
    dict(arrivalEnergyJ=round(0.5 * I_lip * w * w, 3), stopPreloadNm=STOP_PRELOAD), "theta = E / preload", math.degrees(0.5 * I_lip * w * w / STOP_PRELOAD), 31.2, "deg", need=2.0)

# ---------------------------------------------------------------- roller shafts and side plates
SPAN = 2 * 170.45 / 1e3                      # between the plate bearings, m
hex_af = 12.7e-3
I_hex = 0.0601 * hex_af ** 4
S_hex = 0.1203 * hex_af ** 3
BAND_HI = S["acceptanceBandNmm"][1]
dyn = S["g2RollerN"] / (S["bandNmm"][-1] * g.SQUEEZE)      # trial peak over the static squeeze force at the stiffest trialled tread
for label, load in ((f"acceptance maximum {BAND_HI:.0f} N/mm", BAND_HI * g.SQUEEZE * dyn), ("nominal 80 N/mm", 80.0 * g.SQUEEZE * dyn),
                    ("beyond the band, 160 N/mm (trialled)", S["g2RollerN"])):
    Mb = load * SPAN / 8
    row(f"S1 ({label})", "Bending stress in a 1/2 in steel hex roller shaft under the bite load spread along the pipe",
        dict(rollerLoadN=round(load, 0), spanMm=round(SPAN * 1e3, 1)), "simply supported, uniform load: M = F L / 8; sigma = M / (0.1203 d^3)",
        Mb / S_hex / 1e6, 400.0, "MPa", need=1.5)
    d_steel = 5 * load * SPAN ** 3 / (384 * 200e9 * I_hex) * 1e3
    row(f"S2 ({label})", "Mid-span deflection of the steel hex shaft, against the 6 mm squeeze",
        dict(rollerLoadN=round(load, 0), aluminiumShaftWouldDeflectMm=round(d_steel * 200 / 69, 2)), "delta = 5 F L^3 / (384 E I), I = 0.0601 d^4",
        d_steel, 0.25 * g.SQUEEZE, "mm", need=1.0, note="Allowable: a quarter of the squeeze. An aluminium shaft deflects 2.9 times as far, so the shafts are steel")
n_h, n_v = (g.BITE_X - g.THROAT_X) / (g.R + g.ROLLER_R), g.ROLLER_DZ / (g.R + g.ROLLER_R)
f_h = 2 * S["g1RollerN"] * (abs(n_h) + n_v * 1.0)
LANDING_Z = 500.45                          # side-plate foot on the pocket landing shelf (build_layout.py LANDING_Z)
arm = (g.POCKET_Z - LANDING_Z) / 1e3
BOLT_SPACING = 0.045
t_bolt = f_h / 2 * arm / BOLT_SPACING
row("P1", "Side-plate landing: bolt tension from the receive impact (two 10-32 bolts per plate, 45 mm apart on the landing shelf)",
    dict(rollerPeakN=S["g1RollerN"], horizontalBothRollersN=round(f_h, 0), heightAboveLandingMm=round(arm * 1e3, 1), boltSpacingMm=45.0),
    "horizontal F = 2 N (n_x + n_z mu) with tread mu 1.0; per plate T = (F / 2) h / s", t_bolt, 4500.0, "N", need=2.0,
    note="Allowable: 10-32 thread pull-out from 3 mm of 6061 (shear area pi d L / 2 at 200 MPa); the bolt itself proofs at 7.5 kN")
shelf_arm, shelf_b, shelf_t = (232.0 - 170.45) / 1e3, 0.200, 6.35e-3
row("P4", "Pocket landing shelf: bending at the stringer edge with the P1 bolt tension applied at the side-plate line (1/4 in 6061, 200 mm long, 61.6 mm overhang)",
    dict(loadN=round(t_bolt, 0), overhangMm=round(shelf_arm * 1e3, 1), lengthMm=200.0, thicknessMm=6.35),
    "sigma = 6 F a / (b t^2)", 6 * t_bolt * shelf_arm / (shelf_b * shelf_t ** 2) / 1e6, 276.0, "MPa", need=2.0,
    note="The shelf carries the side-plate foot and the floor post inboard of the stringer, which has material only outboard of Y 232")
row("P5", "Pocket landing shelf: tip deflection under the same load",
    dict(loadN=round(t_bolt, 0), overhangMm=round(shelf_arm * 1e3, 1)), "delta = F a^3 / (3 E I), I = b t^3 / 12, E = 69 GPa",
    t_bolt * shelf_arm ** 3 / (3 * 69e9 * shelf_b * shelf_t ** 3 / 12) * 1e3, 1.0, "mm", need=1.0, note="Allowable: 1 mm at the plate line")
plate_t, plate_w = 6.35e-3, 0.060
row("P2", "Side-plate bending at the landing under the same impact (1/4 in 6061, 60 mm wide section, in plane)",
    dict(momentPerPlateNm=round(f_h / 2 * arm, 1)), "sigma = M / (t w^2 / 6)", f_h / 2 * arm / (plate_t * plate_w ** 2 / 6) / 1e6, 276.0, "MPa", need=2.0)
row("P3", "Plate ligament between the two shaft bores under the band-maximum bite load (tension)",
    dict(biteLoadN=S["g2RollerN"], ligamentMm2=round(6.35 * 40.0, 0)), "sigma = F / (2 plates x t x 40 mm)", S["g2RollerN"] / (2 * plate_t * 0.040) / 1e6, 276.0, "MPa", need=3.0)

# ---------------------------------------------------------------- drive
t_avail = 0.0097 * 30.0 * 36.0 * 0.729
row("D1", "Gearbox static rating against the largest shaft torque in any trial (MAXPlanetary 1/2 in hex socket output)",
    dict(peakShaftNm=max(S["g2ShaftDemandNm"], S["g1BackDriveNm"]), source="REV MAXPlanetary load ratings: 3:1 stage tested to 290 N m, hex socket output fails at 250 N m"),
    "peak trial shaft torque against the weakest rated stage", max(S["g2ShaftDemandNm"], S["g1BackDriveNm"]), 180.0, "N m", need=5.0)
row("D2", "Drive torque at the 30 A limit against the mean torque that stalls it (the 240 N/mm bracket stalls; 160 passes)",
    dict(motorNm=0.291, ratio=36.0, efficiencyAssumed=0.729, holdTorqueNm=round(t_avail, 2)), "T = Kt I N eta; the trials decide stall, this row records the number",
    t_avail, 2.0, "N m", need=1.0, smaller_is_better=False, note="G2 measures the real efficiency: pass if the nip never stalls at 9 V on the installed tread")

out = dict(schema="red2/structure-checks@1", issue="red2-002-preliminary-layout-002",
           basis=__doc__.strip(), stop=dict(armMm=STOP_ARM * 1e3, preloadNm=STOP_PRELOAD, rateNmPerRad=round(STOP_RATE, 3), overTravelDeg=round(over, 1)),
           lip=dict(widthMm=LIP_W * 1e3, thicknessMm=g.LIP_T, massKg=round(lip_mass, 3), deployS=round(tt, 3)),
           rows=rows, failing=[r["id"] for r in rows if r["status"] == "fail" and "control" not in r["id"] and "beyond the band" not in r["id"]])
Path(sys.argv[2]).write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8")
for r in rows:
    print(f"{r['id']:28s} {r['value']:10.3f} {r['unit']:5s} allow {r['allowable']:8.2f} margin {r['margin']:6.2f} {r['status']}")
print("failing:", out["failing"])
