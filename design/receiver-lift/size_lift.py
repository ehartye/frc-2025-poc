"""Size the station receiver lift against the pinned carriage BOM and actual mission moves."""
import csv
import hashlib
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
MM = 0.001
INCH = 0.0254
G = 9.80665
MOTOR_RPM = 5676
FREE_CURRENT = 1.8
STALL_CURRENT = 105.0
STALL_TORQUE = 2.6
OMEGA_FREE = MOTOR_RPM * 2 * math.pi / 60
PHASE_LIMIT_A = 35.0
ROTOR_STACK_INERTIA = 0.000075  # kg m², motor-side engineering allowance


def demand(speed, acceleration, axial_force, radius, reduction, efficiency, inertia):
    omega = speed / radius * reduction
    alpha = acceleration / radius * reduction
    torque = axial_force * radius / (reduction * efficiency) + inertia * alpha
    current = FREE_CURRENT + (STALL_CURRENT - FREE_CURRENT) * torque / STALL_TORQUE
    resistance = 12 / STALL_CURRENT
    emf = omega / OMEGA_FREE * (12 - resistance * FREE_CURRENT)
    voltage = emf + current * resistance
    return omega * 60 / (2 * math.pi), current, voltage


def profile(stroke, seconds, mass, radius, reduction, efficiency, inertia):
    # An upper envelope for motor effort in both directions. Brake heating during
    # overhauling descent is a separate energy/duty requirement.
    rpm_max = amps_max = volts_max = speed_max = 0.0
    min_tension = math.inf
    for i in range(1001):
        u = i / 1000
        speed = 30 * stroke * (u * (1-u))**2 / seconds
        accel = abs(60 * stroke * u*(1-u)*(1-2*u) / seconds**2)
        guide_drag = 20.0
        force = mass*(G + accel) + guide_drag
        rpm, amps, volts = demand(speed, accel, force, radius, reduction, efficiency, inertia)
        min_tension = min(min_tension, (mass*(G - accel) - guide_drag)/2)
        rpm_max, amps_max = max(rpm_max, rpm), max(amps_max, amps)
        volts_max, speed_max = max(volts_max, volts), max(speed_max, speed)
    return {"peak_rpm":rpm_max, "peak_motor_phase_A":amps_max, "peak_needed_V":volts_max,
            "peak_lift_mps":speed_max, "minimum_line_tension_per_side_N":min_tension,
            "within_9p5V_35A_envelope":volts_max <= 9.5 and amps_max <= PHASE_LIMIT_A and min_tension > 0}


def run():
    bom = ROOT / "design/station-l1/bom.csv"
    with bom.open(newline="", encoding="utf-8") as stream:
        parts = list(csv.DictReader(stream))
    participating = [r for r in parts if r["kinematicFrame"] in {"lift", "middle", "slide"}]
    if {r["kinematicFrame"] for r in participating} != {"lift","middle","slide"}:
        raise ValueError("Missing moving frame inventory")
    config = json.loads((ROOT/"mission/candidate.json").read_text())
    poses = config["poses"]
    moving = sum(float(row["maxMassKg"]) for row in participating)
    extra = 1.0  # bracketry, wiring and sensing carried above the measured CAD
    dry = moving + extra
    loaded = dry + config["motion"]["pipeMass_kg"]
    # Timings of the mission's quintic loaded and empty recovery poses.
    moves = [("loaded_receive_to_carry","receive","transport",2.0,loaded),
             ("loaded_carry_to_place","transport","place",1.5,loaded),
             ("empty_place_to_carry","place","transport",1.0,dry),
             ("empty_carry_to_receive","transport","receive",2.0,dry),
             ("rejected_one_second_return","transport","receive",1.0,dry)]
    history_radius = (0.750 + 0.125)*INCH/2
    catalog_radius = (0.787 + 0.125)*INCH/2
    radii = [history_radius,catalog_radius]
    # Gives both extremes of the first layer for the two drawing revisions.
    winch = {}
    for name, src, dst, seconds, mass in moves:
        stroke = abs(poses[src]["height"]-poses[dst]["height"])*MM
        winch[name] = [profile(stroke,seconds,mass,r,16,0.78,ROTOR_STACK_INERTIA) for r in radii]
    travel = (max(p["height"] for p in poses.values())-min(p["height"] for p in poses.values()))*MM
    rope_d = 0.125*INCH
    drum_width = 1.06*INCH
    retained_turns = 2.0  # proposed minimum tail wraps; termination carries retained load
    turns = retained_turns + travel/(2*math.pi*history_radius)
    max_stroke = max(abs(poses[s]["height"]-poses[d]["height"])*MM for _,s,d,_,_ in moves)
    screw_lead = INCH/6
    screw_radius = screw_lead/(2*math.pi)
    # Equivalent rotational inertia bounds two 0.52 m steel shafts as solid major-diameter cylinders.
    screw_length = 0.52
    screw_mass = 1.15*0.45359237/0.3048 * screw_length
    screw_inertia = 2 * 0.5 * screw_mass * (0.75*INCH/2)**2
    screw_profile = profile(max_stroke,2.0,loaded,screw_radius,1.0,0.34,ROTOR_STACK_INERTIA+screw_inertia)
    torque = loaded * G * catalog_radius
    return {"basis":"Engineering envelope; no commissioned device, physical load or holding test",
            "input_sha256":{str(p.relative_to(ROOT)).replace(chr(92),"/"):hashlib.sha256(p.read_bytes()).hexdigest()
              for p in [bom, ROOT/"mission/candidate.json"]},
            "motion_schedule_source":"1578434f5e3c20468ccdea51fca300dc27687777 scripts/station_l1_mission.py and scripts/station_l1_phase.py",
            "moving_upper_kg":moving,"additional_allowance_kg":extra,"loaded_design_kg":loaded,
            "candidate":"NEO V1.1 + 16:1 Sport + synchronized twin 1:1 drums",
            "winch":winch,"single_layer":{"total_turns_with_tail":turns,"width_needed_mm":turns*rope_d/MM,
                "width_available_mm":drum_width/MM,"margin_mm":(drum_width-turns*rope_d)/MM},
            "holding":{"gravity_torque_at_output_Nm":torque,"two_times_static_design_Nm":2*torque,
                "energy_to_arrest_J_at_10mm_drop":loaded*G*.01+.5*loaded*winch["loaded_receive_to_carry"][0]["peak_lift_mps"]**2,
                "actual_ratchet_catch_rating":None,"power_loss_during_descent_verified":False},
            "acme_alternative":{"motor":screw_profile,"critical_safe_rpm_fixed_supported":
                4.76e6*.537*1.55/(2*(screw_length/INCH)**2),"thread_surface_ft_per_min":screw_profile["peak_rpm"]*59/300,
                "vendor_allowable_at_duty":None,"power_off_hold_verified":False}}


if __name__ == "__main__":
    print(json.dumps(run(), indent=2, allow_nan=False))
