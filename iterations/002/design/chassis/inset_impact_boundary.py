"""Compare signed normal collisions on the governed Dock 2 chassis scenario."""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
VELOCITY = 1.8
RUNOUT = 0.018


def collision(name, own_mass, other_mass, own_speed, other_speed, stiffness, runout=None):
    relative = own_speed-other_speed
    if relative <= 0:
        raise ValueError("Expected closing motion toward the wall in +normal")
    reduced = own_mass if other_mass is None else 1/(1/own_mass+1/other_mass)
    energy = 0.5*reduced*relative**2
    stroke = runout if runout is not None else (2*energy/stiffness)**0.5
    peak = 2*energy/stroke
    return dict(name=name, ownKg=own_mass, otherKg=other_mass, incomingNormalMs=[own_speed,other_speed],
                closingMs=relative, effectiveKg=reduced, compressionEnergyJ=energy,
                strokeMm=stroke*1000, relativeStiffnessNPerMm=peak/stroke/1000,
                ownPeakNormalN=-peak, otherPeakNormalN=peak, ownImpulseToMaximumCompressionNs=-reduced*relative)


def run():
    input_file = HERE/"inset-stagger-all-contacts.json"
    source = json.loads(input_file.read_bytes())
    mass = source["massKg"]
    one_bumper = mass*VELOCITY**2/RUNOUT**2
    cases = [
        collision("Fixed wall, stopped ride",mass,None,VELOCITY,0,one_bumper,RUNOUT),
        collision("Equal physical peer stationary, two equal compliant bumpers in series",mass,mass,VELOCITY,0,one_bumper/2),
        collision("Equal physical peer approaching at the same speed, two equal bumpers",mass,mass,VELOCITY,-VELOCITY,one_bumper/2),
        collision("Equal physical peer approaching at the same speed, total stroke capped at 18 mm",mass,mass,VELOCITY,-VELOCITY,one_bumper/2,RUNOUT)]
    if abs(cases[0]["ownPeakNormalN"]-cases[2]["ownPeakNormalN"])>1e-7 or abs(cases[3]["ownPeakNormalN"]/cases[0]["ownPeakNormalN"]-2)>1e-9:
        raise ValueError("Energy and compliance comparisons lost their physical ordering")
    return dict(authority=source["authority"],sourceSweepSha256=hashlib.sha256(input_file.read_bytes()).hexdigest(),
                cases=cases,workingLimit={"material":"Unwelded 6061-T6 eligible extruded stock",
                "yieldLowerBoundMPa":240,"factor":2,"workingMPa":120,
                "source":"Hydro Alloy 6061, 2019 data sheet p.2 and Coast 112418T61; see 2025-bumper-kit and coast-112418t61-rectangular-tube in the FRC research wiki.",
                "limits":"A net-section beam screen. Fastener bearing, threads, local concentration, fatigue, HAZ and material condition require their own limits."},
                scope="The controlled rare case for these two free approach trips is a one-robot 1.8 m/s centered normal stop against a fixed wall, dissipating its full normal kinetic energy through 18 mm of combined stroke with a linearly increasing force. Event probability and physical foam dynamics are unmeasured. Off-center inertia-relief variants lower contact energy. The independent opposed pinch needs an imposed second external load. A moving opponent changes the relative speed, effective mass and compliance; the cap at 18 mm for two bodies is an adverse control, not a qualified league-wide survival envelope. No synchronized six-body peer run proving a strike or its likelihood is claimed. Routine 10.05 J docking is separate.")


if __name__ == "__main__":
    value=run()
    (HERE/"inset-impact-boundary.json").write_text(json.dumps(value,indent=2)+"\n")
    for row in value["cases"]:
        print(row["name"],round(row["compressionEnergyJ"],4),round(row["ownPeakNormalN"],2),round(row["strokeMm"],3))

