"""Bound the removable lower tie's installation torque against its actual seats and matched bumper load cases."""
import hashlib
import json
import math
from pathlib import Path


HERE = Path(__file__).resolve().parent


def read(name):
    return json.loads((HERE / name).read_bytes())


def digest(name):
    return hashlib.sha256((HERE / name).read_bytes()).hexdigest()


def run():
    head = read("port-504-independent.json")
    force = read("lower-terminal-anchor-loads.json")
    source = read("terminal-bolt-loads.json")["assumptions"]
    head_area = min(s["areaMm2"] for k, s in head["allSixHeadSeats"].items() if "91251A542" in k)
    # The actual load washer has a 6.70 mm bore; the continuous flat contact ends at diameter 8.89.
    ri, ro = 3.35, 4.445
    if abs(math.pi * (ro**2 - ri**2) - head_area) > .001:
        raise ValueError("Actual seated annulus differs from the torque model")
    contact_radius = 2 / 3 * (ro**3 - ri**3) / (ro**2 - ri**2)
    pitch, diameter = 1.27, 6.35
    pitch_diameter = diameter - .64952 * pitch
    root_diameter = diameter - 1.30 * pitch
    stress_area = math.pi * (diameter - .9743 * pitch)**2 / 4
    def leverage(friction):
        thread = pitch_diameter / 2 * math.tan(
            math.atan(pitch / (math.pi * pitch_diameter)) + math.atan(friction / math.cos(math.pi / 6)))
        return thread, thread + friction * contact_radius
    friction = [.12, .18]
    low, high = [leverage(v) for v in friction]
    result = dict(authority=head["authority"], inputSha256={n: digest(n) for n in
                 ("port-504-independent.json", "terminal-bolt-loads.json", "lower-terminal-anchor-loads.json")},
                 annularBearingMm=[ri, ro], areaMm2=head_area, meanHeadFrictionRadiusMm=contact_radius,
                 trialThreadAndHeadFrictionRange=friction, toolRelativeScatter=.04,
                 threadPitchMm=pitch, conditionalProofMPa=source["assumedMinProofMPa"], windows={})
    for min_force, max_force in ((6000, 9000), (4000, 8000)):
        # The low end must be possible with high friction and a low-reading application, and vice versa.
        limits = [min_force * high[1] / .96 / 1000, max_force * low[1] / 1.04 / 1000]
        actual = {}
        if limits[0] <= limits[1]:
            dial = round(sum(limits) / 2, 1)
            actual = dict(dialNm=dial, clampRangeN=[1000 * dial * .96 / high[1], 1000 * dial * 1.04 / low[1]])
        outcomes = {}
        for tag, values in force["results"].items():
            peak = values["maxTensionN"]["value"]
            external = 2 * peak
            remaining = .8 * min_force - .85 * external
            bolt = max_force + .35 * external
            torsion = 16 * max_force * high[0] / (math.pi * root_diameter**3)
            combined = math.hypot(bolt / stress_area, math.sqrt(3) * torsion)
            outcomes[tag] = dict(factoredExternalN=external, clampRemainingAfter20PctSettleN=remaining,
                                 headAverageMPa=bolt / head_area, boltCombinedMPa=combined,
                                 fractionConditionalProof=combined / source["assumedMinProofMPa"])
        result["windows"][f"{min_force}-{max_force}"] = dict(dialIntervalNm=limits, feasible=limits[0] <= limits[1],
                                                           illustrativeInstallation=actual, loadCases=outcomes)
    result["scope"] = ("Lower cartridge primary A542 screws only. The pilots carry transverse shear. The modeled coefficients, "
                       "tool scatter, preload loss and joint load sharing are trial bounds. A representative nutbar, screw finish "
                       "and washer must establish a controlled torque–clamp relation, retention and repeatability. "
                       "A positive average clamp and a screw proof screen do not measure washer bending or certify 10-32 keepers, "
                       "the outer bumper mounts, fatigue, or incoming stock.")
    return result


if __name__ == "__main__":
    output = run()
    (HERE / "port-preload.json").write_text(json.dumps(output, indent=2) + "\n")
    for window, case in output["windows"].items():
        print(window, case["dialIntervalNm"], case["illustrativeInstallation"])
