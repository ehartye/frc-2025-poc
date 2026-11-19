"""Compare six simultaneous level scale readings with the registered balance targets."""
import argparse
import json
import math
from pathlib import Path

from stability import CLOCK, DEFAULT, G, HERE, Load, assembly, drive_check, extremum


def compare(spec,review):
    review=review["poses"]
    pose=spec["pose"]
    load=spec["load"]
    if pose not in review or load not in {"empty","seated"} or spec.get("stepSha256")!=review[pose]["step_sha256"]:
        raise ValueError("Select the registered pose, payload and assembly identity")
    if spec.get("units")!="kg" or not spec.get("source") or spec.get("basis") not in {"measured","synthetic"}:
        raise ValueError("Supply a scale source, evidence basis and kg units")
    goal=review[pose][load]
    targets=goal["prior_balance_targets"]
    weights=spec["scales"]
    error=spec["absoluteErrorKg"]
    if not isinstance(error,(float,int)) or not math.isfinite(error) or error<0:
        raise ValueError("Invalid absolute error per scale")
    rows=[]
    for side,side_sign in (("L",1),("R",-1)):
        for axle,k in (("front",1),("center",0),("rear",-1)):
            key=f"wheel-{axle}-{side}"
            v=weights[key]
            if not isinstance(v,(float,int)) or not math.isfinite(v) or v<=error:
                raise ValueError("Every modeled wheel must have a resolved positive static reaction")
            location=(k*assembly.drive_model.CENTERS,side_sign*DEFAULT["trackM"]*500,abs(k))
            rows.append(Load(key,(v-error,v+error),location*2,location*2))
    if set(weights)!={r.name for r in rows}:
        raise ValueError("Six installed wheel readings are required")
    axes=[(1,0,0),(0,1,0),(0,0,1)]
    x,y,outer=[[extremum(rows,a,False),extremum(rows,a,True)] for a in axes]
    radius=max(math.hypot(xx,yy) for xx in x for yy in y)/1000
    side_min=.5-max(map(abs,y))/(DEFAULT["trackM"]*1000)
    mass=[sum(r.mass[i] for r in rows) for i in (0,1)]
    checks={"mass_inside_modeled_interval":goal["mass_kg"][0]<=mass[0] and mass[1]<=goal["mass_kg"][1],
            "radial_CG":radius<=targets["requires_radius_m_at_most"],
            "static_side_reaction":side_min>=targets["requires_static_side_fraction_at_least"],
            "static_outer_axles":outer[1]<=targets["requires_outer_axle_fraction_at_most"]}
    lateral_acc=radius*math.hypot(2*CLOCK["turnWheelAccelMps2"]/DEFAULT["trackM"],CLOCK["maximumTurnRadS"]**2)
    dynamic_side=side_min-goal["cg_mm"][2][1]/1000*lateral_acc/(G*DEFAULT["trackM"])
    # Without assumptions about center-wheel preload, all the load could act at the
    # front/rear axles during the spin. Keep the installed CAD yaw bound.
    spin=None
    if dynamic_side>0 and checks["mass_inside_modeled_interval"]:
        ends=[drive_check(m,goal["yaw_inertia_upper_kg_m2"],radius,dynamic_side,1.0)["turn"] for m in mass]
        spin={key:max(e[key] for e in ends) for key in ends[0]}
    return {"basis":spec["basis"],"source":spec["source"],"step_sha256":spec["stepSha256"],
            "pose":pose,"load":load,"mass_kg":mass,"cgX_mm":x,"cgY_mm":y,
            "minimum_static_side_fraction":side_min,"outer_axle_fraction":outer,"checks":checks,
            "minimum_modeled_turn_side_fraction":dynamic_side,"turn_with_unrestricted_outer_axle_loading":spin,
            "static_balance":"within targets" if all(checks.values()) else "outside targets",
            "scope":"Stationary, level, supported on all six wheels. Dynamic axle loading, grip and turn scrub need separate observations."}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("readings",type=Path)
    opt=p.parse_args()
    print(json.dumps(compare(json.loads(opt.readings.read_text()),json.loads((HERE/"review.json").read_text())),indent=2))
