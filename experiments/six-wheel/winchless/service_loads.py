"""Static fixture, bench anchorage and four-pad acceptance exercises in robot coordinates."""
import json
import math
from itertools import product
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent


def solve():
    fit=json.loads((HERE/"fit.json").read_bytes())
    loads=json.loads((ROOT/"fixed-lift-omission/mechanics.json").read_bytes())
    gravity=9.80665
    mass=18.  # Receiving limit for the actual unloaded stowed assembly and attached service equipment.
    force=mass*gravity*2.
    modulus=200000.  # Hypothetical steel properties; kN loads are not inferred from a material label.
    post_i=(40**4-34**4)/12
    boom_i=(30*35**3-24*29**3)/12
    stands={}
    for name,(x,y,z) in fit["locationsMm"].items():
        span=470-y
        offset=490-y
        moment=force*span
        weld_i=2*(3/math.sqrt(2))*30**3/12  # two effective 30 mm vertical fillets at boom root
        pad_area=min(fit["contacts"][name+"-"+s]["areaMm2"] for s in ("L","R"))
        # Bolt rows at 452/538 mm. Resolve couple at inner row with a compression-only bench.
        stands[name]={"wholeReferenceReactionN":force,"rootMomentNm":moment/1000,
            "grossBoomBendingMPa":moment*17.5/boom_i,"grossColumnBendingMPa":force*offset*20/post_i,
            "rootWeldBendingMPa":moment*15/weld_i,"rootWeldDirectShearMPa":force/(2*30*3/math.sqrt(2)),
            "boomTipFlexMm":force*span**3/(3*modulus*boom_i),
            "columnRotationAtPadMm":force*offset**2*(z-25)/(modulus*post_i),
            "outerRowTotalTensionN":force*(452-y)/(538-452),
            "nominalPadPressureMPa":force/pad_area,
            "standAloneTipMomentAboutInsideFootNm":force*(440-y)/1000}
    rear_y,front_y=fit["locationsMm"]["rear"][1],fit["locationsMm"]["front"][1]
    rear_x,front_x=fit["locationsMm"]["rear"][0],fit["locationsMm"]["front"][0]
    locations=[(x,y) for x,span in ((rear_x,rear_y),(front_x,front_y)) for y in (span,-span)]
    # Independent pad readings come from a calibrated setup. Do not infer reactions from equal-height screws.
    def readings(values):
        extreme=[]
        for errors in product((-.1,.1),repeat=4):
            masses=[v+e for v,e in zip(values,errors)]
            total=sum(masses)
            cx,cy=[sum(m*pt[n] for m,pt in zip(masses,locations))/total for n in range(2)]
            half=rear_y+(front_y-rear_y)*(cx-rear_x)/(front_x-rear_x)
            slope=(front_y-rear_y)/(front_x-rear_x)
            edge=min(cx-rear_x,front_x-cx,(half-abs(cy))/math.hypot(1,slope))
            extreme.append((min(masses),total,edge))
        low=min(v[0] for v in extreme);high=max(v[1] for v in extreme);margin=min(v[2] for v in extreme)
        return {"indicationsKg":values,"minPadKg":low,"maxTotalKg":high,"minSupportPolygonDistanceMm":margin,
                "accepted":low>0 and high<=mass and margin>=20}
    return {"referenceMovingMassKg":loads["oneSideLockLoadKg"],"assumedFixtureLimitKg":mass,
            "perPadReferenceAccelerationG":2.,"tableAnchorageInstalled":None,"asBuiltStiffness":None,
            "stands":stands,"padOrder":["rear-L","rear-R","front-L","front-R"],
            "independentScaleAbsoluteErrorKg":.10,"minimumScalePolygonMarginMm":20.,
            "hypotheticalEvenTrial":readings([4.25]*4),"hypotheticalRearOnlyTrial":readings([8.5,8.5,0,0]),
            "actualScaleTrial":None}


if __name__=="__main__":
    (HERE/"service-loads.json").write_text(json.dumps(solve(),indent=2)+"\n",encoding="utf8",newline="\n")
