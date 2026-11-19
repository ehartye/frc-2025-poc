"""Sensitivity to post bending and to pressure on an idealized flat cover panel."""
from math import pi, sin
from pathlib import Path
import json

HERE=Path(__file__).resolve().parent


def panel_displacement(gauge, terms):
    # Navier sine modes of D * biharmonic(w) = pressure, on a simply
    # supported rectangle. Each sinc integrates the centered 20 mm square.
    length, width, patch, force=240.,43.,20.,5.
    rigidity=2300*gauge**3/(12*(1-.37**2))
    series=0.
    for m in range(1,terms+1,2):
        p=m*pi*patch/(2*length)
        for n in range(1,terms+1,2):
            q=n*pi*patch/(2*width)
            series+=sin(p)/p*sin(q)/q/((m/length)**2+(n/width)**2)**2
    return 4*force*series/(length*width*pi**4*rigidity)


def post_displacement():
    # Transverse force perpendicular to the turned post, along the chassis
    # plane. A lateral force on the outer cover instead acts along its axis.
    length=272.4376-.30-235
    total=0.
    for a,b,od,bore in ((0,8.5,7.2,3.),(8.5,length-8.2,5.,0.),(length-8.2,length,8.,3.)):
        inertia=pi*(od**4-bore**4)/64
        total+=20*((length-a)**3-(length-b)**3)/(3*69000*inertia)
    return total


if __name__=="__main__":
    receipt=json.loads((HERE/"fit.json").read_bytes())
    sampled=min(v["sampleMin"][0] for k,v in receipt["drive"].items() if k.startswith("tray-"))
    lower=min(v["continuousLowerMm"] for k,v in receipt["drive"].items() if k.startswith("tray-"))
    # These are proposed fixture allocations, independent of the motion sampling allowance.
    allowances={"assemblyAndFormMm":.30,"relativeWheelRunoutMm":.25,"shellAxialFloatMm":.30}
    available=lower-sum(allowances.values())
    data={"minimumSampledSkinToDriveMm":sampled,"certifiedRigidTranslationMm":lower,
          "proposedAllowances":allowances,"remainingTrialDeflectionMm":available,
          "minimumFixtureStiffnessNPerMmAt5N":5/available,"transversePost20Nmm":post_displacement(),
          "referencePanel":{"spanMm":[240,43],"centralPatchMm":[20,20],"forceN":5,
                            "modulusMPa":2300,"assumedPoissonRatio":.37,
                            "simplySupportedLinearMm":{str(t):panel_displacement(t,159) for t in (.5,.75,1.)}}}
    assert abs(panel_displacement(.5,79)-panel_displacement(.5,159))<.0001
    (HERE/"compliance.json").write_text(json.dumps(data,indent=2)+"\n",encoding="utf8",newline="\n")
