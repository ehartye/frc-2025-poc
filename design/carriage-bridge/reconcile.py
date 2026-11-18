"""Apply one signed lift-member substitution to the existing weight and fault screens."""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
SCREW=ROOT/"design/lift-screw"
sys.path.insert(0,str(SCREW))
import screen
from drag import interval

NATIVE=HERE/"native.json"
MASS=ROOT/"design/competition-package/shop-duplex/mass.json"
DUPLEX=ROOT/"design/receiver-lift/delivery-retention.json"
BASE_BOM=ROOT/"design/drivetrain/review/bom.csv"
ARM=ROOT/"design/lift-arrest/study.json"
LIGHT=ROOT/"design/installed-procurement/clamp-review.json"
SCREW_STUDY=SCREW/"screen.json"


def data(p):return json.loads(p.read_bytes())


def window_area(w,h,r):return w*h-(4-math.pi)*r*r


def stock_interval():
    with (HERE/"windows.csv").open(encoding="utf-8",newline="") as f:rows=list(csv.DictReader(f))
    def total(dim,rad):
        return sum(window_area(float(r["maxU"])-float(r["minU"])+dim,
                               float(r["maxV"])-float(r["minV"])+dim,8+rad) for r in rows)
    # Variation and density are explicit acceptance assumptions; no lot measured.
    lo=((322-.25)*(410-.25)-total(.2,-.25))*4.9*2.65e-6
    hi=((322+.25)*(410+.25)-total(-.2,.25))*5.1*2.75e-6
    return [lo,hi]


def beam_cases(mass):
    # Rail alone: fixed root at front face of seated crossbar, no composite
    # plate width counted. A separate adverse case removes rail restraint.
    E=68300
    lengths=[(36,3,1.5),(3,35,20.5),(3,35,20.5)]
    cy=sum(b*h*y for b,h,y in lengths)/sum(b*h for b,h,y in lengths)
    inertia=sum(b*h**3/12+b*h*(y-cy)**2 for b,h,y in lengths)
    def cantilever(force,length,I,c):
        return {"deflectionMm":force*length**3/(3*E*I),"nominalRootStressMPa":force*length*c/I}
    # Longest loaded contact-reaction arm from x=-77 to x=160. The overhung
    # shelf may develop a negative rear rail reaction and larger front force.
    span=237
    two_g=2*mass*9.80665
    overhang=535 # published maximum tip at full extension in robot coordinates
    moment_arm=overhang+77
    tip_load=two_g*moment_arm/span
    result={"caseMassKg":mass,"twoGLoadN":two_g,"railIxxMm4":inertia,
            "halfLoadSharedAtOuterEnd":cantilever(two_g/2,span,inertia,max(cy,38-cy)),
            "fullLoadOnOneRailAtOuterEnd":cantilever(two_g,span,inertia,max(cy,38-cy)),
            "oneSideExtendedTipEquivalentForceN":tip_load,
            "oneSideExtendedTipMomentMatched":cantilever(tip_load,span,inertia,max(cy,38-cy)),
            "ineffectiveRail43mmPlateStrip":cantilever(two_g,span,43*5**3/12,2.5)}
    # Retained 28-mm wide bridge strip around the fixed rack (Y65..93).
    # Use a rectangle inscribed in the strip and ignore rack stiffness.
    bay=82
    I=28*5**3/12
    result["rackWeb80NMidspan"]={"simpleSupportDeflectionMm":80*bay**3/(48*E*I),
        "simpleSupportStressMPa":80*bay*2.5/(4*I),
        "oneEndLost":cantilever(80,bay,I,2.5)}
    return result


def report():
    prior=data(MASS)
    both=data(DUPLEX)["registeredMoving"]["bodyMassKg"]["slide-base-bridge"]
    with BASE_BOM.open(encoding="utf-8",newline="") as f:
        [stock]=[r for r in csv.DictReader(f) if r["id"]=="slide-base-bridge"]
    if [float(stock["minMassKg"]),float(stock["maxMassKg"])]!=both["kg"] or both["frame"]!="lift":
        raise ValueError("Historical mass ownership does not match")
    old=both["kg"][0]
    if both["kg"][1]!=old:raise ValueError("Provide a correlated stock interval")
    native=data(NATIVE)["poses"]["place"]
    nominal=native["volumeMm3"]*2.70e-6
    new=stock_interval()
    delta=[m-old for m in new]
    clamp=data(LIGHT)["deltaKg"]
    arm=data(ARM)
    def find_mass(obj):
        if isinstance(obj,dict):
            if "massWithoutFastenersKg" in obj:yield obj["massWithoutFastenersKg"]
            for val in obj.values():yield from find_mass(val)
        elif isinstance(obj,list):
            for val in obj:yield from find_mass(val)
    [stop]=find_mass(arm)
    carry=prior["poses"]["transport"]
    screw=data(SCREW_STUDY)
    budget=screw["mass"]
    if "slide-base-bridge" in budget["explicitDuplexRetirementsKg"]:raise ValueError("Repeated retirement")
    source_range=arm["size"]["massRangeKg"]
    if abs(source_range[1]-prior["liftLoadedUpperKg"])>1e-8:raise ValueError("Mismatched lift editions")
    low,high=[source_range[i]+delta[i] for i in (0,1)]
    thread=screw["threads"]["three-quarters-6"]["screw"]
    before=screen.coast(thread,.10,source_range[0],.8,coupled=True)
    after=screen.coast(thread,.10,low,.8,coupled=True)
    selected=18/21
    drags={}
    for friction in ((0,.2),(.1,.15)):
        drags[str(friction)]=interval(thread,.5575,(low,high+screw["assumedExtraMovingKg"]),selected,*friction,
                                      downstream_inertia=.000020)
    head=carry["R103_margin_at_high_kg"]-delta[1]
    replacement_room=budget["fullReplacementRemainderAtUpperKg"]+budget["shopCarrierAllowanceKg"]-clamp-delta[1]
    conservative_room=budget["sameRoomWithLowOriginalDriveCreditKg"]+budget["shopCarrierAllowanceKg"]-clamp-delta[1]
    files=(MASS,DUPLEX,BASE_BOM,ARM,LIGHT,SCREW_STUDY,NATIVE,HERE/"windows.csv",SCREW/"drag.py",SCREW/"screen.py")
    return {"sourceSha256":{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        "retiredId":"slide-base-bridge","retiredBookedKg":old,"referenceCadKg":native["referenceVolumeMm3"]*2.70e-6,
        "newNominalKg":nominal,"newAllowanceKg":new,"changeKg":delta,
        "R103Kg":[carry["R103_kg"][i]+delta[i] for i in (0,1)],
        "R408Kg":[carry["R408_kg"][i]+delta[i] for i in (0,1)],"R103HighMarginKg":head,
        "headroomWithLightClampsAndEndStopsKg":head-clamp-stop,
        "loadedWholeRobotHighKg":carry["seated"]["mass_kg"][1]+delta[1],
        "liftMassRangeKg": [low,high],"replacementScrewMassRangeWithAllowanceKg":[low,high+screw["assumedExtraMovingKg"]],
        "screwRemainderWithLightClampAtHighOldDriveCreditKg":replacement_room,
        "screwRemainderWithLightClampAtLowOldDriveCreditKg":conservative_room,
        "oldLightCoast":before,"newLightCoast":after,"screwDragAt18to21":drags,
        "conditionalStructure":beam_cases(high),"asBuiltMassKg":None}


if __name__=="__main__":
    (HERE/"review.json").write_text(json.dumps(report(),indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
