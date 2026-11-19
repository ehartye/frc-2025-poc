"""Bound whole-robot mass properties and planar load transfer from registered pose geometry."""
import argparse
import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
DESIGN=HERE.parent
ROOT=DESIGN.parent
CAD=DESIGN/"competition-package"
sys.path[:0]=[str(CAD),str(DESIGN/"electrical-inspection"),str(DESIGN/"drivetrain")]
import build as assembly
import inspection
from route_check import DEFAULT,demand

G=9.80665
CLOCK=json.loads((ROOT/"mission/67kg-field-clock.json").read_text())["focal_drive"]
BASE_BOM=DESIGN/"drivetrain/review/bom.csv"
GRIP_BOM=DESIGN/"receiver-drives/bom.csv"
BUMPER_BOM=DESIGN/"bumper-kit/mass.csv"


@dataclass(frozen=True)
class Load:
    name:str
    mass:tuple
    bounds:tuple  # admissible CG box in mm
    extent:tuple  # complete material bounds, for inertia


def union(boxes):
    return tuple((min if i<3 else max)(b[i] for b in boxes) for i in range(6))


def extremum(loads,coefficients,maximize):
    """Optimize sum(m*q)/sum(m), choosing independent mass and position intervals."""
    limits=[]
    for load in loads:
        q=sum(c*load.bounds[i+(3 if (c>0)==maximize else 0)] for i,c in enumerate(coefficients))
        limits.append((q,*load.mass))
    lo=min(p[0] for p in limits)
    hi=max(p[0] for p in limits)
    for _ in range(56):
        t=(lo+hi)/2
        residual=sum((q-t)*(large if (q>t)==maximize else small) for q,small,large in limits)
        if residual>0: lo=t
        else: hi=t
    return (lo+hi)/2


def csv_rows(file):
    with file.open(encoding="utf-8-sig",newline="") as stream:
        return list(csv.DictReader(stream))


def add_bumpers(shapes,add):
    rows=csv_rows(BUMPER_BOM)
    removable=[r for r in rows if r["assignment"]=="removable"]
    total=sum(float(r["massKg"]) for r in removable)
    for r in rows:
        name=r["solid"]
        if name not in shapes:
            raise ValueError(f"Bumper mass row does not identify an installed solid: {name}")
        m=float(r["massKg"])
        # Allocate the removable kit allowance proportionally to its measured CAD
        # composition, allowing independent variations across the two sides.
        limits=(5.9*m/total,7.5*m/total) if r["assignment"]=="removable" else (m,m)
        add(name,[name],limits,homogeneous=True)


def loads_for(pose):
    report=json.loads((CAD/"review.json").read_text())[pose]
    shape_file=CAD/"poses"/(pose+".step")
    if hashlib.sha256(shape_file.read_bytes()).hexdigest()!=report["export_sha256"]:
        raise ValueError("Installed robot pose differs from the registered export")
    frames=json.loads((CAD/"part-frames.json").read_text())
    shapes=assembly.native.read_named(shape_file,[{"id":name} for name in frames])
    bounds={name:assembly.native.bounds(shape) for name,shape in shapes.items()}
    total_box=union(list(bounds.values()))
    used=set()
    loads=[]
    def add(name,keys,mass,homogeneous=False,position=None):
        keys=set(keys)
        if missing:=keys-bounds.keys():raise ValueError(f"Missing bodies {missing}")
        if repeated:=keys&used:raise ValueError(f"Double-counted bodies {repeated}")
        used.update(keys)
        ext=union([bounds[k] for k in keys]) if keys else total_box
        cg=ext
        if homogeneous:
            [key]=keys
            center=assembly.native.volume(shapes[key])[1]
            cg=(*center,*center)
        loads.append(Load(name,tuple(mass),tuple(position or cg),ext))

    new_bumpers="bumper-foam-front" in shapes
    for r in csv_rows(BASE_BOM):
        key=r["id"]
        if key in inspection.BASE_REPLACED or (new_bumpers and r["group"]=="bumper"): continue
        m=[float(r[k]) for k in ("minMassKg","maxMassKg")]
        if key=="unmodeled-allocation":
            add(key,[],m)
        else:
            add(key,[key],m,r["materialOrBasis"] in {"Al","steel","PE"})
    for r in csv_rows(GRIP_BOM):
        key=r["id"]
        add(key,[key],[float(r[k]) for k in ("lowMassKg","highMassKg")],r["description"].startswith("Original "))
    masses=inspection.ADDITIONS
    groups={
        "NP18-12 battery":["np18-battery"],
        "REV PDH + PH":["rev-pdh","rev-ph"],
        "radio, controller, RSL, main breaker":["rio2-envelope","vh109-reserve","rsl-reserve","main-breaker"],
        "four mechanism SPARK MAX units":[k for k in shapes if k.startswith("spark-")],
        "battery hold-down, panel, posts and brackets":[k for k in shapes if k.startswith(("battery-cheek","battery-hold","battery-side-stop","platform-post"))]+["distribution-bridge","front-electronics-platform"],
        "compressor, reservoir, valve, regulator, gauges and fittings":[k for k in shapes if k.startswith(("tank-","viair-","gauge-regulator-","regulated-pawl-"))],
        "long-range sensors and anchors":[k for k in shapes if k.startswith("qtrx-") or "harness-" in k],
        "observer hardware, pole, support and pads":[k for k in shapes if k.startswith("observer-")],
        "power cables, branches, moving service loops, leads and terminations":[],
    }
    for name,keys in groups.items():add(name,keys,masses[name])
    lift_ids=set(assembly.routing(assembly.native.MODEL["poses"][pose]["height"]))
    add("lift",lift_ids,masses["lift drive, ratchet, drums, lines, hardware and release"])
    if new_bumpers:
        add_bumpers(shapes,add)
    if used!=shapes.keys():raise ValueError(f"Unassigned CAD bodies: {sorted(shapes.keys()-used)}")
    return loads,report["export_sha256"]


def payload(pose):
    m=assembly.native.MODEL["motion"]
    p=assembly.native.MODEL["poses"][pose]
    r=m["pipeRadius_m"]*1000
    x=p["stroke"]+m["seatU_m"]*1000
    z=p["height"]-m["shelfTopBelowLift_m"]*1000+r
    half=m["pipeLength_m"]*500
    mass=m["pipeMass_kg"]
    return Load("seated pipe",(mass,mass),(x,0,z,x,0,z),(x-r,-half,z-r,x+r,half,z+r))


def drive_check(mass,inertia,radius,side_fraction,outer_fraction):
    if not 0 < side_fraction <= .5 or not 0 <= outer_fraction <= 1:
        raise ValueError("Invalid side or outer-axle load fraction")
    p={**DEFAULT,"massKg":mass,"cgOffsetM":radius,"inertiaKgM2":inertia,
       "minSideWeightFraction":side_fraction,"outerAxleWeightFraction":outer_fraction}
    result={}
    for kind,peak,acc in (("turn",CLOCK["maximumTurnRadS"]*DEFAULT["trackM"]/2,CLOCK["turnWheelAccelMps2"]),
                            ("drive",CLOCK["maximumWheelMps"],CLOCK["maximumWheelAccelMps2"])):
        checks=[demand(p,kind,v,a) for v in (0,peak) for a in (-acc,0,acc)]
        result[kind]={name:max(c[name] for c in checks) for name in ("phaseA","frictionRequired")}
    return result


def analyze(loads):
    axes=[[1,0,0],[0,1,0],[0,0,1]]
    cg=[[extremum(loads,a,False),extremum(loads,a,True)] for a in axes]
    radial_upper=max(math.hypot(x,y) for x in cg[0] for y in cg[1])/1000
    mass=[sum(r.mass[i] for r in loads) for i in (0,1)]
    inertia=sum(r.mass[1]*sum(max(abs(r.extent[i]),abs(r.extent[i+3]))**2 for i in (0,1))
                for r in loads)/1e6
    half_track=DEFAULT["trackM"]*500
    half_span=assembly.drive_model.CENTERS
    # The registered route alternates pure translation with in-place turns. For spin,
    # tangential and radial accelerations are orthogonal, with radius measured from yaw axis.
    spin_factor=math.hypot(2*CLOCK["turnWheelAccelMps2"]/DEFAULT["trackM"],CLOCK["maximumTurnRadS"]**2)
    spin_accel=radial_upper*spin_factor
    result={}
    for name,axis,support,acc in (("straight",0,half_span,CLOCK["maximumWheelAccelMps2"]),("turn-fore-aft",0,half_span,spin_accel),
                                 ("turn-side",1,half_track,spin_accel)):
        offsets=[]
        for direction in [-1,1]:
            coeff=[0,0,acc/G]
            coeff[axis]=direction
            offsets.append(extremum(loads,coeff,True))
        offset=max(offsets)
        result[name]={"acceleration_mps2":acc,"load_line_margin_mm":support-offset,
                      "opposite_edge_equivalent_load_fraction":(support-offset)/(2*support)}
    side=result["turn-side"]["opposite_edge_equivalent_load_fraction"]
    # Evaluate the previously specified balance limits against this actual assembly's
    # inertia bound. A high-mass inertia divided by the low mass can overstate traction
    # demand; evaluating both mass endpoints conservatively covers the entire interval.
    def cases(radius,min_side,outer):
        low=drive_check(mass[0],inertia,radius,min_side,outer)
        high=drive_check(mass[1],inertia,radius,min_side,outer)
        return {k:{v:max(low[k][v],high[k][v]) for v in low[k]} for k in low}
    drive_screen=cases(radial_upper,side,1.0) if side>0 else None
    radius_limit=DEFAULT["cgOffsetM"]
    side_limit=DEFAULT["minSideWeightFraction"]
    outer_limit=DEFAULT["outerAxleWeightFraction"]
    existing=cases(radius_limit,side_limit,outer_limit)
    static_min=side_limit+(cg[2][1]/1000)*radius_limit*spin_factor/(G*DEFAULT["trackM"])
    return {"mass_kg":mass,"cg_mm":cg,"radial_cg_upper_m":radial_upper,"yaw_inertia_upper_kg_m2":inertia,
            "load_transfer":result,"drive_demand_at_profile_extrema":drive_screen,
            "prior_balance_targets":{"requires_radius_m_at_most":radius_limit,"requires_dynamic_side_fraction_at_least":side_limit,
              "requires_static_side_fraction_at_least":static_min,"requires_outer_axle_fraction_at_most":outer_limit,
              "drive_demand_at_profile_extrema":existing}}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    inputs=[BASE_BOM,GRIP_BOM,CAD/"review.json",CAD/"part-frames.json",
            DESIGN/"electrical-inspection/inspection.py",DESIGN/"drivetrain/route_check.py",
            DESIGN/"drivetrain/drive_parts.py",DESIGN/"station-l1/assembly.json",
            ROOT/"mission/67kg-field-clock.json", Path(__file__).resolve()]
    if BUMPER_BOM.exists():inputs.append(BUMPER_BOM)
    review={"schema":"whole-robot-load-transfer/1","inputs_sha256":{
        p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},"poses":{}}
    for pose in assembly.native.MODEL["poses"]:
        loads,sha=loads_for(pose)
        review["poses"][pose]={"step_sha256":sha,"empty":analyze(loads),"seated":analyze(loads+[payload(pose)])}
    args.output.write_text(json.dumps(review,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__":main()
