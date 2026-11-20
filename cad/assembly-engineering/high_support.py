"""Test top-plate support and unconstrained body pitch in registered field coordinates."""
import argparse
import itertools
import json
import math
from pathlib import Path
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf
from installed_audit import HERE, ROOT, sha, bounds, box_gap
from attachments import common, base
from clearance import hanging_inventory, move, read_assembly
from lift import CG_BOUNDS,MASS,G

def turn(shape,about,axis,radians):
    tr=gp_Trsf();tr.SetRotation(gp_Ax1(gp_Pnt(*about),gp_Dir(*axis)),radians)
    return shape.Moved(TopLoc_Location(tr))
def run(field,centers=(300,)):
    stations,shape_table=hanging_inventory(field)
    chassis={k:s for g,k,s in read_assembly(ROOT/"cad/swerve-captive/public-robot.step") if g!="Demonstration CORAL"}
    receipt=json.loads((HERE/"interfaces.json").read_bytes())
    overrides=base.named(HERE/"station-and-jack-interfaces.step")
    chassis.update({k:s for k,s in overrides.items() if not k.endswith("rail saddle")})
    for k in receipt["fixedHeight"]["removed"]:chassis.pop(k)
    boxes={k:bounds(s) for k,s in chassis.items()}
    witness=["FL-display-tread","FR-display-tread","RL-display-tread","RR-display-tread","bumper-cover-front-red","bumper-cover-rear-red"]
    answer={}
    for side,key,sign in (("red","NAUO260",-1),("blue","NAUO263",1)):
        station=stations[key];fx,fz=station["axis_mm"]
        answer[side]={}
        for cx in centers:
            tr=gp_Trsf();tr.SetValues(sign,0,0,cx-sign*fx,0,0,-sign,sign*fz,0,1,0,0)
            cage=shape_table[station["cage"]].Moved(TopLoc_Location(tr))
            result={}
            for h in (787.4,1340):
                results=[]
                for cg in itertools.product(*CG_BOUNDS):
                    p=319.4
                    # The pair of symmetric pads on the raised top-plate concept has
                    # a resultant along the cage axis. The chassis hangs on a gimbal.
                    dx,dy=1000*cg[0]-cx,1000*cg[1]
                    radius=math.hypot(dx,dy);nx,ny=-dx/radius,-dy/radius
                    angle=math.atan2(radius,h-p-1000*cg[2])
                    about=(cx,0,h);ax=(ny,-nx,0)
                    upright_in_robot=move(turn(cage,about,ax,-angle),(0,0,-p))
                    cb=bounds(upright_in_robot)
                    overlaps={}
                    for k,s in chassis.items():
                        if box_gap(cb,boxes[k])>.001:continue
                        ov=common(s,upright_in_robot)
                        if ov>.01:overlaps[k]=round(ov,2)
                    actual={k:round(bounds(turn(move(chassis[k],(0,0,p)),about,ax,angle))[2],4) for k in witness}
                    results.append(dict(CGm=cg,freeAngleDeg=math.degrees(angle),lowestObservedMm=min(actual.values()),lowestPart=min(actual,key=actual.get),
                                chassisIntersections=overlaps))
                result[str(h)]=results
            # Four separated top pads. Positive original field material on a 0.1 mm
            # inward press obtains the real footprint rather than a projected box.
            pads={}
            for xoff in (-35,35):
                for yoff in (-78,78):
                    x=cx+xoff
                    gauge=base.block((x-9,yoff-5,1371.60,x+9,yoff+5,1376.60))
                    pads[f"{xoff},{yoff}"]={"nominalOverlapMm3":common(cage,gauge),
                         "contactAreaMm2At01mm":common(cage,move(gauge,(0,0,-.1)))/.1}
            result["topPads"]=pads
            answer[side][str(cx)]=result
    return dict(fieldSha256=sha(field),chassisStepSha256=sha(ROOT/"cad/swerve-captive/public-robot.step"),interfaceSha256=receipt["outputSha256"],
         maxConstantVerticalLoadPerPadN=MASS*G/4,designOnePadN=MASS*G*1.5*.5,
         topBearingPlaneMm=1371.60,hypotheticalTopGimbalHeightMm=1340,
         finalUpwardStrokeMm=319.4,results=answer,
         limitations="Comparison of final equilibrium. The top design needs an articulated yoke, entry over the upper perimeter, confirmed symmetric support and a continuous collision-free approach and lift. It does not load the top eye, screw or rigging by design. An exact unplanned chassis-cage or carpet contact rejects the passage.")
if __name__=="__main__":
    cli=argparse.ArgumentParser();cli.add_argument("field",type=Path)
    cli.add_argument("--centers",type=int,nargs="+",default=[300,340,365,390])
    args=cli.parse_args()
    (HERE/"high-support.json").write_bytes((json.dumps(run(args.field,args.centers),indent=2)+"\n").encode())

