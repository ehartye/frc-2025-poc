"""Screen seam registration, screw-bottom clearance and loaded crown exposure."""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt,gp_Trsf

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/split-drop-native'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_bytes())

def main(assets,plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import common
    from showcase_contacts import load_solids
    reg=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    if sha(ROOT/reg['file'])!=reg['sha256']:raise ValueError('Changed engineer authority')
    raw=read(OUT/'aluminum-hub-fit.json')
    hubfile=assets/'REV-21-3002.STEP';screwfile=assets/'REV-29-2943.STEP'
    if sha(hubfile)!=raw['sourceWheelSha256'] or sha(screwfile)!='4c253e1b8a23613182cf6040a59f547948c3e6d3b0844ac81d0b3f365b8904cb':raise ValueError('Supplier change')
    hub=load_solids(hubfile)[0]['REV-21-3002'];screw=next(iter(load_solids(screwfile)[0].values()))
    seats={}
    for item in raw['radialHoles']:
        x,y,z=item['point'];r=math.hypot(y,z)
        if y<0:seats[(round(x,4),round(z/r,6))]=(x,y/r,z/r)
    if len(seats)!=4:raise ValueError('Expected four hub tap axes at the chosen seam')
    bed,free,flange=30.480,38.354,33.655
    width=21.59;outerHeadR=4.5847;headHeight=7.6327-5.0673
    phase=math.acos(-next(iter(seats.values()))[1])
    stock=[]
    for scale in (1.,1.03):
        length=207.96*scale;rn=length/(2*math.pi)
        # Prefer actual printed and trimmed widths over assuming isotropic scaling.
        holes=[dict(scale=scale,printedAxialPitchMm=9.6*scale,hubAxialPitchMm=9.525,
                    axialOffsetOneEndMm=(9.6*scale-9.525)/2,arcOffsetOneEndMm=7.4*scale-rn*phase)]
        stock.append(dict(scale=scale,lengthMm=length,printedWidthMm=width*scale,freeBedWidthMm=22.225,
                          widthReserveMm=22.225-width*scale,neutralRadiusMm=rn,innerFiberStrain=bed/rn-1,
                          outerFiberStrain=free/rn-1,holes=holes))
    fits=[]
    for compression in (0,.8,1.5):
        # Bound the circular crown with a cylinder, retaining 0.25 mm of carpet clearance.
        loadedR=free-compression
        recess=free+headHeight-math.sqrt((loadedR-.25)**2-outerHeadR**2)
        for depth in (0,recess):
            mounting=free-depth
            for _,(x,dy,dz) in seats.items():
                shift=mounting-5.0673
                tr=gp_Trsf();tr.SetValues(1,0,0,x,0,dz,dy,dy*shift,0,-dy,dz,dz*shift)
                placed=BRepBuilderAPI_Transform(screw,tr,False).Shape()
                tip=mounting-12.7
                witness=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x,dy*tip,dz*tip),gp_Dir(0,dy,dz)),1.5,bed-tip).Shape()
                fits.append(dict(compressionMm=compression,recessMm=depth,axisAndAxialX=[x,dy,dz],
                    crownBoundMm=math.hypot(mounting+headHeight,outerHeadR),loadedRadiusMm=loadedR,
                    remainingTreadAtUnderheadMm=mounting-bed,requestedTapEngagementMm=bed-tip,
                    actualScrewToHubCommonMm3=common(placed,hub),pilotBottomCommonMm3=common(witness,hub),
                    flangeToCarpetMm=loadedR-flange))
    result=dict(authority=reg,sourceWheel=sha(hubfile),sourceScrew=sha(screwfile),drawing=sha(assets/'REV-21-3002-DR.pdf'),
        template=sha(assets/'REV-21-2996-PRINTABLE-TREAD-PATTERN.pdf'),fourChosenSeamHoles=list(seats.values()),samples=fits,flatStock=stock,
        basis='Natural belt neutral axis and compressive force curve are unmeasured. Nominal 0.310 inch fabric-reinforced belt and exact hub/fastener STEP. The recessed tool envelope is a proposed compression/trim test, not approval to mill through load-bearing plies. Positive major/minor stock overlap is expected only around the tapped form; an additional 3 mm-diameter probe catches unintended solid below the bore.',
        unresolved=['Mounting torque, clamp load and tear-out across residual reinforcement','Measured pan/compaction rebound and corner-specific tread compression, slip, radius/calibration','Full native wheel reopen with four mounted screws and fork/chassis service'])
    (OUT/'tread-retention-fit.json').write_text(json.dumps(result,indent=2)+'\n')
    print([(round(s['compressionMm'],2),round(s['recessMm'],3),round(s['remainingTreadAtUnderheadMm'],3),
        round(s['requestedTapEngagementMm'],3),round(s['pilotBottomCommonMm3'],4)) for s in fits[::4]],flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('assets',type=Path);parser.add_argument('plugin',type=Path)
    a=parser.parse_args();main(a.assets,a.plugin)
