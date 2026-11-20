"""Independently fit sourced replacement tab screws/nuts and service gauges to the actual selected module and robot."""
import argparse,hashlib,json,math,sys
from pathlib import Path
import numpy as np
from OCP.BRep import BRep_Builder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common,BRepAlgoAPI_Cut
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon,BRepBuilderAPI_MakeFace
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.BRepTools import BRepTools
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.TopoDS import TopoDS_Shape
from OCP.gp import gp_Ax2,gp_Pnt,gp_Dir,gp_Trsf,gp_Vec

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('project',type=Path,help='season checkout with issued servicing interface STEP')
parser.add_argument('supplier_cache',type=Path,help='authorized packaging cache including the original vendor STEP and native BReps')
parser.add_argument('output',type=Path,help='write the numerical receipt here')
parser.add_argument('--overlay',type=Path,help='an archived station-and-jack interfaces STEP when working from a later checkout')
parser.add_argument('--overlay-register',type=Path,help='the matching archived interfaces.json')
cli=parser.parse_args()
root=cli.project.resolve()
assets=cli.supplier_cache.resolve()
eng=root/'cad/assembly-engineering'
pkg=root/'experiments/swerve-drive/packaging'
sys.path[:0]=[str(eng),str(pkg)]
from installed_audit import reopen
from attachments import base
from module_placement import SITES,basis
dest=Path(__file__).resolve().parent

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def cast(x,y,z0,z1,r):return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x,y,z0),gp_Dir(0,0,1)),r,z1-z0).Shape()
def tube(x,y,z0,z1,ro,ri):return BRepAlgoAPI_Cut(cast(x,y,z0,z1,ro),cast(x,y,z0-1,z1+1,ri)).Shape()
def vol(s):
    p=GProp_GProps();BRepGProp.VolumeProperties_s(s,p);return p.Mass()
def box(s):
    b=Bnd_Box();BRepBndLib.AddOptimal_s(s,b,False,False)
    if b.IsVoid():return None
    return [getattr(p,a)() for p in (b.CornerMin(),b.CornerMax()) for a in ('X','Y','Z')]
def placed(s,site):
    m=np.column_stack((basis(site),[*SITES[site],38.1]))
    t=gp_Trsf();t.SetValues(*m.flatten());return BRepBuilderAPI_Transform(s,t,False).Shape()
def common(a,b):return BRepAlgoAPI_Common(a,b).Shape()
def swing(x,y,z,site):
    # Includes a 28 mm head around the tool axis plus a 120 mm radial lever.
    # At either extreme a 20 mm wide lever rotated ±15 degrees fits in the trapezoid.
    out=np.array([x-SITES[site][0],y-SITES[site][1]]);out=out/np.linalg.norm(out)
    lat=np.array([-out[1],out[0]])
    wire=BRepBuilderAPI_MakePolygon()
    for a,b in ((-14,-14),(0,-14),(130,-49),(130,49),(0,14),(-14,14)):
        pt=np.array([x,y])+out*a+lat*b
        wire.Add(gp_Pnt(*pt,z))
    wire.Close()
    return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(wire.Wire()).Face(),gp_Vec(0,0,16)).Shape()
def hits(s,obs,omit=()):
    b=box(s);found={}
    for name,(part,pb) in obs.items():
        if name in omit or any(b[i+3]<pb[i]-.00001 or b[i]>pb[i+3]+.00001 for i in range(3)):continue
        v=vol(common(s,part))
        if v>.001:found[name]=round(v,5)
    return found

extern=sha(assets/'REV-21-3005.STEP')
assert extern==(assets/'vendor-native/source.sha256').read_text().strip()
solid_ids=json.loads((assets/'original.json').read_bytes())
env=json.loads((pkg/'envelopes.json').read_bytes())
raw={}
for n in solid_ids:
    key=n.split('/',1)[1].replace('/','_').replace(':','-')
    s=TopoDS_Shape();assert BRepTools.Read_s(s,str(assets/'vendor-native'/(key+'.brep')),BRep_Builder())
    if key in env['movingRadiusMm']:
        b=box(s);s=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0,b[1],0),gp_Dir(0,1,0)),env['movingRadiusMm'][key],b[4]-b[1]).Shape()
    raw['supplier/'+key]=s
assert len(raw)==72
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
raw['supplier/V11 bridge reserve']=BRepPrimAPI_MakeBox(gp_Pnt(-15.875,40.95,-28.58),gp_Pnt(15.875,71.05,28.58)).Shape()
for key,z0,z1,rad in [('shaft-key',41,70.8,9.6),('V2 wheel',-38.1,38.1,math.hypot(38.1,12.8))]:
    raw['supplier/'+key]=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0,z0,0),gp_Dir(0,1,0)),rad,z1-z0).Shape()
kit=cli.overlay or eng/'station-and-jack-interfaces.step'
kitpin=sha(kit);ints=json.loads((cli.overlay_register or eng/'interfaces.json').read_bytes());assert kitpin==ints['outputSha256']
src={k:s for k,_,_,representation,_,s in reopen() if representation!='public-module-representation'}
src.update(base.named(kit))
for k in ints['fixedHeight']['removed']:src.pop(k,None)
frame={k:(s,box(s)) for k,s in src.items()}
print('opened',len(frame),flush=True)
report={}
for site in SITES:
    mod={n:placed(s,site) for n,s in raw.items()}
    mo={n:(s,box(s)) for n,s in mod.items()}
    obs=frame|mo
    for pos in ('A','B'):
        for pitch in [57.15,82.55]:
            short=f'{site}-{pos}';station=short+'-'+str(pitch)
            screw=short+'-tab-screw-'+str(pitch);nut=short+'-tab-locknut-'+str(pitch)
            bb=frame[screw][1];x=(bb[0]+bb[3])/2;y=(bb[1]+bb[4])/2
            ann=tube(x,y,1,191,3.7,2.85)
            low=box(common(ann,mod['supplier/9-REV-21-3005-P01']))
            high=box(common(ann,mod['supplier/10-REV-21-3005-P02']))
            sad=box(common(ann,frame[short+'-tab-saddle'][0]))
            lo,hi=low[2],high[5];axial=hi-lo
            # Nut as a full circumscribed cylinder (all possible installed hex phases);
            # bolt body as full major diameter for interference. Actual thread voids omitted.
            headHeight=.190*25.4; headR=.312*25.4/2; halfAF=.376*25.4/2
            new={'head':cast(x,y,lo-headHeight,lo,headR),
                 'shankThroughReceiptLengthMax':cast(x,y,lo,lo+70.30,4.826/2),
                 'allClockingNut':tube(x,y,hi,hi+.249*25.4,halfAF/math.cos(math.pi/6),4.826/2)}
            # Bench stage uses two planar saddles and the four tab screws before cartridge installation.
            detached=mo|{k:v for k,v in frame.items() if k.startswith(site+'-') and ('tab-' in k)}
            tools={'keyFromBelow':cast(x,y,lo-headHeight-75,lo-headHeight,3.2),
                   'socketFromAbove':cast(x,y,hi,hi+45,7.5),
                   'upperHandleSwing':swing(x,y,hi+45,site),
                   'lowerHandleSwing':swing(x,y,lo-headHeight-75-16,site),
                   'screwExtractionFromBelow':cast(x,y,lo-75-headHeight,lo,headR)}
            new_hits={t:hits(s,obs,{screw,nut}) for t,s in new.items()}
            offhits={t:hits(s,detached,{screw,nut}) for t,s in tools.items()}
            onhits={t:hits(s,obs,{screw,nut}) for t,s in tools.items()}
            # Demonstrate a real bearing seat on each plate by pressing a washer-shaped gauge 0.01 mm.
            lower_area=vol(common(tube(x,y,lo-.01,lo+.01,.298*25.4/2,2.413),mod['supplier/9-REV-21-3005-P01']))/.01
            upper_area=vol(common(tube(x,y,hi-.01,hi+.01,.367*25.4/2,2.413),mod['supplier/10-REV-21-3005-P02']))/.01
            report[station]=dict(x=x,y=y,plateIntervals=[[low[2],low[5]],[high[2],hi]],saddle=[sad[2],sad[5]],
                  gripMm=axial,installedShimGapsMm=[sad[2]-low[5],high[2]-sad[5]],nominalWorstNutProtrusionMm=lo+69.85-hi-.249*25.4,
                  rejectedShort2p5inProtrusionMm=63.5-axial-.249*25.4,proposedAcceptedStackProtrusionMinMm=69.0-59.0-.249*25.4,
                  minimumHeadSeatAreaMm2=lower_area,inscribedNutSeatAreaMm2=upper_area,articleHits=new_hits,detachedToolHits=offhits,installedToolHits=onhits)
    print(site,'done',flush=True)
outfile=cli.output.resolve()
summary=dict(sourceModule=extern,sourcePlan=sha(pkg/'frame.json'),frameSha=sha(pkg/'swerve-frame.step'),baseStepSha=sha(root/'cad/swerve-captive/public-robot.step'),interfaceStepSha=kitpin,
             supplierScrew='Bolt Depot 8232',supplierNut='Bolt Depot 2623',stations=report)
outfile.write_text(json.dumps(summary,indent=2)+'\n')
assert len(report)==16
assert not any(any(station['articleHits'].values()) or any(station['detachedToolHits'].values()) for station in report.values())
print('active clashes',[(k,v['articleHits']) for k,v in report.items() if any(v['articleHits'].values())],flush=True)
print('detached tool clashes',[(k,v['detachedToolHits']) for k,v in report.items() if any(v['detachedToolHits'].values())],flush=True)
print('sample',next(iter(report.values())))
