"""Register the six real kit screws and its original NEO motor plate pair."""
import math,json,sys
from pathlib import Path
import numpy as np
from OCP.BRep import BRep_Builder
from OCP.BRepTools import BRepTools
from OCP.TopoDS import TopoDS_Shape
from OCP.STEPControl import STEPControl_Writer,STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from output_mount import ROOT,digest,read,place
D=ROOT/'artifacts/red2-integrated'
def run(project,plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from showcase_contacts import load_solids
    from mechanical_motion import bounds
    source=read(D/'pan-drive-registration.json')
    assert digest(ROOT/source['authority']['file'])==source['authority']['sha256']
    directory=project/'artifacts/swerve-package'
    assert digest(directory/'REV-21-3005.STEP')==read(ROOT/'artifacts/split-drop-native/jaw-drive-source-stack.json')['sourceModule']
    article=D/'alternative-supplier/M3X40.STEP'
    screw=next(iter(load_solids(article)[0].values()))
    ret=[]
    for i in range(6):
        a=math.radians(60*i);radius=32/math.sqrt(3)
        # Use the actual source's recess centres (+/-16 mm on Y) at
        # 18.4752 mm, inside the drawing's rounded 37 mm circle.
        t=np.array([[1,0,0,radius*math.cos(a)],[0,-1,0,radius*math.sin(a)],[0,0,-1,2]])
        ret.append(dict(id='REV UP three-stage case screw '+str(i+1),source=article.relative_to(ROOT).as_posix(),sourceSha256=digest(article),frameMm=t.tolist(),installedShaftFrameBoundsMm=bounds(place(screw,t)),
            basis='The native M3x40 head shoulder is z=-3, positioned at the physical positive z=5 output recess. The fastener reaches z=45 into the 41.25 to 46.25 mm motor plate. Verify thread/seat contact and full driver entrance.'))
    mot=next(x for x in source['components'] if x['id']=='REV-21-1651')
    for i in (6,7):
        article=directory/'vendor-native'/(str(i)+'-M3 BHCS SS_08mm Long.brep')
        s=TopoDS_Shape();assert BRepTools.Read_s(s,str(article),BRep_Builder())
        out=D/'pan-supplier'/('motor-plate-bhcs-08-original-'+str(i)+'.step')
        if not out.exists():
            writer=STEPControl_Writer();writer.Transfer(s,STEPControl_AsIs);assert writer.Write(str(out))==IFSelect_RetDone
        ret.append(dict(id='M3x8 NEO plate original '+str(i),source=out.relative_to(ROOT).as_posix(),sourceSha256=digest(out),frameMm=mot['frameMm'],originalNativeBRepSha256=digest(article),installedShaftFrameBoundsMm=bounds(place(s,np.array(mot['frameMm']))),basis='Unchanged parent transform shared with source-assembled motor and motor plate; measured motor thread engagement still required.'))
    result=dict(sourceInstallation=digest(D/'pan-drive-registration.json'),authority=source['authority'],factoryAssembly='https://docs.revrobotics.com/rev-crossover-products/ultraplanetary/assembly-instructions',preparedForNextNativeIssue=ret)
    (D/'pan-assembly-fasteners.json').write_text(json.dumps(result,indent=2)+'\n')
    print([(r['id'],r['installedShaftFrameBoundsMm'][2],r['installedShaftFrameBoundsMm'][5]) for r in ret])
if __name__=='__main__':run(Path(sys.argv[1]),Path(sys.argv[2]))
