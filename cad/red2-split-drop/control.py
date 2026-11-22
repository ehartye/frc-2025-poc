"""Bind the concept's first obstruction to native world coordinates and the source pipe."""
import hashlib
import itertools
import json
import math
from pathlib import Path
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox,BRepPrimAPI_MakeCylinder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common,BRepAlgoAPI_Cut
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt

ROOT=Path(__file__).resolve().parents[2]
ISSUE=ROOT/'artifacts/split-drop-native'
def ref(p):return {'file':p.relative_to(ROOT).as_posix(),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
def data(p):return json.loads(p.read_bytes())
def volume(q):
    prop=GProp_GProps();BRepGProp.VolumeProperties_s(q,prop);return prop.Mass()

def main():
    register=ROOT/'cad/fusion-authority.json'
    archive=ISSUE/'red2-split-drop-fixture-r1.f3d';readback=ISSUE/'pose-01.json'
    r=data(readback)
    if r['sha256']!=ref(archive)['sha256'] or r['authority']['archiveSha256']!=data(register)['source']['archive']['sha256']:raise ValueError('Changed source or fixture')
    open_pose=next(x for x in r['native']['states'] if x['pose']=='open')
    name='Longitudinal carriage:1/Front transverse slide surface'
    b=open_pose['bodies'][name]
    expected=[576,-215,510,604,215,518]
    if max(abs(i-j) for i,j in zip(expected,b))>.01:raise ValueError('Changed R1 crosspiece')
    route=ROOT/'experiments/red2-successor/executions.json'
    matched=data(route)['runs']['split-drop'];controls={}
    for k,c in matched.items():
        sx,sz=c['capture']['finalCenterMm'];x=sx+404
        t=math.sqrt(2*(sz-570)/9806.65)
        axis=gp_Ax2(gp_Pnt(x,-150.8125,570),gp_Dir(0,1,0))
        outer=BRepPrimAPI_MakeCylinder(axis,57.15,301.625).Shape()
        inner=BRepPrimAPI_MakeCylinder(axis,50.8,301.625).Shape()
        pipe=BRepAlgoAPI_Cut(outer,inner).Shape()
        cases={}
        for label,forward in [('r1',0),('proposed-r2',106)]:
            a=[b[0]+forward,b[1],b[2],b[3]+forward,b[4],b[5]]
            bridge=BRepPrimAPI_MakeBox(gp_Pnt(*a[:3]),gp_Pnt(*a[3:])).Shape()
            op=BRepAlgoAPI_Common(pipe,bridge)
            if not op.IsDone():raise ValueError('Unresolved overlap')
            cases[label]=dict(boundsMm=a,solidOverlapMm3=volume(op.Shape()),horizontalEdgeGapMm=a[0]-x-57.15)
        controls[k]=dict(source='coral-A before independent scoring',originMm=[x,0,sz],axis=[0,1,0],targetCenterMm=[x,0,570],timeAfterFreeReleaseS=t,
                         outerRadiusMm=57.15,innerRadiusMm=50.8,lengthMm=301.625,firstTopContactDropMm=sz-57.15-b[5],cases=cases)
    peak=max(x['capture']['peakNormalN'] for x in matched.values())
    x=max(x['capture']['finalCenterMm'][0]for x in matched.values())
    a,bc=-43,290
    front=peak*(x-a)/(bc-a)
    mounted=peak*(x+404-144)/(330-144)
    report=dict(authority=ref(register),fixture=ref(archive),nativeReadback=ref(readback),nativePose='open',
                affirmativeEmptyChecks=[ref(ISSUE/f) for f in ('home-native.json','present-native.json','clear-native.json','jam-left-native.json')],
                historicGateControl=ref(ROOT/'experiments/red2-successor/straight-gate-field-control.json'),sourceMatched=ref(route),
                geometry='R1 native rectangular extrusion verified by its readback bounds, mathematical annular pipe; R2 here is a proposed X+106 mm bridge offset pending native rebuild.',
                member=name,scenarios=controls,reactions=dict(peakCaptureN=peak,comparison='Apply the measured concept impact conservatively even at maximum extension, static force balance only.',
                jawFootCentersMm=[a,bc],combinedFrontN=front,combinedRearN=peak-front,oneSideJamFrontN=front,oneSideJamRearN=peak-front,
                fixedSupportExtremeLocationsMm=[144,330],frontDownN=mounted,rearUpliftN=mounted-peak),
                limits='No real hardware load rating. The proposed R2 corridor value is bound to the crosspiece only. Final integrated transfer needs the actual path, shaped support release, inertia, field target, reset, native candidate and every fixed/powered body.')
    (ISSUE/'r1-gamepiece-control.json').write_text(json.dumps(report,indent=2))
    print(controls,report['reactions'])
if __name__=='__main__':main()
