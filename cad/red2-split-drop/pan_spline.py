"""Measure the actual male carrier and female output splines at the seated depth."""
import math,sys,json
from pathlib import Path
import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.GCPnts import GCPnts_QuasiUniformDeflection
from OCP.gp import gp_Pln,gp_Ax3,gp_Pnt,gp_Dir
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopoDS import TopoDS
from output_mount import ROOT,OUT,read,digest,place
D=ROOT/'artifacts/red2-integrated'
def profile(s,z,angles,inner):
    op=BRepAlgoAPI_Section(s,gp_Pln(gp_Pnt(0,0,z),gp_Dir(0,0,1)),False);op.Build()
    edges=TopExp_Explorer(op.Shape(),TopAbs_EDGE);segments=[]
    while edges.More():
        a=BRepAdaptor_Curve(TopoDS.Edge(edges.Current()));edges.Next()
        interp=GCPnts_QuasiUniformDeflection(a,.0005)
        points=np.array([interp.Value(i).Coord()[:2] for i in range(1,interp.NbPoints()+1)])
        segments.extend(zip(points,points[1:]))
    a,b=np.array(segments).transpose(1,0,2);du=b-a;num=a[:,0]*du[:,1]-a[:,1]*du[:,0]
    result=[]
    for phi in angles:
        ux,uy=math.cos(phi),math.sin(phi);den=ux*du[:,1]-uy*du[:,0]
        good=abs(den)>1e-10
        rr=num[good]/den[good];t=(a[good,0]*uy-a[good,1]*ux)/den[good]
        candidates=rr[(t>=-1e-8)&(t<=1+1e-8)&(rr>1.)&(rr<8.0)]
        if len(candidates)==0:raise ValueError('Unresolved section at '+str([z,phi]))
        result.append(min(candidates) if inner else max(candidates))
    return np.array(result)
def run(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from showcase_contacts import load_solids
    from mechanical_motion import common,bounds
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    d=read(D/'pan-drive-registration.json');s=next(r for r in d['components'] if r['id']=='REV-41-1601')
    mode=sys.argv[2] if len(sys.argv)>2 else 'retained-donor'
    if mode not in ('retained-donor','standalone-issue'):raise ValueError('Unknown manufacturer issue')
    if mode=='standalone-issue':
        x=next(r for r in d['components'] if r['id']=='REV-41-1603')
        frame=np.array(x['frameMm']);frame[2,3]-=20
        issue=D/'alternative-supplier/REV-41-1601.STEP'
        s=dict(id='REV-41-1601 standalone catalog',source=issue.relative_to(ROOT).as_posix(),sourceSha256=digest(issue),frameMm=frame.tolist(),sourceBasis=x['sourceBasis'])
    donor=load_solids(ROOT/s['source'])[0]
    def vol(s):
        p=GProp_GProps();BRepGProp.VolumeProperties_s(s,p);return p.Mass()
    placed=[place(shape,np.array(s['frameMm'])) for shape in donor.values()]
    male,=[shape for shape in placed if abs(bounds(shape)[2]-4.375)<.005]
    female=load_solids(OUT/'supplier/REV-41-1615.STEP')[0]['REV-41-1611_Through Hole']
    n=3600;angles=np.arange(n)*2*math.pi/n
    slices=[]
    for z in (6.5,8.0,10.0):
        small=profile(male,z,angles,False);big=profile(female,z,angles,True)
        slices.append((small,big))
    def gap(shift):return min(min(np.roll(b,shift)-a) for a,b in slices)
    trials=sorted([(round(gap(x),8),x) for x in range(-140,141)],key=lambda v:(-v[0],abs(v[1])))
    result=[]
    for shift in dict.fromkeys((0,trials[0][1],trials[0][1]-1,trials[0][1]+1)):
        angle=360*shift/n;q=math.radians(angle);c=math.cos(q);ss=math.sin(q)
        posed=place(female,np.array([[c,-ss,0,0],[ss,c,0,0],[0,0,1,0]]))
        result.append(dict(femaleRelativeAngleDeg=angle,sectionRadialGapMm=gap(shift),fullSolidCommonMm3=common(male,posed)))
    data=dict(source=s,authority=d['authority'],outputSource=digest(OUT/'supplier/REV-41-1615.STEP'),axialSectionMm=[6.5,8,10],angularStepDeg=.1,results=result)
    data['driveInstallationPlanSha256']=digest(D/'pan-drive-registration.json')
    data['sourceIssueMode']=mode
    data['shaftFrameToRobotXYZmm']=[[1,0,0,205],[0,0,1,216],[0,-1,0,580]]
    data['axialSeat']=dict(femaleOutputInputMountPlaneZmm=11.25,cartridgeMatingPlaneZmm=11.25,sourceMountedPairProjectionMm=[float(bounds(male)[2]),float(bounds(female)[5])])
    data['factoryInterface']='REV documents the 13T M0.55 carrier and matching V2 input sliding fit. Exact tolerance and ratio-specific tooth build are not established by these neutral files.'
    data['sourceURL']='https://docs.revrobotics.com/rev-crossover-products/ultraplanetary/features'
    (D/('pan-spline-'+mode+'.json')).write_text(json.dumps(data,indent=2)+'\n')
    print(result)
if __name__=='__main__':run(Path(sys.argv[1]))
