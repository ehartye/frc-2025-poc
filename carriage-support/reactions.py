"""Resolve complete signed, simultaneous shelf inputs at all four fixed carriage lands."""
import argparse
from pathlib import Path
import numpy as np
from support import ROOT,HERE,j,REAR,FRONT,Y_FRONT,Y_REAR,BOLTS,blank
import importlib.util
spec=importlib.util.spec_from_file_location("mast_loads",ROOT/"mast-attachment/loads.py")
mast_loads=importlib.util.module_from_spec(spec)
spec.loader.exec_module(mast_loads)
group=mast_loads.group
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer

PITCH_ARM=(FRONT-REAR)/1000
E=70_000.0  # MPa, stiffness screening assumption for the aluminum plate
PRELOAD=1500.
# Rear and forward installed centers in mm. Conservative calculations put all
# horizontal loads through these fasteners; the original locks receive no credit.
P=np.array([(FRONT,np.mean(Y_FRONT),433.),(FRONT,-np.mean(Y_FRONT),433.),
            (REAR,Y_REAR,412.),(REAR,-Y_REAR,412.)])
O=np.array([REAR,0.,422.5])
FOOT=np.array([-45.,0.,154.])

def section(shape,x):
    dx=.10
    cut=BRepAlgoAPI_Common(shape,j.box(x-dx/2,x+dx/2,170,211,295,440)).Shape()
    props=GProp_GProps();BRepGProp.VolumeProperties_s(cut,props)
    n=props.Mass()
    if n<=0:raise ValueError(x)
    c=props.CentreOfMass()
    mat=props.MatrixOfInertia()
    yy=mat.Value(3,3)/dx-n*dx/12 # integral y², corrected for slab thickness
    zz=mat.Value(2,2)/dx-n*dx/12
    yz=-mat.Value(2,3)/dx
    b=j.cad.bounds(cut);center=np.array([x,c.Y(),c.Z()])
    children=[]
    it=TopExp_Explorer(cut,TopAbs_SOLID)
    while it.More():
        shape=it.Current();pr=GProp_GProps();BRepGProp.VolumeProperties_s(shape,pr)
        mass=pr.Mass();mat2=pr.MatrixOfInertia();cen=pr.CentreOfMass()
        covariance=np.array([[mat2.Value(3,3),-mat2.Value(2,3)],[-mat2.Value(2,3),mat2.Value(2,2)]])/dx
        covariance-=np.eye(2)*mass*dx/12
        children.append((cen.Z(),mass/dx,[cen.Y()-c.Y(),cen.Z()-c.Z()],float(np.linalg.eigvalsh(covariance)[0])))
        it.Next()
    children.sort(key=lambda a:a[0])
    return dict(children=children,area=n/dx,center=center,inv=np.linalg.inv([[yy,yz],[yz,zz]]),
                offsets=np.array([[y-c.Y(),z-c.Z()] for y in [b[1],b[4]] for z in [b[2],b[5]]]))

def distribute(f,m,left_share):
    off=(P-O)/1000.
    # Resolve Fx/Fy on four stopped contacts including an explicit yaw moment.
    # Subtract their contribution to roll/pitch because the land heights differ.
    h=np.zeros((3,8));h[0,::2]=1;h[1,1::2]=1
    h[2,::2]=-off[:,1];h[2,1::2]=off[:,0]
    xy=np.column_stack((f[:,:2],m[:,2]))@np.linalg.pinv(h).T
    forces=np.zeros((len(f),4,3));forces[:,:,:2]=xy.reshape(-1,4,2)
    residual=m-np.sum(np.cross(off,forces),axis=1)
    v=np.vstack((np.ones(4),off[:,1],-off[:,0],[1-left_share,-left_share,0.,0.]))
    targets=np.column_stack((f[:,2],residual[:,0],residual[:,1],np.zeros(len(f))))
    forces[:,:,2]=targets@np.linalg.inv(v).T
    if not np.allclose(forces.sum(axis=1),f,atol=1e-7) or not np.allclose(np.cross(off,forces).sum(axis=1),m,atol=1e-7):
        raise ValueError("Contact forces fail force/moment closure")
    return forces

def maxabs(a):return float(np.max(np.abs(a)))

def evaluate(t,f,m,sections,share):
    forces=distribute(f,m,share)
    rows={}
    for i,s in enumerate((1,-1)):
        front,rear=forces[:,i],forces[:,i+2]
        F=front+rear
        origin=np.array([-44.,210.*s,367.])
        M=np.cross((P[i]-origin)/1000.,front)+np.cross((P[i+2]-origin)/1000.,rear)
        # group() places its origin on the outer mast Y235. The new plate seat
        # lies on the inner plane Y210; shift applied moment to that exact plane.
        centered=origin.copy();centered[1]=235*s
        root={key:group(F,M,centered,positions,s) for key,positions in [('intact',BOLTS)]+[
            ('omit'+str(q),[p for n,p in enumerate(BOLTS) if n!=q]) for q in range(4)]}
        upper=0.;governing={};vertical_flexure=[]
        columns={c:dict(minI=float("inf"),compressionN=0.) for c in ("lower","upper")}
        for x,g in sections.items():
            p=g['center'].copy();p[1]*=s
            at=np.cross((P[i]-p),front)
            # M in N mm. Cross-sectional normal stress includes eccentric lateral
            # loading, the asymmetric rib, both bending moments and direct axial.
            slopes=np.column_stack((-at[:,2]*s,at[:,1]))@g['inv'].T
            vertical_flexure.append((x,slopes[:,1]*(-(FRONT-x))/E))
            normal=maxabs(front[:,0,None]/g['area']+slopes@g['offsets'].T)
            if len(g['children'])==2:
                for name,(_,a,offset,weak) in zip(('lower','upper'),g['children']):
                    # Effective chord compression combines the section resultant
                    # and bending stress at the center of this particular branch.
                    compression=float(np.max(-(front[:,0]/g['area']+slopes@offset)*a))
                    columns[name]['compressionN']=max(columns[name]['compressionN'],compression)
                    columns[name]['minI']=min(columns[name]['minI'],weak)
            if normal>upper:upper=normal;governing={'xMm':x,'normalMPa':normal}
        for c in columns.values():
            # Conservative full clear diagonal of the window (true length under
            # 190 mm). Pin-pin and cantilever are separate end-fixity hypotheses.
            c['pinPinCriticalN']=float(np.pi**2*E*c['minI']/190.**2)
            c['cantileverCriticalN']=c['pinPinCriticalN']/4
        flexure=np.trapezoid(np.stack([v for _,v in vertical_flexure]),x=[x for x,_ in vertical_flexure],axis=0)
        front_up=max(float(front[:,2].max()),0.)
        rear_up=max(float(rear[:,2].max()),0.)
        # All rebound taken by one front screw; 2.0 mm minimum remaining sheet (2.25 mm nominal) at the
        # head flank is a minimum first-article acceptance, not an acquired fact.
        rows['left' if s==1 else 'right']={'frontVerticalN':[float(front[:,2].min()),float(front[:,2].max())],
            'rearVerticalN':[float(rear[:,2].min()),float(rear[:,2].max())], 'chordBuckling':columns,
            'sampledSpanBendingDeflectionMm':[float(flexure.min()),float(flexure.max())],
            'oneForwardFastenerTensionMPa':(PRELOAD+front_up)/14.2,'oneForwardHeadPunchingMPa':front_up/(np.pi*9*2.0),
            'rearTappedJointTensionMPa':(PRELOAD+rear_up)/20.1,
            'rearThreadStrippingMPaAt12mmFullEngagement':(PRELOAD+rear_up)/(.5*np.pi*4.92*12),
            'rearResidualIfTappedScrewMissingN':rear_up,'mastConnections':root,'frontCantileverSection':governing,
            'maxVerticalClockS':float(t[int(np.abs(front[:,2]).argmax())])}
    return rows

def run(folder,out):
    old=j.read(ROOT/'fixed-hoist/lock-transfer.json')
    src=ROOT/'shelf-structure/coupled-mounts.json'
    if j.digest(src)!=old['sourceSha256']:raise ValueError("Wrong shelf histories")
    gravity=old['centeredReferenceDeadLoadN'];body=blank(1)
    sections={x:section(body,x) for x in range(-23,156,2)}
    result={}
    for case in j.read(src):
        csv=folder/(case['id']+'-support.csv')
        if j.digest(csv)!=case['supportHistorySha256']:raise ValueError('Changed force trace')
        data=np.loadtxt(csv,skiprows=1,delimiter=',');t=data[:,0];f=data[:,1:4].copy()
        # The source includes both dampers. Add the earlier centered gravity force once
        # at its historical lock location after transferring the source moments to O.
        cases={}
        for yaw in (-50.,50.):
            m=np.column_stack((data[:,4:6],np.full(len(data),yaw)))+np.cross((FOOT-O)/1000.,f)
            loaded=f.copy();loaded[:,2]-=gravity
            for share in (.35,.5,.65):
                cases[f'{yaw:+.0f}Nm/{share:.2f}']=evaluate(t,loaded,m,sections,share)
        one={}
        for s,side in ((1,'L'),(-1,'R')):
            loc=np.array([-44.,s*210.,374.5])
            translated=m+np.cross((O-loc)/1000.,loaded)
            # Translate fictitious coordinate representation so the earlier
            # fastener routine uses the true INNER face, not the outer face.
            fict=loc.copy();fict[1]=235*s
            one[side]=group(loaded,translated,fict,BOLTS,s)
        result[case['case']]={'sha256':j.digest(csv),'samples':len(t),'cases':cases,'entireSideLostAtPlus50NmYaw':one,
            'legacy':old['cases'][case['case']]}
    j.write(out,{'traceRegistrySha256':j.digest(src),'lockReferenceSha256':j.digest(ROOT/'fixed-hoist/lock-transfer.json'),
        'pitchArmMm':FRONT-REAR,'registeredLandCentersMm':P.tolist(),'centeredGravityN':gravity,
        'sectionModel':'linear intact gross material, bounds at corner of each cut; torsion, buckling, fatigue and assembled stiffness require separate checks',
        'measuredShare':None,'results':result})

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('traces',type=Path);p.add_argument('--out',type=Path,default=HERE/'reactions.json')
    a=p.parse_args();run(a.traces,a.out)
