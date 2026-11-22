"""Compute elastic reaction sharing in the proposed perimeter and challenge a missed attachment."""
import json,math
from pathlib import Path
import numpy as np
p=Path(__file__).parent
parent=json.loads((p/'bumper-duty.json').read_bytes())
# Beam model at section centrelines; outline is offset at the two true rear miters.
def corners():
    b=12.7
    x=370-b;y=365-b;cut=42-b*(2-math.sqrt(2))
    return [(x,y),(-x+cut,y),(-x,y-cut),(-x,-y+cut),(-x+cut,-y),(x,-y)]

def assemble(h, mount_positions):
    v=corners();nodes=[];segments=[];anchors={};force_nodes={}
    for i,(a,b) in enumerate(zip(v,v[1:]+v[:1])):
        axis=0 if abs(a[1]-b[1])<.01 else 1 if abs(a[0]-b[0])<.01 else None
        pts=[(0.,a,None)]
        if axis is not None:
            for station in mount_positions:
                u=(station-a[axis])/(b[axis]-a[axis]);xy=list(a);xy[axis]=station
                pts.append((u,tuple(xy),'mount'))
            # explicitly measure each span's midpoint, including both ends
            ordered=[v[0] for v in sorted(pts)]+[1.]
            for u,w in zip(ordered,ordered[1:]):
                f=(u+w)*.5;xy=tuple(a[j]+f*(b[j]-a[j]) for j in (0,1));pts.append((f,xy,'mid'))
        for u,xy,kind in sorted(pts):
            n=len(nodes);nodes.append(xy)
            if kind=='mount':anchors[n]=dict(face=i,xy=xy)
            if kind in ('mid',None):force_nodes[n]=i
    count=len(nodes);K=np.zeros((3*count,3*count))
    A=25.4*h-(25.4-6)*(h-6);I=(h*25.4**3-(h-6)*(25.4-6)**3)/12
    for n,(a,b) in enumerate(zip(nodes,nodes[1:]+nodes[:1])):
        x,y=b[0]-a[0],b[1]-a[1];L=math.hypot(x,y);c=x/L;s=y/L;E=68900.
        k=E*np.array([[A/L,0,0,-A/L,0,0],[0,12*I/L**3,6*I/L**2,0,-12*I/L**3,6*I/L**2],[0,6*I/L**2,4*I/L,0,-6*I/L**2,2*I/L],[-A/L,0,0,A/L,0,0],[0,-12*I/L**3,-6*I/L**2,0,12*I/L**3,-6*I/L**2],[0,6*I/L**2,2*I/L,0,-6*I/L**2,4*I/L]])
        T=np.zeros((6,6));rot=np.array([[c,s,0],[-s,c,0],[0,0,1]])
        T[:3,:3]=rot;T[3:,3:]=rot
        ids=list(range(3*n,3*n+3))+list(range(3*((n+1)%count),3*((n+1)%count)+3))
        K[np.ix_(ids,ids)]+=T.T@k@T
        segments.append((ids,k,T))
    return nodes,anchors,force_nodes,K,segments,A,I

def compare(h,stations,k,disabled):
    nodes,anchors,force_nodes,K,segs,A,I=assemble(h,stations)
    active={n:a for n,a in anchors.items() if a['xy']!=disabled}
    for n in active:
        for j in (0,1):K[3*n+j,3*n+j]+=k
    compliance=np.linalg.solve(K,np.eye(len(K)))
    F=parent['upper']['peakN'];cases=[];directions=[(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,0)]
    for n,face in force_nodes.items():
        d=directions[face];d=np.array(d)/np.linalg.norm(d)
        # At a corner include BOTH adjacent face normals, especially the actual
        # front impact at the front-left corner of the frame.
        ds=[d]
        if nodes[n] in corners():
            z=np.array(directions[(face-1)%6]);ds.append(z/np.linalg.norm(z))
        for d in ds:
            f=np.zeros(K.shape[0]);f[3*n:3*n+2]=F*d
            u=compliance@f
            reactions={m:(-k*u[3*m:3*m+2]).tolist() for m in active}
            if np.linalg.norm(F*d+np.sum(list(reactions.values()),axis=0))>.001:raise ValueError('Frame equilibrium mismatch')
            mm=sum(np.cross([*nodes[m],0],[*fr,0])[2] for m,fr in reactions.items())+np.cross([*nodes[n],0],[*list(F*d),0])[2]
            if abs(mm)>.5:raise ValueError('Unbalanced planar moment')
            tangent=max(abs(reactions[m][0 if active[m]['face'] in (0,4) else 1]) for m in active)
            normal=max(abs(reactions[m][1 if active[m]['face'] in (0,4) else 0]) for m in active)
            critical=max(reactions,key=lambda m:np.linalg.norm(reactions[m]))
            stress=0
            for ids,kl,T in segs:
                fl=kl@T@u[ids]
                stress=max(stress,abs(fl[0])/A+max(abs(fl[2]),abs(fl[5]))*12.7/I)
            cases.append(dict(atMm=nodes[n],appliedUnit=d.tolist(),peakAnchorN=np.linalg.norm(reactions[critical]),anchorXYmm=nodes[critical],
                            peakNormalToTubeN=normal,peakAlongTubeN=tangent,peakNormalStressMPa=stress,peakDisplacementMm=max(np.linalg.norm(u.reshape(-1,3)[:,:2],axis=1)),criticalReactionN=reactions[critical]))
    return dict(maximumAlong=max(cases,key=lambda v:v['peakAlongTubeN']),maximumNormal=max(cases,key=lambda v:v['peakNormalToTubeN']),maximumAnchor=max(cases,key=lambda v:v['peakAnchorN']),maximumStress=max(cases,key=lambda v:v['peakNormalStressMPa']),numberCases=len(cases))
def run():
    studies={}
    for h,stations in [(50.8,[-150,-100,100,150]),(76.2,[-150,-100,0,100,150]),(101.6,[-150,-100,0,100,150])]:
        for stiff in [3e3,3e4]:
            for loss in [None,(150,352.3)]:
                k=f'h{h}-K{stiff}-loss{loss}';studies[k]=compare(h,stations,stiff,loss)
    return dict(cases=studies,forceN=parent['upper']['peakN'],stiffnessRangeNPerMm=[3000,30000],loadFactorsIncluded='Compare computed stress to 120 MPa at FoS2; compare anchor to 3429 N single-wall yield /2. Reverse load changes reaction signs in this linear model.',limits='Planar Euler-Bernoulli frame with perfect rigid miter splices, finite bearing stiffness on each allocated vertical fastener and clamped inner primary chassis. Sensitivity to one missing station. Vertical prying, contact compliance/fatigue and inboard frame flexibility are independent.')
if __name__=='__main__':
 d=run();(p/'perimeter-elastic-load.json').write_text(json.dumps(d,indent=2)+'\n');print([(k,round(v['maximumAnchor']['peakAnchorN']),round(v['maximumStress']['peakNormalStressMPa'])) for k,v in d['cases'].items()])
