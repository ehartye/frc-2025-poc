"""Resolve three-axis upper reactions into closed tube sections inward of the encoder relief."""
import hashlib,json,sys
from pathlib import Path
import numpy as np
from inset_key_contact import section

HERE=Path(__file__).resolve().parent


def station(x,y,v):
    a,b,c,d,e,T=v
    p=np.array([a,0.,b]);s=np.array([c,e,d])
    moment=np.cross([.025,.195775,.1],p)+np.cross([.025,-.195775,.1],s)+[0,T,0]
    dx=(x-180)/1000;dy=y/1000;j=4*(.07**2+.242**2)
    return 2*((p+s)/4+[-moment[2]*dy/j,moment[2]*dx/j,moment[0]*dy/(4*.242**2)-moment[1]*dx/(4*.07**2)])


def path_gap(a,b,bounds,radius):
    # Minimum Euclidean distance of the complete segment to the solid AABB.
    # Every piecewise quadratic interval between face crossings is convex.
    low,high=np.array(bounds[:3]),np.array(bounds[3:]);v=b-a
    ts=[0.,1.]
    for j in range(3):
        if abs(v[j])>1e-12:
            ts.extend(t for t in ((low[j]-a[j])/v[j],(high[j]-a[j])/v[j]) if 0<t<1)
    ts=sorted(set(ts));tests=list(ts)
    for lo,hi in zip(ts,ts[1:]):
        at=a+(lo+hi)*v/2;mask=(at<low)|(at>high)
        if (v[mask]@v[mask])>0:
            target=np.clip(at,low,high)
            t=(v[mask]@(target-a)[mask])/(v[mask]@v[mask]);tests.append(np.clip(t,lo,hi))
    return min(np.linalg.norm(a+t*v-np.clip(a+t*v,low,high)) for t in tests)-radius


def run(pkg, record_foot=None):
    sys.path.insert(0,str(pkg/"scripts"))
    from fusion_authority import register_path,verify
    root=HERE.parents[3];reg=register_path(root);auth=verify(root,reg,reg.parent/"issue-005/lineage.json")
    d=root/"artifacts/red2-integrated";source=d/"single-pivot-spatial-rim-installed-reactions-frame-anchors.json"
    ref=d/"pan-thrust/two-sill-cartridge-selected-duty.json";src=json.loads(source.read_bytes());old=json.loads(ref.read_bytes())
    h=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    if h(source)!=old["sourceMatched"]:raise ValueError("Unregistered load history")
    shop=d/"pan-mechanics.json";cy=json.loads(shop.read_bytes())["movingShopCentreMm"][1]/1000
    deck=HERE/"inset-electrical-package.json";elec=json.loads(deck.read_bytes())
    if elec["authority"]!=auth:raise ValueError("Electrical packaging on wrong chassis")
    gauges={**{k:r["boundsMm"] for k,r in elec["candidatePlacements"].items()},**elec["gaugesMm"]}
    full=section(3.,False)
    side,w=19.05,1.5875
    area=side**2-(side-2*w)**2;inertia=(side**4-(side-2*w)**4)/12
    stock=[];actions=[];layout={};loads=[];hist={}
    for name,row in src["runs"].items():
        for tower in ("port","starboard"):
            frac=(1+(1 if tower=="port" else -1)*cy/.195775)/2
            for key,ev in row["anchorRows"]["pipeAndShopMass/"+tower].items():
                rp=np.array(ev["portRadialXZ_N"]);rs=np.array(ev["starboardRadialXZ_N"]);f=np.array(ev["forceToTowerN"])
                fabrication=(f[[0,2]]+(rp if tower=="port" else rs))/frac
                P=-rp+fabrication*(1+cy/.195775)/2;S=-rs+fabrication*(1-cy/.195775)/2
                loads.append((dict(run=name,channel=tower+"/"+key,time=ev["timeS"]),[*P,*S,ev["pipeForceN"][1],-ev["requiredActuatorTorqueY_Nm"]]))
    for sy in (-1,1):
        nodes=np.array([[110,242*sy,480.],[250,242*sy,480.],*[ [x,y*sy,187.325] for x in (110.,250.) for y in (145.,212.)]])
        edges=[(0,2),(0,3),(1,4),(1,5),(0,1),(0,4)]
        B=np.zeros((18,6));lengths=[];gaps=[]
        for n,(i,j) in enumerate(edges):
            direction=nodes[j]-nodes[i];L=np.linalg.norm(direction);u=direction/L;lengths.append(L)
            B[3*i:3*i+3,n]=u;B[3*j:3*j+3,n]=-u
            gaps.append({name:path_gap(nodes[i],nodes[j],bb,side/np.sqrt(2)) for name,bb in gauges.items()})
            stock.append(dict(side=sy,start=nodes[i].tolist(),end=nodes[j].tolist(),massKg=L*area*2.7e-6,
                              minimumDeviceOrServiceGapBoundMm=min(gaps[-1].values()),limiting=min(gaps[-1],key=gaps[-1].get)))
        coeff=-np.linalg.inv(B[:6])
        if np.linalg.matrix_rank(B[:6])!=6:raise ValueError("Unsupported upper coordinate")
        A=[np.column_stack([station(x,sy*242.,q) for q in np.eye(6)]) for x in (110.,250.)]
        for event,v in loads:
            F=np.r_[A[0]@v,A[1]@v];N=coeff@F;floor=(B[6:]@N).reshape(4,3)
            if np.max(abs(floor.sum(0)-F.reshape(2,3).sum(0)))>1e-7:raise ValueError("Unbalanced forces")
            M=sum((np.cross(nodes[i],f) for i,f in enumerate(F.reshape(2,3))),np.zeros(3))
            if max(abs(M-sum((np.cross(p,f) for p,f in zip(nodes[2:],floor)),np.zeros(3))))>.001:raise ValueError("Unbalanced moments")
            if record_foot:
                for point,reaction in zip(nodes[2:],floor):
                    record_foot(event,point,reaction)
            for j,(x,p,q) in enumerate(((110.,floor[0],floor[1]),(250.,floor[2],floor[3]))):
                for y in (115.3,145.,212.):
                    active=[(n,f) for n,f in zip(nodes[2+j*2:4+j*2],(p,q)) if sy*n[1]>=y]
                    net=sum((f for n,f in active),np.zeros(3));moment=sum((np.cross(n-[x,y*sy,174.625],f) for n,f in active),np.zeros(3))
                    sigma=abs(net[1])/full["area"]+abs(moment[0])/full["Zvertical"]+abs(moment[2])/full["Zhorizontal"]
                    # Closed thin-wall torsion plus deliberately conservative twice-average transverse shear.
                    tau=abs(moment[1])/(2*(50.8-3)*(25.4-3)*3)+2*np.hypot(net[0],net[2])/full["area"]
                    eq=np.hypot(sigma,np.sqrt(3)*tau)
                    if eq>hist.get("rail",{}).get("MPa",-1):
                        hist["rail"]=dict(MPa=eq,normalMPa=sigma,shearMPa=tau,x=x,y=y*sy,netN=net.tolist(),momentNmm=moment.tolist(),event=event)
            member_stress=max(abs(N))/area
            if member_stress>hist.get("stock",{}).get("MPa",-1):hist["stock"]=dict(MPa=member_stress,forcesN=N.tolist(),event=event,side=sy)
            euler=[max(0,-n)*L**2/(np.pi**2*68900*inertia) for n,L in zip(N,lengths)]
            if max(euler)>hist.get("buckling",{}).get("ratio",-1):hist["buckling"]=dict(ratio=max(euler),event=event,side=sy)
            if np.linalg.norm(floor,axis=1).max()>hist.get("foot",{}).get("N",-1):
                i=int(np.argmax(np.linalg.norm(floor,axis=1)));hist["foot"]=dict(N=np.linalg.norm(floor[i]),station=nodes[i+2].tolist(),reactionN=floor[i].tolist(),event=event)
        layout[str(sy)]=dict(nodes=nodes.tolist(),edges=edges)
    return dict(authority=auth,inputs={p.name:h(p) for p in (source,ref,shop,deck)},events=len(loads),workingMPa=120,
                layout=layout,stock=stock,idealPinEndedStockMassKg=sum(p["massKg"] for p in stock),sizing=hist,
                limits="A prospective triangulated support joins the existing four upper pickup datums to two full-depth stations on each X110/250 beam. The 10.5 mm end relief carries no upper fitting. Two separate side trusses with actual force/moment equilibrium, geometry, nominal 19.05 square x 1.5875 wall section and signed peak event loads from the old 2x duty. Buckling uses pinned ends. Connection offset, stock tolerance, fitting and fastener mass, in-plane wall event, actual pan and wiring sweeps remain to be resolved. A negative capsule-to-box lower bound is inconclusive and needs actual geometry.")
if __name__=="__main__":
    result=run(Path(sys.argv[1]));(HERE/"inset-pan-truss.json").write_text(json.dumps(result,indent=2)+"\n")
    print(result["idealPinEndedStockMassKg"],result["sizing"])
    print([(r["side"],r["limiting"],round(r["minimumDeviceOrServiceGapBoundMm"],2)) for r in result["stock"]])
