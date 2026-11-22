"""Resolve a bumper strike from the perimeter saddles directly into the socket rails and the upper sills."""
from pathlib import Path
import json
import sys
import numpy as np
from diagonal_inertia_relief import run as relief
from diagonal_lower_elastic import planar_point,beam,SIDES
from perimeter_footprint import structure,cases_and_forces

HERE=Path(__file__).resolve().parent


def ring_section(xy,direction,inset=0.,stagger=False):
    # An 8.0 mm through-sleeve replaces the wall at each of two bolt elevations
    # per station. No composite section or structural credit for the installed pin.
    w,h,t=25.4+inset,101.6,3.
    area=w*h-(w-2*t)*(h-2*t)
    ix=(h*w**3-(h-2*t)*(w-2*t)**3)/12
    if min(abs(direction))<1e-7:
        s=xy[np.argmax(abs(direction))]
        chords=[]
        for spread in (13,6 if stagger else 13):
            dx=min(abs(s-off) for c in (-140+inset,140-inset) for off in (c-spread,c+spread))
            chords.append(2*np.sqrt(max(0.,4.05**2-dx**2)))
        taken=2*t*sum(chords) # one chord on each radial wall for each bolt elevation
        area-=taken
        ix-=taken*(t*t/12+((w-t)/2)**2)
    return area,ix,w/2


def construct(pkg,inset=0.,stagger=False):
    from four_corner_packaging import outline
    sys.path.insert(0,str(pkg/"scripts"))
    from fusion_authority import register_path,verify
    from math import hypot
    root=HERE.parents[3]
    reg=register_path(root);lin=reg.parent/"issue-005/lineage.json"
    auth=verify(root,reg,lin)
    rail_defs=json.loads((HERE/"socket-holes.json").read_bytes())["rails"]
    offsets={0.,9.,-9.}
    for station in (-153.+inset,-127.+inset,127.-inset,153.-inset):
        for d in (-4.05,0,4.05):
            offsets.add(station+d)
    if stagger:
        for center in (-140+inset,140-inset):
            for span in (-6,6):
                for edge in (-4.05,0,4.05):offsets.add(center+span+edge)
    nodes,faces,_,ring,recover,indices,_,_=structure(14,lambda xy,d:ring_section(xy,d,inset,stagger),sorted(offsets),outline(370,365,42,(25.4+inset)/2))
    n=len(ring)
    applied,cases=cases_and_forces(nodes,faces)
    corners=np.array([[282.,277.],[-282.,277.],[-282.,-277.],[282.,-277.]])
    side_centers=np.array([[357.3,0],[0,352.3],[-357.3,0],[0,-352.3]])
    corners-=np.sign(corners)*inset
    side_centers-=np.sign(side_centers)*inset/2
    support_station=140-inset
    endpoint=330.6-1.5*inset
    sill_y=128-inset
    next_dof=n
    coords={3*i:v for i,v in enumerate(nodes)}
    add=[]
    def create(xy):
        nonlocal next_dof
        first=next_dof;next_dof+=3;pt=np.array(xy,dtype=float)
        coords[first]=pt;add.append((first,pt))
        return first
    def ring_node(xy):
        d=np.linalg.norm(nodes-xy,axis=1);i=int(np.argmin(d))
        if d[i]>1e-6:raise ValueError(f"Missing structural register {xy}: {d[i]}")
        return 3*i
    rails=[];rail_maps=[];lower_nodes=[]
    for side,name in enumerate(SIDES):
        rail=rail_defs[name+" socket rail"]
        xy=np.array(rail["boltCentersMm"]);axis=rail["lengthAxis"];xy-=np.sign(xy)*inset;mean=xy.mean(0)
        stations=sorted(set(xy[:,axis])|{-support_station-13,-support_station+13,support_station-13,support_station+13})
        chain={}
        for s in stations:
            p=mean.copy();p[axis]=s;idx=create(p);chain[s]=idx
            lower_nodes.append((idx,side,p))
        rails.append((name,chain,rail));rail_maps.append(chain)
    saddles={}
    connections=[[],[],[]]
    for side,center in enumerate(side_centers):
        along=1 if side%2==0 else 0
        for station in (-support_station,support_station):
            pt=center.copy();pt[along]=station
            bracket=create(pt);saddles[(side,station)]=bracket
            for shift in (-13,13):
                line=pt.copy();line[along]+=shift
                connections[0].append((ring_node(line),bracket,line,0.5 if stagger else 1.))
                if stagger:
                    second=pt.copy();second[along]+=6*np.sign(shift)
                    connections[0].append((ring_node(second),bracket,second,.5))
                lower=rail_maps[side][station+shift]
                connections[1].append((lower,bracket,coords[lower]))
    sill={};bars={};frame_members=[]
    for y in (-sill_y,sill_y):
        points=[(-endpoint,y),(-100.,y),(110.,y),(250.,y),(endpoint,y)]
        for x,_ in points:sill[x,y]=create((x,y))
        frame_members.extend(("sill",sill[a[0],y],sill[b[0],y],False) for a,b in zip(points,points[1:]))
        for x,side in ((-endpoint,2),(endpoint,0)):
            support=saddles[(side,support_station*np.sign(y))]
            connections[0].append((sill[x,y],support,np.array([x,y])))
    for x in (110.,250.):
        ys=(-242.,-sill_y,sill_y,242.)
        for y in ys:bars[x,y]=sill[x,y] if (x,y) in sill else create((x,y))
        for y,z in zip(ys,ys[1:]):
            frame_members.append(("upper crossbar",bars[x,y],bars[x,z],True))
    module0=next_dof
    modules=[create(xy) for xy in corners]
    size=next_dof
    matrix=np.zeros((size,size));matrix[:n,:n]=ring
    joints=[[],[],[]];connections_xy=[]
    for side,(label,chain,rail) in enumerate(rails):
        for s,idx in chain.items():
            if all(abs(abs(s)-p)>1e-6 for p in (support_station-13,support_station+13)):
                point=coords[idx]
                m=modules[int(np.argmin(np.linalg.norm(corners-point,axis=1)))]
                connections[2].append((idx,m,point));connections_xy.append(point)
    for j,group in enumerate(connections):
        for term in group:
            a,b,world=term[:3];weight=term[3] if len(term)==4 else 1.
            joints[j].append(np.sqrt(weight)*(planar_point(size,a,world-coords[a])-planar_point(size,b,world-coords[b])))
    lower_members=[]
    def member(label,a,b,stock_width,stock_height):
        k,g,area,iz=beam(coords[a],coords[b],3.175)
        if (stock_width,stock_height)==(50.8,25.4):
            # Swap rectangular stock orientation in plane without rotating the axes.
            new_iz=(25.4*50.8**3-19.05*44.45**3)/12
            dx=np.linalg.norm(coords[b]-coords[a])
            tang=(coords[b]-coords[a])/dx
            T=np.r_[tang,0.,-tang,0.]
            axial=68900*area/dx*np.outer(T,T)
            ratio=new_iz/iz
            k=axial+(k-axial)*ratio
            g[[1,2,4,5]]*=ratio
        dofs=[a+i for i in range(3)]+[b+i for i in range(3)]
        matrix[np.ix_(dofs,dofs)]+=k
        return g,dofs,area,iz
    for label,chain,rail in rails:
        keys=list(chain)
        for a,b in zip(keys,keys[1:]):
            g,dofs,area,iz=member(label,chain[a],chain[b],25.4,50.8)
            lower_members.append((label,[a,b],g,dofs,area,iz))
    for name,a,b,wide in frame_members:
        member(name,a,b,50.8 if wide else 25.4,25.4 if wide else 50.8)
    upper=[bars[x,y] for x in (110.,250.) for y in (-242.,242.)]
    inner=[sill[x,y] for x in (110.,250.) for y in (-sill_y,sill_y)]
    battery=[sill[x,y] for x in (-100.,110.) for y in (-sill_y,sill_y)]
    deck=[sill[x,y] for x in (-100.,250.) for y in (-sill_y,sill_y)]
    joint_keys=sorted(saddles)
    saddle_ids=[saddles[k] for k in joint_keys]
    by_side=[ring_node(p) for p in side_centers]
    payload={"Port":[saddle_ids,saddle_ids],"Pan drive":[upper,inner],"Held CORAL":[upper,inner],
        "Upper contingency":[upper,inner],"Battery":[battery,battery],"Panel and electronics":[deck,battery],
        "Harness contingency":[deck,battery],"Joint contingency":[saddle_ids,upper],
        "General hardware":[deck,inner],"Unissued frame":[saddle_ids,deck]}
    impact=json.loads((HERE/"corner-tie-trial.json").read_bytes())["upperCaseN"]
    load=np.zeros((size,len(cases)));load[:n]=applied*impact
    return dict(authority=auth,nodes=nodes,ringSize=n,ringMembers=recover,ringDofs=indices,ringSection=lambda xy,d:ring_section(xy,d,inset,stagger),
        base=matrix,stiffnessParts=[sum(j.T@j for j in g) for g in joints],loadCases=cases,forces=load,
        sideCenters=side_centers,moduleStart=module0,moduleCenters=corners,lowerNodes=lower_nodes,lowerMembers=lower_members,
        anchorJac=joints[2],anchorXY=connections_xy,pilots=joints[1],diagonals=[],additionalNodes=add,terminalNodes=by_side,
        payloadNodes=payload,impactY=side_centers[1][1],ledgerFile="inset-stagger-inertia-ledger.json" if stagger else "inset-saddle-inertia-ledger.json" if inset else "direct-saddle-inertia-ledger.json")


if __name__=="__main__":
    inset=12.7 if "--inset" in sys.argv else 0.
    stagger='--stagger' in sys.argv
    result=relief(Path(sys.argv[1]),alternative=lambda p:construct(p,inset,stagger))
    result["moduleInsetPerAxisMm"]=inset
    result["designScope"]="Eight separate straddling saddles directly attach the perimeter to 24 original motor-module mounting screw stations via the four lower socket rails. Two inboard longitudinal sills and two transverse bars carry the pan and battery. The same registered heavy mass envelope is retained for like impact. The 'pilotN' metric refers to lower saddle transfer stations. Spring magnitudes are sensitivity tests; resistance to out-of-plane prying and sleeve/bolt strength require the actual joint."
    (HERE/("inset-stagger-inertia.json" if stagger else "inset-saddle-inertia.json" if inset else "direct-saddle-inertia.json")).write_text(json.dumps(result,indent=2)+"\n")
    for label in result["states"][0]["peak"]:
        print(label,{k:round(max(st["peak"][label][k]["value"] for st in result["states"]),2) for k in result["states"][0]["peak"][label]})
