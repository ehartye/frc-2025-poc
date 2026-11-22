"""Resolve how much bumper wrench crosses a commercial corner and test separate socket ties."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from direct_saddle_inertia import construct
from diagonal_lower_elastic import beam,planar_point

HERE=Path(__file__).resolve().parent


def run(package,decks=False,clearance=False,truss=False):
    m=construct(package,12.7,True)
    names=["inset-stagger-inertia-ledger.json","inset-stagger-all-contacts.json","bumper-duty.json"]
    ledger,previous,docking=[json.loads((HERE/n).read_bytes()) for n in names]
    if ledger["authority"]!=m["authority"] or previous["authority"]!=m["authority"]:raise ValueError("Changed assembly")
    original=previous["peaks"]["innerScrewForceUpperN"]
    c=next(i for i,s in enumerate(m["loadCases"]) if s==original["contact"])
    xy={int(k):np.array(v) for k,v in ledger["fixedSupportDOFs"].items()}
    rigid=np.zeros((len(m["base"]),3))
    for d,(x,y) in xy.items():rigid[d:d+3]=[[1,0,-y],[0,1,x],[0,0,1]]
    mass,cg,yaw=ledger["massKg"],np.array(ledger["cgMm"]),ledger["yawKgM2"]
    start=m["moduleStart"]; centers=m["moduleCenters"]
    hit=m["forces"][:,c]; fn=(rigid.T@hit)[:2]; normal=fn/np.linalg.norm(fn)
    wheel=original["tireWheel"]; kr,kp,km=original["stiffnessNPerMm"]
    def load(force,tire):
        applied=hit*(force/np.linalg.norm(fn))
        applied[start+3*wheel:start+3*wheel+2]+=normal*tire
        fx,fy,mz=rigid.T@applied
        linear=np.array([fx,fy])/mass;alpha=(mz-cg[0]*fy+cg[1]*fx)/(1000*yaw)
        for _,kg,pt,j,groups in ledger["items"]:
            pt=np.array(pt); rel=pt-cg;ids=groups[original["payloadCase"]]
            f=-kg*(linear+alpha*np.array([-rel[1],rel[0]])/1000)
            mean=np.mean([xy[d] for d in ids],axis=0);r=pt-mean
            moment=-1000*j*alpha+r[0]*f[1]-r[1]*f[0]
            if len(ids)==1:
                d=ids[0];applied[d:d+2]+=f;applied[d+2]+=moment
            else:
                offs=np.array([xy[d]-mean for d in ids])
                for d,(dx,dy) in zip(ids,offs):applied[d:d+2]+=f/len(ids)+moment*np.array([-dy,dx])/(offs**2).sum()
        if max(abs(rigid.T@applied))>.001:raise ValueError("Unbalanced study")
        return applied
    routine=docking["routine"]
    forces=[load(np.linalg.norm(fn),original["tireForceSign"]*.8*mass*9.80665),
            load(routine["quasistaticInterpolatedPeakN"],0),
            load(0,.8*mass*9.80665)]
    labels=["governing rare wall","routine dock force at same site, coasting","single-wheel traction without impact"]
    rhs=np.column_stack(forces)
    axes=np.array(m["anchorXY"]);ids=np.argmin(np.linalg.norm(axes[:,None]-centers,axis=2),axis=1)
    joined=[]
    for corner,center in enumerate(centers):
        nodes=[(d,side,pt) for d,side,pt in m["lowerNodes"] if np.all(np.sign(pt)==np.sign(center))]
        side_ids=sorted({s for d,s,pt in nodes})
        if len(side_ids)!=2:raise ValueError("Missing perpendicular frame face")
        sets=[]
        for s in side_ids:
            chain=sorted([(d,p) for d,side,p in nodes if side==s],key=lambda e:abs(e[1][1 if s%2==0 else 0]))
            sets.append(chain)
        for select,rank in ((1,"saddle"),(2,"inner bolt"),(4,"outer bolt")):
            (a,p),(b,q)=sets[0][select],sets[1][select]
            joined.append(dict(corner=corner,rank=rank,a=a,b=b,p=p,q=q))
    base=m["base"]+sum(k*v for k,v in zip((kr,kp,km),m["stiffnessParts"]))
    free=np.setdiff1d(np.arange(len(base)),[start,start+1,start+2])
    def spread(K,index,stiffness):
        # Move the outward half of each of the eight existing saddles toward
        # the module. Remove its original upper and lower attachment springs.
        # Interpolate a load wherever the new attachment falls between FEM nodes.
        size=len(base)
        def at_host(rows,axis,at):
            points=sorted(rows,key=lambda row:row[1][axis])
            nearest=min(points,key=lambda row:abs(row[1][axis]-at[axis]))
            if abs(nearest[1][axis]-at[axis])<1e-7:
                return planar_point(size,nearest[0],at-nearest[1])
            for (i,p),(j,q) in zip(points,points[1:]):
                if p[axis]<at[axis]<q[axis]:
                    t=(at[axis]-p[axis])/(q[axis]-p[axis])
                    return (1-t)*planar_point(size,i,at-p)+t*planar_point(size,j,at-q)
            raise ValueError("Outside a host member")
        locations=[]
        for side,center in enumerate(m["sideCenters"]):
            axis=1 if side%2==0 else 0
            rings=[(3*i,p) for i,p in enumerate(m["nodes"]) if abs(p[1-axis]-center[1-axis])<1e-6]
            rails=[(d,p) for d,s,p in m["lowerNodes"] if s==side]
            for ordinal,sgn in enumerate((-1,1)):
                at=center.copy();at[axis]=sgn*127.3
                jj=m["pilots"][side*4+ordinal*2+int(sgn>0)]
                inds=np.where(np.any(abs(jj)>1e-8,axis=0))[0]
                bracket,=[d for d in set((inds//3)*3) if np.linalg.norm(xy[d]-at)<1e-6]
                K-=kp*(jj.T@jj)
                for offset in (6,13):
                    old=at.copy();old[axis]+=sgn*offset
                    jj=at_host(rings,axis,old)-planar_point(size,bracket,old-at)
                    K-=.5*kr*(jj.T@jj)
                anchors=sorted({abs(p[axis]) for p in m["anchorXY"][side*6:side*6+6]})
                station=sgn*(anchors[index]+anchors[index+1])/2
                lower=rails[0][1].copy();lower[axis]=station
                upper=at.copy();upper[axis]=station
                mid=upper.copy();mid[axis]-=sgn*7
                foot=(upper+lower)/2
                world=[lower,upper,mid]
                hosts=[at_host(rails,axis,lower),at_host(rings,axis,upper),at_host(rings,axis,mid)]
                hs=np.vstack(hosts);js=np.vstack([planar_point(3,0,w-foot) for w in world])
                weights=np.diag(np.repeat([stiffness,stiffness/2,stiffness/2],2))
                reduced=weights-weights@js@np.linalg.solve(js.T@weights@js,js.T@weights)
                K+=hs.T@reduced@hs
                locations.append(dict(side=side,lowerMm=lower.tolist(),upperMm=[upper.tolist(),mid.tolist()]))
        return locations
    def outer_knee(K,stiffness):
        # One mitered two-leg tube outside each module. Two external mounting
        # stations on each leg lie between the original vertical factory screws.
        # Release all bolt rotational DOFs. Carry horizontal load through the
        # two screw stations in shear, with finite housing/bolt compliance.
        cases=[];mass=0.
        for i,c in enumerate(centers):
            resources=[]
            for side in ([0,1] if i==0 else [2,1] if i==1 else [2,3] if i==2 else [0,3]):
                ax=1 if side%2==0 else 0
                rails=sorted([(d,p) for d,s,p in m["lowerNodes"] if s==side and np.sign(p[ax])==np.sign(c[ax])],key=lambda v:abs(v[1][ax]))
                # Two midpoints between the three selected purchased screw stations.
                stations=[]
                for (a,p),(b,q) in zip(rails[-3:],rails[-2:]):
                    inside=(p+q)/2;at=inside.copy();at[1-ax]+=np.sign(c[1-ax])*25.4
                    JJ=.5*(planar_point(len(base),a,inside-p)+planar_point(len(base),b,inside-q))
                    stations.append((at,inside,JJ))
                resources.append(stations)
            pts=[v[0] for v in resources[0]]+[np.array([resources[0][0][0][0],resources[1][0][0][1]])]+[v[0] for v in reversed(resources[1])]
            B=np.zeros((15,15));recover=[];legs=0.
            for j,(p,q) in enumerate(zip(pts,pts[1:])):
                k,g,area,I=beam(p,q,3.175);d=np.arange(3*j,3*j+6)
                B[np.ix_(d,d)]+=k;recover.append((d,g));legs+=np.linalg.norm(q-p)
            hs=[];subset=[]
            for j,(a,inside,J) in zip((0,1,3,4),resources[0]+resources[1][::-1]):
                hs.append(J);subset.append(planar_point(15,j*3,inside-a))
            H=np.vstack(hs);J=np.vstack(subset);W=np.eye(8)*stiffness
            element=np.linalg.solve(B+J.T@W@J,J.T@W@H)
            K+=H.T@W@(H-J@element)
            cases.append((pts,element,recover,W@(H-J@element)));mass+=legs*area*2.7e-6
        return cases,mass
    outcomes={}
    midcut=94.
    variants=(("existing six screws",[],1,False),("one inner socket-to-socket tube",["inner bolt"],1,False),
                               ("two tubes between the lower sockets",["inner bolt","outer bolt"],1,False),
                               ("ideal stiff dual tie control",["inner bolt","outer bolt"],1000,False),
                               ("independent pairs of sockets",[],1,True),
                               ("separate sockets and inner tie",["inner bolt"],1,True),
                               ("separate sockets and outer tie",["outer bolt"],1,True),
                               ("separate sockets and dual tie",["inner bolt","outer bolt"],1,True),
                               *((f"split saddle interval {i} at {k}",[],k,True) for i in (0,1) for k in (3000,30000)),
                               *((f"vertical follower on transverse faces {k}",[],k,True) for k in (0,30,100,300,3000)),
                               *((f"ring cap {r}"+(" with continuous sockets" if not split else ""),[r],1,split) for r in ("saddle","inner bolt","outer bolt") for split in (False,True)),
                               *((f"outboard miter at {k}",[],k,True) for k in (3000,10000,30000)),
                               *((f"single-face cassette at {k}",[],k,True) for k in (3000,10000,30000)))
    if decks:
        variants=(variants[0],("single-face cassette at 10000",[],10000,True),
                  *((f"single-face deck {k} {g}",[],10000,True) for k,g in ((500,18),(3000,18),(10000,18),(3000,13))),
                  *((f"single-face deck keyed {k} {g}",[],10000,True) for k,g in ((500,18),(3000,18),(10000,18),(3000,13))))
    if clearance:
        variants=tuple(v for v in variants if v[0]=="existing six screws" or "deck keyed" in v[0])
        labels += ["reversed "+v for v in labels]
        rhs=np.column_stack((rhs,-rhs))
    for title,ranks,factor,cut in variants:
        K=base.copy();ties=[];stockkg=0;removed=0.
        if cut:
            for side in range(4):
                ax=1 if side%2==0 else 0
                stations=[(d,p) for d,s,p in m["lowerNodes"] if s==side]
                (a,p),(b,q)=[min((entry for entry in stations if np.sign(entry[1][ax])==sgn),key=lambda e:abs(e[1][ax])) for sgn in (-1,1)]
                k,g,area,_=beam(p,q,3.175)
                d=np.r_[a:a+3,b:b+3];K[np.ix_(d,d)]-=k
                removed+=2*midcut*area*2.7e-6
        for ch in joined:
            if ch["rank"] not in ranks:continue
            a,b,p,q=ch["a"],ch["b"],ch["p"],ch["q"]
            if title.startswith("ring cap"):
                face_centers=m["sideCenters"]
                targets=[]
                for v in (p,q):
                    side=int(np.argmin([abs(v[1 if j%2 else 0]-center[1 if j%2 else 0]) for j,center in enumerate(face_centers)]))
                    side_axis=1 if side%2==0 else 0
                    target=face_centers[side].copy();target[side_axis]=v[side_axis]
                    near=int(np.argmin(np.linalg.norm(m["nodes"]-target,axis=1)))
                    targets.append((3*near,m["nodes"][near]))
                (a,p),(b,q)=targets
            k,g,area,_=beam(p,q,3.175)
            d=np.r_[a:a+3,b:b+3];K[np.ix_(d,d)]+=k*factor;ties.append((d,g*factor,p,q))
            stockkg+=np.linalg.norm(q-p)*area*2.7e-6
        new=spread(K,int(title.split()[3]),factor) if title.startswith("split saddle") else []
        cassette=title.startswith("single-face")
        knee=title.startswith("outboard miter") or cassette
        knee_data=[]
        if knee:knee_data,stockkg=outer_knee(K,factor)
        follower=title.startswith("vertical follower")
        if follower or cassette:
            for i,jj in enumerate(m["pilots"]):
                if (i//4)%2==0:
                    K+=((0 if cassette else factor)-kp)*(jj.T@jj)
        panel_rows=[]
        if title.startswith("single-face deck"):
            from inset_electronics_deck import plate
            k_deck,grid=map(int,title.split()[-2:])
            support,panel_rows=plate(m,xy,k_deck,grid,thickness=1.8796 if "keyed" in title else 2.,keyed="keyed" in title,contact=clearance,truss=truss)
            K+=support
        closure=None
        if clearance and panel_rows:
            from inset_key_contact import solve,relieve_end
            ends=relieve_end(K,xy)
            u,closure=solve(K,free,rhs,panel_rows,k_deck,0.1)
        else:
            u=np.zeros_like(rhs);u[free]=np.linalg.solve(K[np.ix_(free,free)],rhs[free])
            if abs((K@u-rhs)[:,:]).max()>1.:raise ValueError("False gauge reaction")
        reactions=np.array([km*j@u for j in m["anchorJac"]])
        outcomes[title]=dict(grossAddedTubeKg=stockkg,removedCenterStockKg=removed,netBareTubeChangeKg=stockkg-removed,
            splitSaddleStations=new,finiteAttachmentNPerMm=factor if new else None,separateTransverseAttachmentRemoved=cassette,
            decks=[{k:v for k,v in r.items() if k not in ("force","hostMap","contactStiffness")} for r in panel_rows],
            deckAttachmentNPerMm=k_deck if panel_rows else None,
            followerHorizontalNPerMm=factor if follower else None,tubeStiffnessMultiplier=1 if (new or follower or knee) else factor,
            kneePointsMm=[[p.tolist() for p in pts] for pts,_,_,_ in knee_data],kneeAttachmentNPerMm=factor if knee else None,
            ties=[dict(corner=v["corner"],rank=v["rank"],startMm=p.tolist(),endMm=q.tolist()) for v,(_,_,p,q) in zip([v for v in joined if v["rank"] in ranks],ties)],cases={})
        for c,name in enumerate(labels):
            values=reactions[:,:,c];per=[]
            for i,pt in enumerate(centers):
                group=np.where(ids==i)[0];f=values[group];r=axes[group]-pt
                moment=(r[:,0]*f[:,1]-r[:,1]*f[:,0]).sum()
                per.append(dict(module=i,netN=f.sum(0).tolist(),netNm=moment/1000,sides=[dict(rail=int(k),resultantN=values[[j for j in group if j//6==k]].sum(0).tolist()) for k in sorted({j//6 for j in group})]))
            norms=np.linalg.norm(values,axis=1);j=int(np.argmax(norms))
            movement=max((np.linalg.norm(jj@u[:,c]) for i,jj in enumerate(m["pilots"]) if i//4%2==0),default=0)
            bolts=[];knee_moments=[]
            for pts,E,rec,J in knee_data:
                bolts.extend(np.linalg.norm((J@u[:,c]).reshape((-1,2)),axis=1))
                for dd,g in rec:knee_moments.extend(abs((g@(E@u[:,c])[dd])[[2,5]])/1000)
            deck_loads=[]
            for sheet_id,panel in enumerate(panel_rows):
                if closure is None:panel_f=(panel["force"]@u[:,c]).reshape((-1,2))
                else:panel_f=np.array(closure[c]["sheetLocalN"][sheet_id]).reshape((-1,2))
                for slot,(pos,f) in enumerate(zip(panel["supports"],panel_f)):
                    deck_loads.append(dict(xyMm=pos,connection="perimeter" if slot<4 else "inboard sill",
                        sheetForceN=[f[0],f[1]*panel["sign"]],magnitudeN=np.linalg.norm(f)))
            ring=[]
            for i,(rr,d) in enumerate(zip(m["ringMembers"],m["ringDofs"])):
                nxt=(i+1)%len(m["nodes"]);q=rr@u[d,c]
                for ix,ax,bend in ((i,0,2),(nxt,3,5)):
                    a,I,fibre=m["ringSection"](m["nodes"][ix],m["nodes"][nxt]-m["nodes"][i])
                    ring.append(abs(q[ax])/a+abs(q[bend])*fibre/I)
            outcomes[title]["cases"][name]=dict(maxScrewN=norms[j],siteMm=axes[j].tolist(),signedN=values[j].tolist(),
                maxTransverseMountTravelMm=movement,sameCaseRingMPa=max(ring),kneeMaxAttachmentN=max(bolts,default=0.),kneeMaxBendingNm=max(knee_moments,default=0.),
                maxDeckBoltN=max((r["magnitudeN"] for r in deck_loads),default=0.),
                deckLoads=deck_loads,
                governingModule=per[ids[j]],allModuleResultants=per,maxTieEndMomentNm=max((max(abs((g@u[d,c])[[2,5]]))/1000 for d,g,p,q in ties),default=0.))
            if closure is not None:
                row=outcomes[title]["cases"][name]
                row["physicalKeyContact"]=closure[c]
                row["relievedSectionEndWrenches"]=[dict(y=s,localN_N_Nmm=(B@u[d,c]).tolist()) for s,d,B in ends]
                row["modulePoses"]=[dict(atMm=pt.tolist(),dX_dY_yawRad=u[start+3*i:start+3*i+3,c].tolist()) for i,pt in enumerate(centers)]
                row["hostPoses"]=[dict(atMm=p.tolist(),dX_dY_yawRad=u[d:d+3,c].tolist(),dof=d) for d,p in xy.items()]
    old=outcomes["existing six screws"]["cases"][labels[0]]["maxScrewN"]
    if not original["value"]*np.cos(np.radians(2.5))-.001<=old<=original["value"]+.001:raise ValueError("Independent control mismatch")
    return dict(authority=m["authority"],inputs={n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in names},
                basis=original,rarePeakN=np.linalg.norm(fn),routinePeakN=routine["quasistaticInterpolatedPeakN"],
                routineJ=routine["energyJ"],outcomes=outcomes,scope="Matched mass ledger and original governing stiffness, hit, load route and traction. Added ties use a 25.4 × 50.8 × 3.175 mm 6061 tube on horizontal centerlines and ideal rigid end connections. Stock substitution changes stiffness only at fixed total comparison mass. Gross tube weights exclude fasteners. The 1000× case is a limit control. No spatial fit, fatigue, screw engagement or complete frame release is demonstrated.")


if __name__=="__main__":
    decks="--decks" in sys.argv;result=run(Path(sys.argv[1]),decks)
    (HERE/("inset-cassette-decks.json" if decks else "inset-corner-bypass.json")).write_text(json.dumps(result,indent=2)+"\n")
    for k,r in result["outcomes"].items():print(k,round(r["grossAddedTubeKg"],3),[round(v["maxScrewN"],3) for v in r["cases"].values()])
