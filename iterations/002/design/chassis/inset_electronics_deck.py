"""Connect two perforated equipment decks in the horizontal plane between the sills and ring."""
import json,hashlib
from pathlib import Path
import numpy as np
from diagonal_lower_elastic import planar_point

HERE=Path(__file__).resolve().parent


def definition(keyed=False,truss=False):
    pack=json.loads((HERE/"inset-electrical-package.json").read_bytes())
    source=json.loads((HERE/"inset-deck-holes.json").read_bytes())
    if pack["authority"]!=source["authority"]:raise ValueError("Stale apparatus")
    bracket=HERE.parent/"private-inputs/electrical/REV-21-2151.STEP"
    if hashlib.sha256(bracket.read_bytes()).hexdigest()!="f257b9d7e5633e153df2f2079f601e9ace6a5607d076107ca2970a2ded00db07":raise ValueError("Unregistered bracket")
    decks={}
    for sign in (1,-1):
        holes=[];windows=[]
        for name,row in pack["candidatePlacements"].items():
            b=np.array(row["boundsMm"]);c=(b[:2]+b[3:5])/2
            if c[1]*sign<0:continue
            if row["source"]=="SPARK MAX":
                if truss and c[0]>100:c[0]=178.
                # 2022 REV-21-2151 drawing three-hole pattern, verified on the original STEP.
                for dx,dy in ((-12.7,25.4),(12.7,25.4),(0,-25.4)):
                    holes.append((name,c[0]+dx,sign*(c[1]+dy),2.65))
            if name=="Power distribution":
                group=[g[0] for g in source["subjects"][name]["features"] if abs(g[0]["radiusMm"]-2.5)<.01]
                if len(group)!=4:raise ValueError("PDH mounting holes changed")
                holes.extend((name,r["xyMm"][0],sign*r["xyMm"][1],2.65) for r in group)
            if name in ("Control computer","Radio","Main 120 A breaker"):
                # Removable carrier bridges the exposed rectangle without being
                # counted as a frame brace. It supports only its individual device.
                y0,y1=sorted((sign*b[1],sign*b[4]))
                windows.append((name,b[0]-1,y0-1,b[3]+1,y1+1))
                end_margin=40. if keyed and name=="Control computer" else 9.
                for xx in (b[0]-8,b[3]+8):
                    for yy in (y0+end_margin,y1-end_margin):holes.append((name+" carrier",xx,yy,2.65))
        if truss:
            members=json.loads((HERE/"inset-pan-truss.json").read_bytes())["stock"]
            for idx,member in enumerate(members):
                if member["side"]!=sign:continue
                a,b=np.array(member["start"]),np.array(member["end"])
                if a[2]==b[2]:continue
                samples=np.array([a+(b-a)*(z-a[2])/(b[2]-a[2]) for z in (203.2,205.3844)])
                # Stay outside the enclosing square-tube capsule even for a sloped member.
                slope=(b-a)[:2]/(b[2]-a[2])
                margin=19.05/np.sqrt(2)*np.sqrt(1+slope**2)+2.
                u=samples[:,:2].copy();u[:,1]*=sign
                lo=u.min(axis=0)-margin;hi=u.max(axis=0)+margin
                windows.append((f"Pan strut passage {idx}",*lo,*hi))
        if truss=="brace-access":
            for xx,yy in ((234.,132.),(266.,158.)):
                holes.append(("Pan landing screw withdrawal",xx,yy,6.35))
        decks[sign]=dict(limits=[-263.,68.,282.,365.] if keyed else [-263.,72.,260.,357.],holes=holes,windows=windows)
    return decks


def plate(model,xy,stiffness,grid=18.,thickness=2.,with_cutouts=True,keyed=False,contact=False,truss=False,recover=False):
    width=len(model["base"]);addition=np.zeros((width,width));results=[]
    for sign,spec in definition(keyed,truss).items():
        supports=[]
        # Outer station bears directly on the top of the ring; use a real mesh/body
        # datum. Inner fasteners reach the sill through individual formed pedestals.
        yc=abs(model["sideCenters"][1][1])
        ring=[(3*i,p) for i,p in enumerate(model["nodes"]) if abs(p[1]-yc*sign)<1e-5]
        for xx in (-228.,-110.,110.,222.):
            d,p=min(ring,key=lambda e:abs(e[1][0]-xx))
            y=340. if keyed else abs(p[1])
            supports.append((p[0],y,planar_point(width,d,[0,sign*y-p[1]])))
        for xx in (-100.,110.,250.):
            d,= [i for i,p in xy.items() if np.linalg.norm(p-[xx,115.3*sign])<.001]
            y=95. if keyed else 83.
            target=(-240. if sign<0 else -227.) if keyed and xx==-100. else xx
            sill=sorted([(idx,p) for idx,p in xy.items() if abs(p[1]-115.3*sign)<.001],key=lambda row:row[1][0])
            if keyed and xx in (110.,250.):
                # These two pins are physically on the upper crossmember roof.
                # Its ends are already carried by the sills, so it needs a
                # 15.875 mm spacer instead of a 41.275 mm cantilever pedestal.
                siblings=sorted([(idx,p) for idx,p in xy.items() if abs(p[0]-xx)<.001 and any(abs(abs(p[1])-v)<.001 for v in (115.3,242.))],key=lambda row:row[1][1])
                (i,a),(j,b)=next((a,b) for a,b in zip(siblings,siblings[1:]) if a[1][1]<=y*sign<=b[1][1])
                pt=np.array([xx,y*sign]);ratio=(pt[1]-a[1])/(b[1]-a[1])
                host=(1-ratio)*planar_point(width,i,pt-a)+ratio*planar_point(width,j,pt-b)
            elif target==xx:
                host=planar_point(width,d,[0,(y-115.3)*sign])
            else:
                (i,a),(j,b)=next((a,b) for a,b in zip(sill,sill[1:]) if a[1][0]<=target<=b[1][0])
                ratio=(target-a[0])/(b[0]-a[0]);pt=np.array([target,y*sign])
                host=(1-ratio)*planar_point(width,i,pt-a)+ratio*planar_point(width,j,pt-b)
            supports.append((target,y,host))
        x0,y0,x1,y1=spec["limits"]
        # Regular CAD grid, augmented at each finite attachment. A complete cell is
        # removed for each small appliance hole; the plate therefore has visibly
        # less load-carrying material than the drilled shop geometry.
        def refined(lo,hi,points):
            hard=sorted(set([lo,hi]+points));values=[]
            for a,b in zip(hard,hard[1:]):values.extend(np.linspace(a,b,int(np.ceil((b-a)/grid))+1)[:-1])
            return np.array(values+[hi])
        xs=refined(x0,x1,[a for a,b,h in supports]);ys=refined(y0,y1,[b for a,b,h in supports])
        pts=np.array([(x,y) for x in xs for y in ys]);nd=2*len(pts);B=np.zeros((nd,nd))
        E,nu=68900.,.33
        D=E/(1-nu*nu)*np.array([[1,nu,0],[nu,1,0],[0,0,(1-nu)/2]])
        area=0.;tris=[]
        for ix,(a,b) in enumerate(zip(xs,xs[1:])):
            for iy,(c,d) in enumerate(zip(ys,ys[1:])):
                if with_cutouts:
                    if any(a<xhi and b>xlo and c<yhi and d>ylo for _,xlo,ylo,xhi,yhi in spec["windows"]):continue
                    if any(np.hypot(np.clip(x,a,b)-x,np.clip(y,c,d)-y)<r for _,x,y,r in spec["holes"]):continue
                idx=ix*len(ys)+iy
                corners=[idx,idx+len(ys),idx+len(ys)+1,idx+1]
                for tri in ((corners[0],corners[1],corners[2]),(corners[0],corners[2],corners[3])):
                    corners_xy=pts[list(tri)];tarea=(b-a)*(d-c)/2
                    loc=np.linalg.inv(np.column_stack([np.ones(3),corners_xy]))
                    strains=np.zeros((3,6));strains[0,::2]=loc[1];strains[1,1::2]=loc[2];strains[2,::2]=loc[2];strains[2,1::2]=loc[1]
                    dd=np.array([[2*i,2*i+1] for i in tri]).ravel()
                    B[np.ix_(dd,dd)]+=thickness*tarea*(strains.T@D@strains);area+=tarea
                    tris.append((dd,D@strains))
        support_idx=[int(np.argmin(np.linalg.norm(pts-[a,b],axis=1))) for a,b,h in supports]
        sd=np.array([[2*i,2*i+1] for i in support_idx]).ravel()
        active=np.where(np.diag(B)>.0001)[0];inter=np.setdiff1d(active,sd)
        if len(set(sd)-set(active)):raise ValueError("Fastener joins no retained sheet")
        interior_from_support=-np.linalg.solve(B[np.ix_(inter,inter)],B[np.ix_(inter,sd)])
        condensed=B[np.ix_(sd,sd)]+B[np.ix_(sd,inter)]@interior_from_support
        H=np.vstack([np.diag([1,sign])@h for a,b,h in supports]);W=stiffness*np.eye(len(sd))
        response=np.linalg.solve(W+condensed,W@H);force=W@(H-response)
        addition+=H.T@force
        # Count neighboring passages only once where the two root struts converge.
        xs_area=sorted({x for _,x,_,_,_ in spec["windows"]}|{x for _,_,_,x,_ in spec["windows"]})
        opening_area=0.
        for xa,xb in zip(xs_area,xs_area[1:]):
            intervals=sorted((ya,yb) for _,x,ya,xx,yb in spec["windows"] if x<(xa+xb)/2<xx)
            top=-float("inf")
            for ya,yb in intervals:
                opening_area+=(xb-xa)*max(0.,yb-max(top,ya));top=max(top,yb)
        raw_area=(x1-x0)*(y1-y0)-opening_area-sum(np.pi*r*r for _,x,y,r in spec["holes"])
        register_area=36.2*14.2-(4-np.pi)*2.1**2 if keyed else np.pi*2.65**2
        raw_area-=len(supports)*register_area
        results.append(dict(sign=sign,supports=[[float(x),sign*float(y)] for x,y,h in supports],holes=len(spec["holes"]),
            windows=spec["windows"],stiffAreaMm2=area,nominalDrilledBlankKg=raw_area*thickness*2.7e-6,force=force))
        if recover:
            full=np.zeros((nd,len(sd)))
            full[sd]=np.eye(len(sd))
            full[inter]=interior_from_support
            kept=np.flatnonzero(np.diag(B)[::2]>.0001)
            results[-1]["activePoints"]=pts[kept]
            results[-1]["nodalFromSupport"]=full.reshape(-1,2,len(sd))[kept]
        if contact:
            results[-1]["hostMap"]=H
            results[-1]["contactStiffness"]=W-W@np.linalg.solve(condensed+W,W)
    return addition,results
