"""Generate the common machining definitions for the private four-module chassis fit."""
import hashlib,json,math,sys
from pathlib import Path
import numpy as np
from four_corner_packaging import outline
from inset_electronics_deck import definition
HERE=Path(__file__).resolve().parent


def rect(x0,y0,x1,y1,r=0):
    return dict(kind="rect",bounds=[x0,y0,x1,y1],radius=r)


def circ(u,v,r):
    return dict(kind="circle",center=[u,v],radius=r)


def polygon(points):
    return dict(kind="polygon",points=np.array(points).tolist())


def op(name,kind,axis,start,travel,*profiles):
    return dict(name=name,kind=kind,axis=axis,start=start,travel=travel,profiles=list(profiles))


def boxed(name,lo,hi,axis=2):
    proj=lambda p:[b for j,b in enumerate(p) if j!=axis]
    return op(name,"new",axis,lo[axis],hi[axis]-lo[axis],rect(*proj(lo),*proj(hi)))


def tube(name,lo,hi,axis,w=3.175):
    a=np.delete(lo,axis)+w;b=np.delete(hi,axis)-w
    return [boxed(name,lo,hi,axis),op("Open stock bore","cut",axis,lo[axis]-.2,hi[axis]-lo[axis]+.4,rect(*a,*b))]


def build(pkg):
    sys.path.insert(0,str(pkg/"scripts"))
    from fusion_authority import register_path,verify
    root=HERE.parents[3];reg=register_path(root);auth=verify(root,reg,reg.parent/"issue-005/lineage.json")
    anchor=json.loads((HERE/"inset-truss-key-register.json").read_bytes())
    if auth!=anchor["authority"]:raise ValueError("Wrong issue")
    shops=[];hosts={};sites=[];purchased=[]
    def article(name,operations,group,pose=None):
        row=dict(name=name,group=group,stock="6061-T6",steps=operations)
        if pose is not None:row["matrixMm"]=pose
        shops.append(row);return operations
    def key(cx,cy,clearance):
        return rect(cx-18-clearance,cy-7-clearance,cx+18+clearance,cy+7+clearance,2+clearance)
    outer=np.array(outline(370.,365.,42.,0));inner=np.array(outline(370.,365.,42.,38.1))
    g1=np.array(outline(370.,365.,42.,3.175));g2=np.array(outline(370.,365.,42.,38.1-3.175))
    for i in range(6):
        j=(i+1)%6;t=outer[j]-outer[i];u=t/np.linalg.norm(t)
        stem=[op("Saw miter stock","new",2,101.6,101.6,polygon([outer[i],outer[j],inner[j],inner[i]])),
              op("Open both mitered ends","cut",2,104.775,95.25,polygon([g1[i]-20*u,g1[j]+20*u,g2[j]+20*u,g2[i]-20*u]))]
        hosts[("rim",i)]=article("Structural perimeter "+str(i),stem,"perimeter")
    z0=43.83336287672979;z1=z0+50.8
    rails=json.loads((HERE/"socket-holes.json").read_bytes())["rails"]
    for name,spec in rails.items():
        axis=spec["lengthAxis"];side=1-axis
        oldlo,oldhi=np.array(spec["minimumMm"]),np.array(spec["maximumMm"])
        pts=np.array(spec["boltCentersMm"]);pts-=12.7*np.sign(pts)
        for sign in (-1,1):
            chosen=pts[pts[:,axis]*sign>0];lo,hi=oldlo.copy(),oldhi.copy()
            for a in (lo,hi):a[side]-=12.7*np.sign(a[side])
            lo[axis],hi[axis]=sorted([sign*94.,sign*(max(abs(oldlo[axis]),abs(oldhi[axis]))-12.7)])
            ops=tube("Saw socket",lo,hi,axis)
            ops.append(op("Three module mounting axes","cut",2,z0-1,52.8,*[circ(x,y,spec["throughDiameterMm"]/2) for x,y in chosen]))
            hosts[("socket",name,sign)]=article(name+(" negative end" if sign<0 else " positive end"),ops,"socket")
    pts=np.array(json.loads((HERE/"inset-corner-bypass.json").read_bytes())["outcomes"]["single-face cassette at 10000"]["kneePointsMm"][0])
    xc,yc=pts[2];x0,y0=pts[4,0]-10,pts[0,1]-10
    for xs in (-1,1):
        for ys in (-1,1):
            for role in ("transverse","longitudinal"):
                def foot(w,e):
                    points=[(xc-w,y0-e),(xc+w,y0-e),(xc+w,yc+w),(xc-w,yc-w)] if role=="transverse" else [(x0-e,yc-w),(xc-w,yc-w),(xc+w,yc+w),(x0-e,yc+w)]
                    return polygon([(xs*x,ys*y) for x,y in points])
                article(f"Cassette {xs} {ys} {role}",[op("Miter cassette tube","new",2,z0,50.8,foot(12.7,0)),op("Open cassette ends","cut",2,z0+3.175,44.45,foot(9.525,1))],"cassette")
    for sign in (-1,1):
        c=sign*115.3;ops=tube("Saw inset sill",[-311.55,c-12.7,111.125],[311.55,c+12.7,161.925],0)
        hosts[("sill",sign)]=article("Inboard sill "+str(sign),ops,"sill")
    for x in (110.,250.):
        ops=tube("Saw pan beam",[x-25.4,-242,161.925],[x+25.4,242,187.325],1)
        if x==250:
            ops.append(op("Relieve instrument sweep at both tips","cut",2,161.825,12.175,
                          rect(x-26,-243,x+26,-231.5),rect(x-26,231.5,x+26,243)))
        hosts[("beam",x)]=article("Pan crossmember "+str(int(x)),ops,"crossmember")
    panels=definition(True,True)
    for s,p in panels.items():
        a,b,c,d=p["limits"]; y0,y1=sorted((s*b,s*d))
        if s>0:rim=0
        else:rim=4
        sheet=article("Equipment sheet "+str(s),[op("Cut equipment blank","new",2,203.2,2.032,rect(a,y0,c,y1))],"panel")
        for name,x0,y0,x1,y1 in p["windows"]:
            y0,y1=sorted([s*y0,s*y1]);sheet.append(op(name,"cut",2,202.9,2.7,rect(x0,y0,x1,y1)))
        sheet.append(op("Drill equipment and separable carriers","cut",2,202.9,2.7,*[circ(x,s*y,r) for _,x,y,r in p["holes"]]))
        register={tuple(r["xyMm"]):r["host"] for r in anchor["rows"] if r["xyMm"][1]*s>0}
        for (x,y),role in register.items():
            sites.append(dict(x=x,y=y,role=role))
            tag=f"{x:g} {y:g}";sheet.append(op("Positive register "+tag,"cut",2,202.9,2.7,key(x,y,.1)))
            cy=s*345.95 if role=="perimeter" else y
            # Cap fasteners pass outside the positive profile, allowing a separately clamped sheet.
            if role=="perimeter":
                capholes=[(x-d,cy) for d in (-24,24)]
                root=hosts[("rim",rim)]
                root.append(op("Seat cassette key "+tag,"cut",2,199.80,3.6,key(x,y,.1)))
                root.append(op("Deck cap bolts "+tag,"cut",2,199.8,3.6,*[circ(u,v,3.3) for u,v in capholes]))
                low=203.2-3.175-10.
                back=[op("Saw inside backing stock","new",2,low,10.,rect(x-33,cy-15.2,x+33,cy+15.2,1.5)),
                      op("Integral positive deck key","join",2,low+10.,3.175+1.78,key(x,y,0))]
                back.append(op("Two quarter-twenty tap drills","cut",2,low-.2,10.4,*[circ(u,v,2.5527) for u,v in capholes]))
                article("Perimeter key and tapped backing "+tag,back,"panel backer")
            else:
                rear=role=="rear sill";roof=161.925 if rear else 187.325
                capholes=[(x-22,y),(x+22,y)] if rear else [(x,y-19),(x,y+19)]
                py=s*115.3 if rear else y
                lowerholes=[(x+a,py+b) for a in (-10,10) for b in ((-7,7) if rear else (-18,18))]
                root=hosts[("sill",s)] if rear else hosts[("beam",x)]
                # Rear mounting saddle bears on the complete sill roof, with both screw rows inside it.
                halfwidth=41 if rear else 28
                xmin,xmax=x-halfwidth,x+halfwidth; ymin=min(y-11,py-(12.7 if rear else 24));ymax=max(y+11,py+(12.7 if rear else 24))
                base=[op("Machine backed pedestal","new",2,roof,203.2-roof,rect(xmin,ymin,xmax,ymax,2)),
                      op("Positive deck shoulder","join",2,203.2,1.78,key(x,y,0)),
                      op("Lower key enters tube roof","join",2,roof,-2.9,key(x,py,0)),
                      op("Two accessible cap taps","cut",2,203.3,-12.4,*[circ(u,v,2.5527) for u,v in capholes]),
                      op("Drill base attachment","cut",2,roof-.2,203.4-roof,*[circ(u,v,3.3) for u,v in lowerholes]),
                      op("Recess lower mount heads","cut",2,203.3,-(203.3-roof-5.),*[circ(u,v,5.15) for u,v in lowerholes])]
                if rear:
                    # Bolts on the edges of the lower positive key must not intersect its sidewalls.
                    # Relocate the joints axially to put each shaft outside that key.
                    lowerholes=[(x+a,py+b) for a,b in ((-34,-2.5),(-22,2.5),(22,-2.5),(34,2.5))]
                    for step,r in ((base[-2],3.3),(base[-1],5.15)):step["profiles"]=[circ(u,v,r) for u,v in lowerholes]
                root.append(op("Positive lower key and mounting bolts "+tag,"cut",2,roof-3.3,3.5,key(x,py,.1),*[circ(u,v,3.3) for u,v in lowerholes]))
                bwx,bwy=(41,9.2) if rear else (20.8,23.8)
                backing=[op("Saw captive tapped nutbar","new",2,roof-3.175-10,10.,rect(x-bwx,py-bwy,x+bwx,py+bwy,1.5)),
                         op("Drill four base taps","cut",2,roof-3.175-10.2,10.4,*[circ(u,v,2.5527) for u,v in lowerholes])]
                article("Keyed pedestal "+tag,base,"panel pedestal")
                article("Captive pedestal nutbar "+tag,backing,"panel backer")
                for u,v in lowerholes:
                    purchased.append(dict(sku="91251A539",name=f"Pedestal base {tag} {u:g} {v:g}",underHead=[u,v,roof+5],direction=[0,0,-1],host="Captive pedestal nutbar "+tag))
            sheet.append(op("Independent cap holes "+tag,"cut",2,202.9,2.7,*[circ(u,v,3.3) for u,v in capholes]))
            footprint=rect(x-13,cy-26,x+13,cy+26,1.5) if role=="upper crossmember" else rect(x-31,cy-12,x+31,cy+12,1.5)
            cap=[op("Cut removable retaining tab","new",2,205.232,3.175,footprint),
                 op("Cap attachment holes","cut",2,205.1,3.5,*[circ(u,v,3.3) for u,v in capholes])]
            article("Deck retaining tab "+tag,cap,"panel retainer")
            for u,v in capholes:
                purchased.append(dict(sku="91251A539",name=f"Equipment retaining screw {tag} {u:g} {v:g}",underHead=[u,v,208.407],direction=[0,0,-1],host=("Perimeter key and tapped backing " if role=="perimeter" else "Keyed pedestal ")+tag))
    from inset_frame_supports import add as add_supports
    purchased+=add_supports(article,hosts)
    for s in (-1,1):
        for x in (110.,250.):
            for absolute in (145.,212.):
                y=absolute*s;tag=f"{x:g} {y:g}"
                holes=[(x-16,y),(x+16,y)]
                support=[op("Machine truss landing foot","new",2,187.325,6.35,rect(x-24,y-16,x+24,y+16,2.)),
                         op("Fitted boss in crossmember","join",2,187.325,-2.95,rect(x-9,y-11,x+9,y+11,2)),
                         op("Clear two anchoring screws","cut",2,187.1,6.8,*[circ(u,v,3.3) for u,v in holes])]
                backup=[op("Saw internal backing bar","new",2,174.15,10,rect(x-21.8,y-15,x+21.8,y+15,1.5)),
                        op("Tap quarter-twenty brace hold-downs","cut",2,174.,10.4,*[circ(u,v,2.5527) for u,v in holes])]
                hosts[("beam",x)].append(op("Brace foot key and bolts "+tag,"cut",2,184.,3.6,rect(x-9.1,y-11.1,x+9.1,y+11.1,2.1),*[circ(u,v,3.3) for u,v in holes]))
                article("Truss landing "+tag,support,"truss feet")
                article("Captive truss landing nutbar "+tag,backup,"truss backing")
                for u,v in holes:
                    purchased.append(dict(sku="91251A539",name=f"Truss hold down {tag} {u:g}",underHead=[u,v,193.675],direction=[0,0,-1],host="Captive truss landing nutbar "+tag))
    from inset_truss_stock import add as add_truss
    add_truss(article)
    return dict(authority=auth,articles=shops,sites=sites,purchases=purchased,inputs={f:hashlib.sha256((HERE/f).read_bytes()).hexdigest() for f in ("inset-truss-key-register.json","inset-pan-truss.json","inset-corner-bypass.json","socket-holes.json")})
if __name__=="__main__":
    plan=build(Path(sys.argv[1]));(HERE/"inset-chassis-shop.json").write_text(json.dumps(plan,indent=1)+"\n")
    print(len(plan["articles"]),len(plan["sites"]))
