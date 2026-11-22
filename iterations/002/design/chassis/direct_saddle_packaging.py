"""Check eight removable direct saddles, full module tool access, two inboard sills and both upper crossbar locations."""
import json, math, re, sys
from pathlib import Path
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse

from crossmember_envelope import box,cylinder
from four_corner_packaging import rotated,outline
from port_anchor_fit import cut,oriented
from structural_perimeter import tube_segment

HERE=Path(__file__).resolve().parent
Z=94.6333628767298
RAD=(0,1,0)
UP=(0,0,1)


def join(*parts):
    form=parts[0]
    for item in parts[1:]:
        op=BRepAlgoAPI_Fuse(form,item)
        if not op.IsDone():raise ValueError("Incomplete saddle")
        form=op.Shape()
    return form


def section_shapes(quarter,lower,outside=False,inset=0.,longer=None,relief=False,stagger=False):
    if outside:return exterior_shapes(lower,inset,longer,relief,stagger)
    body=join(box((-23,-51.725,55),(23,-46.725,Z)),
              box((-23,-21.325,55),(23,-9.325,Z)),
              box((-23,-51.725,Z),(23,0,101.6)),
              box((-23,-31.75,101.6),(23,-25.4,158.75)))
    bar=box((-22,-22.225,112),(22,-10.225,158))
    pieces={}
    tools={}
    for ds in (-13,13):
        for h in (65.,85.):
            body=cut(body,cylinder((ds,-52,h),RAD,2.65,31.))
            body=cut(body,cylinder((ds,-21.4,h),RAD,2.05,12.2))
            sleeve=cut(cylinder((ds,-46.725+.04,h),RAD,4.,25.32),cylinder((ds,-47,h),RAD,2.65,26.))
            pieces[f"lower sleeve {ds} {h}"]=sleeve
            pieces[f"lower A351 {ds} {h}"]=oriented(lower,(177.225,265.16295474387533,148.54486287672978),(0,0,-1),(ds,-51.725,h),RAD)
            tools[f"lower driver {ds} {h}"]=cylinder((ds,-51.725,h),(0,-1,0),4.2,74)
        for h in (121.,149.):
            body=cut(body,cylinder((ds,-32.,h),RAD,3.35,7.))
            bar=cut(bar,cylinder((ds,-23,h),RAD,2.5527,14))
            pieces[f"upper A542 {ds} {h}"]=oriented(quarter,(274.815795103,241.078975516,75.583362877),UP,(ds,-31.75,h),RAD)
            tools[f"upper driver {ds} {h}"]=cylinder((ds,-31.75,h),(0,-1,0),5.1,75)
    return {"saddle":body,"upper captive plate":bar,**pieces},tools


def exterior_shapes(lower,inset=0.,longer=None,relief=False,stagger=False):
    # All driver heads face the removable bumper. A receiver machined as part of
    # the inboard leg replaces the upper loose backup plate. A fitted crush sleeve
    # spans both walls at every through-fastener.
    body=join(box((-25,-58.725-inset,55),(25,-46.725-inset,Z)),
              box((-25,-21.325-inset,55),(25,-16.325-inset,Z)),
              box((-25,-58.725-inset,Z),(25,0,101.6)),
              box((-25,-39.4-inset*1.5,101.6),(25,-25.4-inset,159.0)))
    if relief:
        from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
        from OCP.gp import gp_Vec
        from four_corner_packaging import polygon
        low=-58.725-inset
        # Trim the two outside tangential edges of the inboard leg and bridge together.
        # The cutter lies below the ring and the upper tapped support ear.
        for sign in (-1,1):
            triangle=[[sign*15,low-1,54],[sign*26,low-1,54],[sign*26,low+10,54]]
            body=cut(body,BRepPrimAPI_MakePrism(polygon(triangle),gp_Vec(0,0,48)).Shape())
    stock={};tools={}
    for signed in (-13,13):
        for kind,zs,head,tip,shoulder_distance,sleeve_at in (
            ("lower",(65.,85.),-16.325-inset,-46.725-inset,25.4,-21.325-inset),
            ("upper",(121.,149.),0.,-25.4-inset,25.4+inset,0.)):
            for z in zs:
                s=int(math.copysign(6,signed)) if stagger and z==121. else signed
                axis=(0,-1,0)
                if kind=="lower":body=cut(body,cylinder((s,head+.2,z),axis,2.65,6.))
                body=cut(body,cylinder((s,tip+.1,z),axis,2.05,13.+(inset if kind=="upper" else 0.)))
                stock[f"{kind} sleeve {s} {z}"]=cut(cylinder((s,sleeve_at-.04,z),axis,4.,shoulder_distance-.08),
                    cylinder((s,sleeve_at+.1,z),axis,2.65,shoulder_distance+.2))
                sku="A354" if kind=="upper" and inset else "A351"
                source=longer if sku=="A354" else lower
                datum=(224.15961849239,219.15961849239,148.54486287672978) if sku=="A354" else (177.225,265.16295474387533,148.54486287672978)
                stock[f"{kind} {sku} {s} {z}"]=oriented(source,datum,(0,0,-1),
                    (s,head,z),axis)
                tools[f"{kind} driver {s} {z}"]=cylinder((s,head,z),RAD,4.25,75)
    return {"saddle":body,**stock},tools


def hollow(lo,hi,axis,wall=3.175):
    lower=list(lo);upper=list(hi)
    for i in range(3):
        delta=-1 if i==axis else wall
        lower[i]+=delta;upper[i]-=delta
    return cut(box(lo,hi),box(lower,upper))


def run(package,outside=False,inset=0.,relief=False,stagger=False):
    sys.path.insert(0,str(package/"scripts"))
    from fusion_authority import verify,register_path
    from mechanical_motion import bounds,distance,common
    from component_shape import volume
    from showcase_contacts import load_solids,box_gap
    root=HERE.parents[3];reg=register_path(root);lin=reg.parent/"issue-005/lineage.json"
    auth=verify(root,reg,lin)
    allparts,sheets=load_solids(lin.with_name("assembly.step"))
    installed={k:s for k,s in allparts.items() if int(re.search(r"\[1\.(\d+)\.",k)[1])<=8}
    if len(installed)!=488 or sheets:raise ValueError("Wrong installed source")
    quarter=installed[next(k for k in installed if "91251A542" in k and "[1.1.1.20.20.1]" in k)]
    lower=installed[next(k for k in installed if "91251A351" in k and "[1.1.1.2.1]" in k)]
    longer=installed[next(k for k in installed if "91251A354" in k and "[1.1.1.3.1]" in k)]
    from structural_perimeter import relocate_lower
    rail_defs=json.loads((HERE/"socket-holes.json").read_bytes())["rails"]
    installed=relocate_lower(installed,rail_defs,inset)
    stock,tools=section_shapes(quarter,lower,outside,inset,longer,relief,stagger)
    extents={k:bounds(p) for k,p in installed.items()}
    centers={"port":(0,365,0),"front":(370,0,-90),"starboard":(0,-365,180),"rear":(-370,0,90)}
    attachments={};access_cases={};hardware_near={}
    masses={}
    for tag,s in stock.items():
        density=8000 if any(s in tag for s in ("A351","A542","A354")) else 7860 if tag=="upper captive plate" else 2700
        masses[tag]=volume(s)*density*1e-9
    def measured_near(part,items,threshold=.3):
        own=bounds(part);hits=[]
        for k,(other,bb) in items.items():
            if box_gap(own,bb)>threshold:continue
            gap=distance(part,other)
            if gap>threshold:continue
            v=common(part,other) if gap<.0001 else 0
            hits.append(dict(id=k,gapMm=gap,commonMm3=v))
        return hits
    original={k:(s,extents[k]) for k,s in installed.items()}
    shop={}
    ring=[]
    for i in range(6):
        ring.append(tube_segment(outline(370,365,42,0),outline(370,365,42,25.4+inset),i,25.4+inset,3.175,101.6,101.6))
    sill_len=330.6-1.5*inset if outside else 338.25
    sill_y=128.-inset if outside else 140.
    for y in (-sill_y,sill_y):
        shop[f"longitudinal sill {y}"]=hollow((-sill_len,y-12.7,111.125),(sill_len,y+12.7,161.925),0)
    for x in (110.,250.):
        shop[f"upper transverse at {x}"]=hollow((x-25.4,-242.,161.925),(x+25.4,242.,187.325),1)
    for side,(x,y,angle) in centers.items():
        for center in (-140+inset,140-inset):
            transform=lambda s:rotated(rotated(s,0,(center,0,0)),math.radians(angle),(x,y,0))
            name=f"{side}/{center}"
            attachments[name]={}
            for tag in (("saddle",) if outside else ("saddle","upper captive plate")):
                form=transform(stock[tag]);shop[name+"/"+tag]=form
                attachments[name][tag]=measured_near(form,original,5 if relief else .3)
            if stagger:
                # Keep deliberate bore cuts confined to the selected tube. New screws
                # and sleeves must miss every other module, sill and crossing bar.
                nearby={k:v for k,v in original.items() if k.count('.')>2}
                nearby.update({k:(p,bounds(p)) for k,p in shop.items() if k.startswith(('longitudinal','upper transverse'))})
                for tag,item in stock.items():
                    if tag=='saddle':continue
                    form=transform(item)
                    hardware_near[name+'/'+tag]=measured_near(form,nearby,2.)
            for tag in (tools if outside else ("lower driver -13 65.0","lower driver 13 85.0","upper driver -13 121.0","upper driver 13 149.0")):
                form=transform(tools[tag]);tools_key=name+"/"+tag
                tool_items=dict(original)
                tool_items.update({k:(s,bounds(s)) for k,s in shop.items() if k.startswith(("longitudinal","upper transverse"))})
                access_cases[tools_key]=measured_near(form,tool_items)
    supplementary={}
    original_tools={}
    for rail in json.loads((HERE/"socket-holes.json").read_bytes())["rails"].values():
        for x,y in rail["boltCentersMm"]:
            x-=math.copysign(inset,x);y-=math.copysign(inset,y)
            gauge=cylinder((x,y,Z+.01),UP,4.15,82.)
            original_tools[f"module tool at {x} {y}"]=(gauge,bounds(gauge))
    for name,body in shop.items():
        if name.startswith(("longitudinal","upper transverse")):
            supplementary[name]={"kg":volume(body)*2700e-9,
                "retainedBRep":measured_near(body,original),"newUpperToolConflicts":measured_near(body,original_tools)}
        if name.endswith("/saddle"):
            attachments[name.split("/saddle")[0]]["moduleTools"]=measured_near(body,original_tools)
    joinery={}
    for name,body in shop.items():
        if name.startswith(("longitudinal","upper transverse")):
            joinery[name]=measured_near(body,{k:(s,bounds(s)) for k,s in shop.items() if k!=name},.02)
    return dict(authority=auth,moduleInsetXYmm=inset,lowerForkCornerReliefMm=9 if relief else 0,retainedStock=488,cartridgesInHistoricalMaster=64,eachSaddleGroupGrossKg=sum(masses.values()),
        individualComponentsKg=masses,grossRingKg=sum(volume(s)*2700e-9 for s in ring),attachments=attachments,
        hardwareNearUnmachinedNeighbors=hardware_near,staggerUpperRowMm=6 if stagger else 13,service=access_cases,sillsAndUpperSupport=supplementary,newJoinery=joinery,
        scope="Feasibility on original wheel axes and parked 552 geometry. Source screws are exact envelopes, mass and procurement identity; original A351-to-motor bottoming is an independent OEM joint. Future rails have four new transverse sleeve holes per saddle and retain separate module screw axes. First check excludes deliberate lower rail drill stock and future tap volume from pass credit. New inner mounting-plate and ring have no open approval. Bare receiver strength, tool turn and fastener removal from this assembled support require the next checks. Inboard sills and crossbars require positive joints, torque and loaded pan checks, including source part alignment in the complete Fusion successor.")

if __name__=="__main__":
    outside="--exterior" in sys.argv
    inset=12.7 if '--inset' in sys.argv else 0.
    relief='--relieved' in sys.argv
    stagger='--stagger' in sys.argv
    if stagger and not (relief and inset and outside):raise ValueError("Stagger study requires inset, relieved exterior saddle")
    report=run(Path(sys.argv[1]),outside,inset,relief,stagger)
    (HERE/("inset-saddle-staggered.json" if stagger else "inset-saddle-relieved.json" if relief else "inset-saddle-exterior.json" if inset else "direct-saddle-exterior.json" if outside else "direct-saddle-packaging.json")).write_text(json.dumps(report,indent=2)+"\n")
    print("mass",report["eachSaddleGroupGrossKg"],report["grossRingKg"],{k:r["kg"] for k,r in report["sillsAndUpperSupport"].items()})
    for k,r in report["sillsAndUpperSupport"].items():print(k,[(a["id"],round(a["commonMm3"],1)) for a in r["retainedBRep"] if a["commonMm3"]>.01])
    print("tool conflicts",sum(any(n["commonMm3"]>.01 for n in v) for v in report["service"].values()),"of",len(report["service"]))
