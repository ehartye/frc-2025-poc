"""Cut and measure the candidate serviceable pan feet with the four actual quarter-twenty screws."""
import copy,hashlib,json,sys
from pathlib import Path
import numpy as np
from inset_chassis_shop import HERE,op,circ
from inset_stock_check import form,run as stock
from port_anchor_fit import oriented
from OCP.gp import gp_Dir,gp_Ax2,gp_Pnt
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder

def revised(articles,purchases,through_radius=3.3):
    articles=copy.deepcopy(articles);purchases=copy.deepcopy(purchases)
    for sign in (-1,1):
        y=145*sign;tag=f"250 {y}";holes=[(234,y-sign*13),(266,y+sign*13)]
        foot=articles["Truss landing "+tag];back=articles["Captive truss landing nutbar "+tag]
        for body,half in ((foot,21.),(back,20.)):
            box=body["steps"][0]["profiles"][0]["bounds"];box[1]=y-half;box[3]=y+half
        host=next(f for f in articles["Pan crossmember 250"]["steps"] if f["name"]=="Brace foot key and bolts "+tag)
        for shape in (foot["steps"][-1],back["steps"][-1],host):
            for p,point in zip((p for p in shape["profiles"] if p["kind"]=="circle"),holes):
                p["center"]=list(point)
                if shape is not back["steps"][-1]:p["radius"]=through_radius
        for x,yy in holes:
            screw=next(v for v in purchases if v["name"]==f"Truss hold down {tag} {x}")
            screw["underHead"][1]=yy
        sheet=articles["Equipment sheet "+str(sign)]
        sheet["steps"].append(op("Access the corrected brace pair","cut",2,202.9,2.7,*[circ(x,yy,6.35) for x,yy in holes]))
    return articles,purchases

def run(plugin,through_radius=3.3):
    sys.path.insert(0,str(plugin/"scripts"))
    from fusion_authority import register_path
    from mechanical_motion import bounds,common,distance
    from showcase_contacts import load_solids,box_gap
    from component_shape import volume
    bodies,base=stock(plugin,True);reg=register_path(HERE.parents[3])
    src=json.loads((HERE/"inset-chassis-shop.json").read_bytes());original=json.loads((HERE/"inset-mount-purchases.json").read_bytes())["originals"]["91251A539"]
    donor=load_solids(reg.parent/"issue-005/assembly.step")[0][original["source"]]
    before={r["name"]:r for r in src["articles"]};after,buys=revised(before,src["purchases"],through_radius)
    delta=[];changes={}
    for name,row in after.items():
        if before[name]==row:continue
        original_shape=bodies[name];new=form(row);bodies[name]=new;delta.append((volume(new)-volume(original_shape))*2.7e-6);changes[name]=row
    boxes={k:bounds(v) for k,v in bodies.items()};stock_overlap=[]
    for name in changes:
        for k,body in bodies.items():
            if k==name or box_gap(boxes[k],boxes[name])>.001:continue
            v=common(bodies[name],body)
            if v>.002:stock_overlap.append(dict(a=name,b=k,commonMm3=v))
    tests=[]
    for r in buys:
        if r["name"].startswith(("Truss hold down 250 145","Truss hold down 250 -145")):
            x,y,z=r["underHead"];d=np.array([0.,0.,-1.])
            screw=oriented(donor,original["underHeadCenterMm"],original["intoThreadUnit"],[x,y,z],d)
            def cylinder(start,diameter,height):
                return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x,y,start),gp_Dir(0,0,1)),diameter/2,height).Shape()
            # Above the underhead plane; below it the screw withdraws inside its machined through bore.
            gauges={"installed":screw,"raised head to remove screw":cylinder(z+.001,9.525,24),
                    "engaged driver and straight withdrawal":cylinder(z+3.05,7.36,47)}
            for duty,feature in gauges.items():
                bb=bounds(feature);pairs=[]
                for name,p in bodies.items():
                    g=box_gap(bb,boxes[name])
                    if g>2:continue
                    if name==r["host"] and duty=="installed":
                        pairs.append(dict(name=name,commissionedTap=True));continue
                    vol=common(feature,p);dg=distance(feature,p)
                    pairs.append(dict(name=name,commonMm3=vol,gapMm=dg))
                tests.append(dict(screw=r["name"],axisMm=r["underHead"],duty=duty,pairs=pairs))
    return dict(authority=base["authority"],predecessor=base["shopPlanSHA256"],stockChanges=changes,changedMassKg=sum(delta),
                hardStockOverlaps=stock_overlap,tests=tests,
                scope="Four candidate screws on two changed feet, genuine unchanged A539 and 146 shop solids. Upper brace, both sheet passthroughs, crossmember, keyed landing and nutbars included. Cylinder gauges bound axial screw-head withdrawal and 7.36 mm bit over 47 mm. Connector housing placement, handheld ratchet approach, backer entry and strength are separate.")
if __name__=="__main__":
    selected="--relieved" in sys.argv
    r=run(Path(sys.argv[1]),3.575 if selected else 3.3)
    filename="inset-corrected-brace-foot-clearance.json" if selected else "inset-corrected-brace-foot.json"
    (HERE/filename).write_text(json.dumps(r,indent=1)+"\n")
