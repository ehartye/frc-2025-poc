"""Check whether real module corner stock can accommodate extended sockets and positive fitted bushings."""
import hashlib, json, math, re, sys
from pathlib import Path

from crossmember_envelope import cylinder
from direct_saddle_packaging import hollow
from port_anchor_fit import cut
from structural_perimeter import relocate_lower

HERE=Path(__file__).resolve().parent


def run(package):
    sys.path.insert(0,str(package/"scripts"))
    from mechanical_motion import bounds,distance,common
    from component_shape import volume
    from showcase_contacts import load_solids,box_gap
    from fusion_authority import register_path,verify
    root=HERE.parents[3]
    reg=register_path(root);line=reg.parent/"issue-005/lineage.json"
    auth=verify(root,reg,line)
    parts,sheets=load_solids(line.with_name("assembly.step"))
    if sheets or len(parts)!=552:raise ValueError("Unexpected issue")
    inputs=HERE/"inset-stagger-all-contacts.json"
    loads=json.loads(inputs.read_bytes());info=json.loads((HERE/"socket-holes.json").read_bytes())["rails"]
    if loads["authority"]!=auth:raise ValueError("Wrong source for support reactions")
    base={k:v for k,v in parts.items() if int(re.search(r"\[1\.(\d+)\.",k)[1])<=8}
    moved=relocate_lower(base,info,12.7)
    modules={k:(v,bounds(v)) for k,v in moved.items() if "socket rail [" not in k}
    if len(modules)!=484:raise ValueError("Incomplete module set")
    rows=[];extensions=[];extra_mass=0.
    for name,defn in info.items():
        axis=defn["lengthAxis"];lo=defn["minimumMm"].copy();hi=defn["maximumMm"].copy()
        lo[axis]+=12.7;hi[axis]-=12.7
        radial=1-axis
        sign=math.copysign(1,lo[radial]);lo[radial]-=sign*12.7;hi[radial]-=sign*12.7
        for end in (-1,1):
            minimum=lo.copy();maximum=hi.copy()
            if end<0:maximum[axis]=minimum[axis];minimum[axis]-=4.
            else:minimum[axis]=maximum[axis];maximum[axis]+=4.
            solid=hollow(minimum,maximum,axis);bb=bounds(solid)
            close=[]
            for key,(other,ob) in modules.items():
                if box_gap(bb,ob)>.5:continue
                gap=distance(solid,other)
                v=common(solid,other) if gap<.0001 else 0.
                close.append(dict(article=key,gapMm=gap,commonMm3=v))
            kg=volume(solid)*2.7e-6
            extra_mass+=kg
            extensions.append(dict(rail=name,end=end,kg=kg,near=close))
        positions=[[x-math.copysign(12.7,x),y-math.copysign(12.7,y)] for x,y in defn["boltCentersMm"]]
        distances=sorted(set(round(abs(pt[axis]),4) for pt in positions))
        for pt in positions:
            rank=("inner","middle","outer")[distances.index(round(abs(pt[axis]),4))]
            if rank=="middle":continue
            d=11.95 if rank=="inner" else 8.95
            lower=lo[2];upper=hi[2]
            sleeve=cut(cylinder((*pt,lower+.0375),(0,0,1),d/2,upper-lower-.075),
                       cylinder((*pt,lower-1),(0,0,1),5.30/2,upper-lower+2))
            bb=bounds(sleeve);near=[]
            for key,(other,ob) in modules.items():
                if box_gap(bb,ob)>.3:continue
                gap=distance(sleeve,other)
                near.append(dict(article=key,gapMm=gap,commonMm3=common(sleeve,other) if gap<.0001 else 0.))
            original=fitted=0.
            for key,part in moved.items():
                if "socket rail [" not in key or box_gap(bb,bounds(part))>.01:continue
                original=common(sleeve,part)
                host=cut(part,cylinder((*pt,lower-1),(0,0,1),(d+.05)/2,upper-lower+2))
                fitted=common(sleeve,host)
                break
            if original<1 or fitted>.002:raise ValueError("Machining failed its matched controls")
            kg=volume(sleeve)*2.7e-6
            extra_mass+=kg
            end_clear=min(pt[axis]-lo[axis],hi[axis]-pt[axis])+4-(d+.15)/2-.25
            force=loads["peaks"][rank+"ScrewForceUpperN"]["value"]
            along=loads["peaks"][rank+"ScrewAlongAbsN"]["value"]
            across=loads["peaks"][rank+"ScrewAcrossAbsN"]["value"]
            rows.append(dict(rail=name,rank=rank,xyMm=pt,odMm=d,idMm=5.30,grossKg=kg,near=near,
                preDrillCommonMm3=original,postDrillCommonMm3=fitted,endLigamentMm=end_clear,
                workingWallBearingMPa=.6*force/(3*(d-.025)),workingSleeveShearMPa=.6*force/(math.pi/4*((d-.025)**2-5.35**2)),
                alongEndTearMPa=.6*along/(2*3*end_clear),acrossSideTearMPa=.6*across/(2*3*(12.7-(d+.15)/2-.25)),
                oneWallBearingMPa=force/(3*(d-.025))))
    return dict(authority=auth,loadsSha256=hashlib.sha256(inputs.read_bytes()).hexdigest(),extensions=extensions,
                sleeves=rows,addedGrossKg=extra_mass,
                scope="Unissued candidate with 4 mm more purchased tube at each end and 8 inner/8 outer machined bushings. Native cold acceptance, actual fitted diameters, tube wall share (60/40), sleeve bending, pin shaft strength, clamp and joint alignment are still open. A conservative single-wall negative and material shear screen 120/sqrt(3) MPa accompany a 120 MPa bearing proxy. No mass credit for removed material in this report. Existing prior eight inner bushings must be removed from the mass account before adding the complete new roster.")


if __name__=="__main__":
    result=run(Path(sys.argv[1]))
    (HERE/"inset-extended-sockets.json").write_text(json.dumps(result,indent=2)+"\n")
    print("kg",result["addedGrossKg"])
    print("end common",max(i["commonMm3"] for r in result["extensions"] for i in r["near"]))
    print("sleeve neighbor common",max(i["commonMm3"] for r in result["sleeves"] for i in r["near"]))

