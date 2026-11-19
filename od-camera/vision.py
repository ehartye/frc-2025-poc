"""Compare installed image evidence from the roof/field and fixed stereo heads for each OD transfer."""
from functools import cache
import numpy as np
from PIL import Image
from PIL import ImageFilter
def binary_erosion(a):
    return np.asarray(Image.fromarray(a.astype(np.uint8)).filter(ImageFilter.MinFilter(3)),dtype=bool)
def binary_dilation(a,iterations):
    return np.asarray(Image.fromarray(a.astype(np.uint8)).filter(ImageFilter.MaxFilter(2*iterations+1)),dtype=bool)
from departure_review import *
import optics
import imaging as camera
from observe import read_pair
from OCP.gp import gp_Trsf
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
sys.path.insert(0,str(ROOT/"mission"))
from check_branch_exclusion import direct_children,name,FIRST_SHA
from native_station import parts as station_parts

def transform(shape,vals):
    tr=gp_Trsf();tr.SetValues(*vals)
    return BRepBuilderAPI_Transform(shape,tr,False).Shape()

def field_source(path):
    if sha(path)!=FIRST_SHA:raise ValueError("Unregistered field")
    doc,table=cad.xcaf_doc();reader=cad.STEPCAFControl_Reader()
    if reader.ReadFile(str(path))!=cad.IFSelect_RetDone or not reader.Transfer(doc):raise ValueError("Field import")
    roots=cad.Sequence_TDF_Label();table.GetFreeShapes(roots)
    occ,ref=next((o,r) for o,r in direct_children(table,roots.Value(1)) if name(r)=="GE-25300_Red")
    if name(occ)!="NAUO36":raise ValueError("Unregistered reef")
    loc=table.GetLocation_s(occ)
    reef={name(o):table.GetShape_s(child).Moved(loc.Multiplied(table.GetLocation_s(o))) for o,child in direct_children(table,ref)}
    return station_parts(path),reef

def field_at(field,c,station):
    if station:
        angle=math.pi+math.radians(c.station_yaw-c.yaw)
        co,si=math.cos(angle),math.sin(angle)
        x,y=474.525+c.normal*1000,c.lateral*1000
        return {n:transform(s,(co,si,0,-co*x-si*y,-si,co,0,si*x-co*y,0,0,1,0)) for n,s in field[0].items()}
    origin=3453.050980729168+3000-1000*c.target_standoff+1000*c.target_error
    # Official field axes are X forward, Y up, Z opposite robot left.
    return {n:transform(s,(1,0,0,-origin,0,0,-1,-c.sector*1000,0,1,0,0)) for n,s in field[1].items()}

def camera_setup(head,stroke):
    x,y,z=EYE if head=="roof" else (-315.,0.,970.)
    angle=math.radians(90+PITCH if head=="roof" else 26.)
    C,S=math.cos(angle),math.sin(angle)
    camera.ORIGIN=np.array([x+(stroke if head=="roof" else 0),y,z])
    camera.C,camera.S=C,S
    camera.DX,camera.DY,camera.DZ=C+camera.Q*S,-(camera.U-camera.W/2)/camera.F,-S+camera.Q*C

def scene(parts):
    out=[]
    for k,shape in parts.items():
        v,f=whole.original.triangulate(shape)
        if not f:continue
        out.append([k,"native","#83918f",v,f])
    return out

def face_masks(background,pos,yaw,c):
    # Finite hollow caps, opaque cylinder sidewall, actual OD/length and CAD depth in mm.
    dx,dy,dz=camera.DX,camera.DY,camera.DZ
    a=math.radians(-yaw);axis=np.array([math.sin(a),math.cos(a),0.])
    dot=dx*axis[0]+dy*axis[1];length2=dx*dx+dy*dy+dz*dz;den=length2-dot*dot
    R=500*c.diameter;half=500*c.length;result=[]
    for eye,(_,depth) in zip(camera.EYES,background):
        origin=camera.ORIGIN+np.array([0,eye,0])-np.asarray(pos)
        along=origin@axis;cross=dx*origin[0]+dy*origin[1]+dz*origin[2]-dot*along
        disc=cross*cross-den*(origin@origin-along*along-R*R);near=(-cross-np.sqrt(np.maximum(0,disc)))/den
        best=np.where((disc>=0)&(near>0)&(abs(along+near*dot)<=half),near,np.inf)
        for end in (-half,half):
            with np.errstate(divide="ignore",invalid="ignore"):
                dist=(end-along)/dot
                rho=origin@origin+2*dist*(cross+dot*along)+dist*dist*length2-half*half
            best=np.minimum(best,np.where((dist>0)&(rho>=(R-6.35)**2)&(rho<=R*R),dist,np.inf))
        result.append((best<depth,np.isfinite(best)))
    return result

def observations(background,pos,yaw,c,head,stroke):
    obs=face_masks(background,pos,yaw,c)
    images=[np.where(v,218,g).astype(np.uint8) for (v,_),(g,_) in zip(obs,background)]
    geometry=[]
    for (mask,ideal) in obs:
        contour=ideal&~binary_erosion(ideal)
        geometry.append({"pipePixels":int(mask.sum()),"possibleInFrame":int(ideal.sum()),
                         "visibleContourFraction":float((contour&mask).sum()/max(1,contour.sum())),
                         "idealClipped":bool(np.any(ideal[[0,-1]]) or np.any(ideal[:,[0,-1]]))})
    if head=="fixed":legacy=read_pair(images,[g for g,_ in background],560,stroke)["status"]
    else:legacy="different optical transform and OD seat, previous shelf-based policy inapplicable"
    # Each hypothesis is rendered independently with exactly the same registered optics and occluders.
    # Count only differences surviving two pixels' edge erosion. Unknown poses outside this finite set remain possible.
    alternatives=[]
    for label,delta,dyaw in (("forward-20",(20,0,0),0),("rear-20",(-20,0,0),0),("left-12",(0,12,0),0),
                             ("right-12",(0,-12,0),0),("below-15",(0,0,-15),0),("yaw-plus4",(0,0,0),4)):
        alt=face_masks(background,np.array(pos)+delta,yaw+dyaw,c)
        dif=[]
        for (a,_),(b,_) in zip(obs,alt):
            mismatch=(a&~binary_dilation(b,iterations=2))|(b&~binary_dilation(a,iterations=2))
            dif.append(int(mismatch.sum()))
        alternatives.append([label,dif])
    return {"fov":geometry,"baselineFixedEstimator":legacy,"twoPixelDiscriminators":dict(alternatives)},images

def run(kit,field):
    info={};panels=[];static={};station,reef=field
    for name,c in cases().items():
        res=motion.run(c,True);tr=res["trace"]
        for label,t in (("seat",.92),("still-held",3.49),("sectional-exit",res["eventsS"]["exit"]),("later",3.75),("front-max",3.950 if name=="center" else 3.974),("held-clear",3.678 if name=="center" else 4.308),
                        ("terminal",tr[-1][0])):
            row=min(tr,key=lambda r:abs(r[0]-t));at,x,z,_,_,_,left,right,_=row
            is_station=label=="seat";stroke=30 if is_station else 370
            key=(name,stroke)
            if key not in static:
                original_robot=baseline(stroke,kit)
                originals={k:s for k,s in original_robot.items() if k not in retired(original_robot) and not k.startswith("roof-")}
                structure_at={k:cad.translated(s,(stroke,0,0)) for k,s in structure(kit[0]).items()}
                visible_field=field_at(field,c,is_station)
                static[key]=scene(originals|structure_at|{"field-"+k:v for k,v in visible_field.items()})
            pieces={k:s for k,s in gate.shapes(stroke,1000*left-85,0,-1000*right-85).items() if k not in GATE_REMOVE}
            moving=scene(pieces)
            for head in ("fixed","roof"):
                camera_setup(head,stroke)
                skip="observer-camera-proxy" if head=="fixed" else "roof-camera-envelope"
                selected=[(["fixed-housing" if p[0]=="observer-camera-proxy" else p[0],*p[1:]]) for p in static[key] if p[0]!=skip]
                bg=camera.images_for_pose({"parts":selected+moving})
                pos=[stroke+1000*x,1000*c.lateral,1000*z]
                obs,images=observations(bg,pos,c.yaw,c,head,stroke);info[name+"-"+label+"-"+head]={"t":at,"position":pos,**obs}
                if name=="center" and label in ("seat","terminal"):
                    thumb=Image.fromarray(images[0]).convert("RGB").resize((240,180));panels.append((label,head,thumb))
    from PIL import ImageDraw
    sheet=Image.new("RGB",(480,420),"white");pen=ImageDraw.Draw(sheet)
    for j,(phase,head,img) in enumerate(panels):
        x=(j%2)*240;y=(j//2)*210;pen.text((x+5,y+3),phase+" / "+head,fill="black");sheet.paste(img,(x,y+25))
    preview = ROOT / "artifacts" / "od-camera" / "registered-views.png"
    preview.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(preview)
    return {"fieldSha":FIRST_SHA,"nativeSha":sha(HERE/"revised-bridge.step"),"stationParts":len(station),"reefParts":len(reef),
            "cases":info,"scope":"Synthetic unmarked images from actual CAD plus registered field. Finite comparisons, no physical measurement or general pose observability."}
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--field",type=Path,required=True);p.add_argument("--lamp",type=Path,required=True)
    a=p.parse_args();(HERE/"images.json").write_text(json.dumps(run(source(a.lamp),field_source(a.field)),indent=2)+"\n",encoding="utf8",newline="\n")
