"""Compose the release-state robot in millimetres from registered body identities."""
import argparse
import csv
import hashlib
import json
import re
import sys
import subprocess
from pathlib import Path
from OCP.BRep import BRep_Builder
from OCP.BRepTools import BRepTools
from OCP.TopoDS import TopoDS_Shape
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
from OCP.Quantity import Quantity_Color, Quantity_TOC_sRGB
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorGen
from OCP.STEPControl import STEPControl_Reader
from OCP.STEPCAFControl import STEPCAFControl_Reader, STEPCAFControl_Writer
from OCP.TDataStd import TDataStd_Name
from OCP.TCollection import TCollection_ExtendedString
from OCP.collections import Sequence_TDF_Label

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
SIX=ROOT/"experiments/six-wheel"
SW=ROOT/"experiments/swerve-drive"
sys.path.insert(0,str(SIX/"design/station-l1"))
import build_assembly as cad
ORIGINAL_ROLES={p['id']:p['group'] for p in cad.PARTS}
CONFIG="fixed-low-straight-arm-swerve-R3-stowed-red"

def read(path):
    return json.loads(path.read_bytes())

def save(path,obj):
    path.write_text(json.dumps(obj,indent=2,ensure_ascii=False)+"\n",encoding="utf8",newline="\n")

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def named(path):
    reader=STEPCAFControl_Reader()
    doc,table=cad.xcaf_doc()
    if reader.ReadFile(str(path))!=cad.IFSelect_RetDone or not reader.Transfer(doc):
        raise ValueError(path)
    labels=Sequence_TDF_Label()
    table.GetFreeShapes(labels)
    out={}
    for i in range(1,labels.Length()+1):
        label=labels.Value(i)
        name=TDataStd_Name()
        if not label.FindAttribute(TDataStd_Name.GetID_s(),name):
            raise ValueError(f"Unnamed item in {path}")
        key=name.Get().ToExtString()
        if key in out:raise ValueError(key)
        out[key]=table.GetShape_s(label)
    return out

def subsolids(shape):
    ex=TopExp_Explorer(shape,TopAbs_SOLID)
    while ex.More():
        yield ex.Current()
        ex.Next()

def block(b):
    return cad.brep(["box",[*b[:3],*(b[i+3]-b[i] for i in range(3))]])

def tube(origin, axis, radius, length):
    return cad.brep(["cyl",[*origin,*axis,radius,length]])

def envelope(shapes):
    all_bounds=[cad.bounds(s) for s in shapes]
    return [min(row[i] for row in all_bounds) if i<3 else max(row[i] for row in all_bounds) for i in range(6)]

def resolve_project_palette(root):
    """Use the project's default component palette and the student's read-only override."""
    helper = ROOT / "scripts" / "component_palette.py"
    command = [sys.executable, str(helper), str(root)]
    resolved = json.loads(subprocess.check_output(command, encoding="utf8"))
    if resolved.get("schema") != "agent-frc/component-palette@1":
        raise ValueError("Unexpected component palette")
    return resolved


def role(part, source, bumpers):
    s=part.lower()
    if part in bumpers:return bumpers[part]
    if "bumper-cover-" in s:return "bumper-red"
    if part in ORIGINAL_ROLES:return ORIGINAL_ROLES[part]
    if "registration-mark" in s:return "wheel"
    if "np18" in s:return "battery"
    if any(w in s for w in ("screw","bolt","nut","washer","dowel","fastener","thread-boss","clip-","-clip","setting-pin","lock-pin")):return "fastener"
    if any(w in s for w in ("sensor","camera","pupil","observer","sp1-","datum-foot","optic","qtrx","sensing","pressure-","photogate","magnet","hall-","lamp","am-3583")):return "sensing"
    if "tread" in s:return "wheel"
    if "battery" in s:return "battery" if "tray" not in s and "stop" not in s and "cheek" not in s and "hold-down" not in s else "plate"
    if any(w in s for w in ("powerhub","breaker","power-","converter","fuse","pdh","channel","terminal","cable","harness","trough","connector","wire-","usb-","controller","can-","radio")):return "electrical"
    if any(w in s for w in ("host","compute","roborio","board")):return "compute"
    if any(w in s for w in ("bearing","bushing")):return "bearing"
    if "rack" in s or "pinion" in s:return "rack"
    if any(w in s for w in ("belt","420-3m","wheel-grip")):return "belt"
    if any(w in s for w in ("grip","receiver","cheek","absorber","r6-")):return "shelf" if "r6-" in s else "receiver"
    if any(w in s for w in ("guide","liner","wedge","tapped-bar")):return "guide"
    if any(w in s for w in ("lift","mast","crossbar","support-machined","support-l-","support-r-","low-lock")):return "lift-drive"
    if "slide" in s or "sync" in s:return "slide"
    if any(w in s for w in ("shaft","axle","crosspin")):return "shaft"
    if "pulley" in s:return "pulley"
    if any(w in s for w in ("saddle","plate","cradle","tray","foot","shoe","face")):return "plate"
    if "motor" in s or "fork" in s or "gear" in s:return "drive"
    return "frame"

def build(artifacts, local):
    local.mkdir(exist_ok=True,parents=True)
    raw=local/"registered"
    account=read(artifacts/"swerve-package/accounting-source.json")
    retirement=read(SW/"packaging/retirement.json")
    active=set(account["make"]+account["undecided"]+[i for a in account["buy"] for i in a["instances"]])
    active.difference_update(retirement["retired"]+retirement["sameDriveMotorsAndControllers"]+retirement["relocatedInnerEarSet"])
    leaves={v.rsplit("/",1)[-1] for v in active}
    records={}
    restricted={}
    proofs={}
    exclusions={}
    def put(key,shape,source,representation="project-cad",r2=False):
        key=key.replace("/","--")
        records[key]=(shape,source,representation,r2)
    def step(path,select=lambda k:True,r2=False):
        rel=path.relative_to(ROOT).as_posix()
        proofs[rel]=sha(path)
        for key,shape in named(path).items():
            if select(key):
                put(key,shape,rel,r2=r2)
    extras={"bumper-cover-front-red","bumper-cover-rear-red","light-spacer","fixed-camera-cradle-drilled",
            "sp1-supplier-reservation","sp1-eye-reservation","sp1-live-cable"}
    deny={"electrical-backplate","observer-pi-and-converter-backplate","observer-pi4-service-envelope",
          "drive-rail-outer-R","drive-rail-outer-L","drive-rail-inner-L","drive-rail-inner-R",
          "lift-crossbar-drilled","backup-L-drilled-mast","backup-R-drilled-mast",
          "observer-usb-service-reserve","observer-cable-trunk-reserve","wire/data-feed","wire/carrier","wire/power-feed"}
    base="experiments/six-wheel/fixed-camera-route/scene.py"
    proofs[base]=sha(ROOT/base)
    for key,f in read(raw/"index.json").items():
        local_supplier=key.startswith(("roof-head/","fixed-head/")) or key=="am-3583"
        want=(key in leaves or key in extras or key.startswith(("support/","corner-","wire/")) or local_supplier)
        relocated=re.match(r"(front|rear)-[LR]-(ear$|pilot$|weld-|screw-(90|130)$|head-washer-(90|130)$|nut-washer-(90|130)$|nut-(90|130)$)",key)
        if (not want and not relocated) or key in deny:
            exclusions[key]="superseded, retired, service-only or represented in selected overlay"
            continue
        shape=TopoDS_Shape()
        if not BRepTools.Read_s(shape,str(raw/f),BRep_Builder()):raise ValueError(key)
        if relocated:
            sign=1 if "-L-" in key else -1
            put("relocated-"+key,cad.translated(shape,[0,-75*sign,0]),"experiments/swerve-drive/packaging/fit.py")
            continue
        if key.startswith("observer-moving-pad-"):
            shape=cad.translated(shape,[0,0,-.8])
        if local_supplier:restricted[key]=shape
        else:put(key,shape,base,"registered-envelope" if key.startswith("wire/") or "reserv" in key else "project-cad")
    for side in ("front","rear"):
        if "bumper-cover-"+side+"-red" not in records:raise ValueError(side)
    layers=[
        ("deck-joints/deck-through-ties.step",lambda k:k.startswith("deck-tie-")),
        ("mast-attachment/mast-joints.step",lambda k:k in leaves and not k.endswith(("-rail","-mast"))),
        ("carriage-support/carriage-support.step",lambda k:k in leaves),
        ("guide-mounts/guide-mounts.step",lambda k:True),
        ("guide-setting/setting.step",lambda k:"setting-pin" not in k),
        ("shelf-structure/stow-receiver.step",lambda k:k!="slide-base-bridge"),
        ("extension-support/rail-fixture.step",lambda k:True),
        ("fixed-camera-route/harness-guides.step",lambda k: not(k.startswith(("pi-post","host-post","base-guide","usb-A-","host-board"))))]
    for file,select in layers:step(SIX/file,select)
    step(SW/"packaging/swerve-frame.step",lambda k:k!="swerve-relieved-ribbed-deck",True)
    step(SW/"packaging/rear-electronics.step",r2=True)
    step(SW/"power/controller-trays.step")
    rear=read(SW/"packaging/rear-electronics.json")
    sys.path.insert(0,str(SW/"packaging"))
    from rear_fit import original_board, up
    for key,shape in original_board().items():
        raised=up(shape)
        if max(abs(a-b) for a,b in zip(cad.bounds(raised),rear["boundsMm"][key]))>.001:raise ValueError(key)
        put(key,raised,"experiments/swerve-drive/packaging/rear_fit.py","registered-envelope")
    for corner,info in read(SW/"power/controller-fit.json")["corners"].items():
        for name in ("drive","steer"):
            put(corner+"-"+name+"-controller",block(info["fit"][name]["boundsMm"]),
                "experiments/swerve-drive/power/controller-fit.json","registered-envelope")
    for num,(x,y) in enumerate([(118,-66),(118,42)]):
        put(f"receiver-absorber-{num}",tube([x,y,423.5],[0,0,1],4,48.3),"experiments/six-wheel/shelf-structure/damper_fit.py","registered-envelope")
        put(f"receiver-absorber-piston-{num}",tube([x,y,471.8],[0,0,1],1.25,11),"experiments/six-wheel/shelf-structure/damper_fit.py","registered-envelope")
        put(f"receiver-absorber-cap-{num}",tube([x,y,482.8],[0,0,1],3.2,2.1),"experiments/six-wheel/shelf-structure/damper_fit.py","registered-envelope")
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
    import ast
    ink_source=SIX/"selected-optics/shelf_marks.py"
    ink=next(ast.literal_eval(node.value) for node in ast.parse(ink_source.read_text()).body
             if isinstance(node,ast.Assign) and getattr(node.targets[0],"id",None)=="INK")
    for i,side in enumerate(("left","right")):
        strokes=[block([x0,y0,500,x1,y1,500.02]) for x0,y0,x1,y1 in ink[6*i:6*i+6]]
        mark=strokes[0]
        for piece in strokes[1:]:mark=BRepAlgoAPI_Fuse(mark,piece).Shape()
        put("surface-registration-mark-"+side,mark,ink_source.relative_to(ROOT).as_posix(),"appearance-overlay")
    for name,ids in {"fixed-camera": [k for k in restricted if k.startswith("fixed-head/")],
                     "moving-camera":[k for k in restricted if k.startswith("roof-head/")],"signal-lamp":["am-3583"]}.items():
        put(name+"-public-envelope",block(envelope([restricted[k] for k in ids])),base,"public-supplier-envelope")
    sites=read(SW/"packaging/module.json")["modules"]
    for key,site in sites.items():
        x,y=site["xM"]*1000,site["yM"]*1000
        f="experiments/swerve-drive/packaging/module.json"
        put(key+"-display-tread",tube([x,y-15,38.1],[0,1,0],38.1,30),f,"public-module-representation")
        for side in (-1,1):
            yy=y+side*21
            put(key+f"-display-fork-{side}",block([x-26,yy-2.5,32,x+26,yy+2.5,78]),f,"public-module-representation")
        put(key+"-display-top-plate",block([x-45,y-37,78,x+45,y+37,84]),f,"public-module-representation")
        put(key+"-display-drive-motor",tube([x,y,84],[0,0,1],30,59),f,"public-module-representation")
        put(key+"-display-steer-motor",tube([x+(-1 if x>0 else 1)*44,y,84],[0,0,1],17.5,46),f,"public-module-representation")
    bumpers={r["solid"]:r["material"] for r in csv.DictReader((SIX/"design/bumper-kit/mass.csv").open())}
    resolved = resolve_project_palette(ROOT)
    save(local / "effective-palette.json", resolved)
    colors = resolved["colors"]
    public=HERE/"full-robot.step"
    solids=write_step(public,records,colors,bumpers)
    save(HERE/"appearances.json",{"schema":"agent-frc/assembly-appearances@1","sourceSha256":sha(public),"configuration":CONFIG,"units":"mm","solids":solids})
    added={k:v for k,v in records.items() if not v[3] and not v[2].startswith("public-")}
    write_step(local/"superstructure.step",added,colors,bumpers)
    local_vendor={k.replace("/","-"):(v,base,"manufacturer-native",False) for k,v in restricted.items()}
    write_step(local/"vendor-optics.step",local_vendor,colors,bumpers,paint=False)
    save(local/"assembly.json",{"configuration":CONFIG,"nativeProjectAdded":sorted(added),"vendorOptics":sorted(local_vendor),"sources":proofs,"excludedLegacy":exclusions})
    print(len(records),len(solids),len(added),len(local_vendor),flush=True)

def write_step(path,objects,colors,bumpers,paint=True):
    doc,table=cad.xcaf_doc()
    ctab=XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    rows=[]
    for key,(shape,source,repre,_) in sorted(objects.items()):
        pieces=list(subsolids(shape))
        if not pieces:raise ValueError(f"No material for {key}")
        for i,solid in enumerate(pieces,1):
            item=key if len(pieces)==1 else f"{key}-solid-{i}"
            label=table.AddShape(solid,False)
            TDataStd_Name.Set_s(label,TCollection_ExtendedString(item))
            group=role(key,source,bumpers)
            if paint:
                rgb=bytes.fromhex(colors[group][1:])
                ctab.SetColor(label,Quantity_Color(*(x/255 for x in rgb),Quantity_TOC_sRGB),XCAFDoc_ColorGen)
            row={"id":item,"role":group,"source":source,"representation":repre,"boundsMm":[round(x,5) for x in cad.bounds(solid)]}
            rows.append(row)
    writer=STEPCAFControl_Writer()
    if not writer.Transfer(doc,cad.STEPControl_AsIs) or writer.Write(str(path))!=cad.IFSelect_RetDone:raise ValueError(path)
    # Keep build dates and machine-generated file paths out of the project exchange.
    data=path.read_text("ascii")
    data=re.sub(r"FILE_NAME\('[^']*','[^']*'","FILE_NAME('Robot','2025-01-01T00:00:00'",data,count=1)
    path.write_text(data,encoding="ascii",newline="\n")
    if paint:
        reader=STEPControl_Reader()
        if reader.ReadFile(str(path))!=cad.IFSelect_RetDone or not reader.TransferRoots():raise ValueError("Could not reopen")
        actual=list(subsolids(reader.OneShape()))
        if len(rows)!=len(actual):raise ValueError((len(rows),len(actual)))
        for row,s in zip(rows,actual):
            if max(abs(a-b) for a,b in zip(cad.bounds(s),row["boundsMm"]))>0.001:raise ValueError("Unexpected transfer order at "+row["id"])
    return rows

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts",type=Path)
    a=p.parse_args()
    build(a.artifacts,a.artifacts/"full-swerve-fusion")

