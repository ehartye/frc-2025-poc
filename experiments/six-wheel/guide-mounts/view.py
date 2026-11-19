"""An offline view of the guide change and the power tail at its actual crossing stroke."""
import base64
import gzip
import importlib.util
import json
import math
import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common

from parts import HERE, ROOT, j, carriage


def cable_mesh(stroke):
    pts=np.asarray(j.read(ROOT/"camera-disconnects/route-controls.json")["power"])
    n=len(pts)-1
    t=np.linspace(0,1,161)
    basis=np.array([math.comb(n,i)*t**i*(1-t)**(n-i) for i in range(n+1)]).T
    axis=basis@pts+[stroke,0,0]
    tangent=np.gradient(axis,axis=0)
    tangent/=np.linalg.norm(tangent,axis=1)[:,None]
    # Parallel transport the circle perpendicular to the registered Bezier centerline.
    u=np.array([0.,1.,0.])
    vertices=[]
    indices=[]
    angle=2*math.pi*np.arange(16)/16
    for i,(center,direction) in enumerate(zip(axis,tangent)):
        u=u-(u@direction)*direction
        u/=np.linalg.norm(u)
        v=np.cross(direction,u)
        ring=center+4.30*(np.cos(angle)[:,None]*u+np.sin(angle)[:,None]*v)
        vertices.extend(np.rint(ring*10).astype(int).ravel().tolist())
        if i:
            for a in range(16):
                k=(i-1)*16+a
                h=(i-1)*16+(a+1)%16
                indices.extend((k,h,h+16,k,h+16,k+16))
    return vertices,indices


def build():
    template=ROOT/"design/competition-package"
    spec=importlib.util.spec_from_file_location("surface_builder",template/"build.py")
    renderer=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(renderer)
    fit=j.read(HERE/"fit.json")
    if j.digest(HERE/"guide-mounts.step")!=fit["stepSha256"]:
        raise ValueError("Unexpected guide exchange")
    kit=j.cad.read_named(HERE/"guide-mounts.step",[{"id":k} for k in fit["volumesMm3"]])
    legacy=carriage.selected(ROOT/"station-guide","narrow-face.step","fit-narrow.json","stepSha256","volumesMm3")
    mast=BRepAlgoAPI_Common(kit["guide-R-machined-mast"],j.box(-75,-18,-239,-204,610,855)).Shape()
    stroke=fit["oldCableWitness"]["strokeMm"]
    data={}
    for state in ("original","revised","service"):
        slide=0 if state=="service" else stroke
        rows=[["power tail","Power cable","#b05622",*cable_mesh(slide)]]
        if state=="original":
            rows.append(["original guide face","Guide","#398479",*renderer.triangulate(legacy["guide-negative-y"])])
        else:
            rows.append(["mast section","Mast","#7c8998",*renderer.triangulate(mast)])
            for key,shape in kit.items():
                if not key.startswith("guide-R-") or key.endswith("machined-mast"):
                    continue
                if state=="service":
                    if "-root-" in key:
                        if not key.endswith("sleeve"):
                            continue
                    else:
                        shape=j.cad.translated(shape,(92,65,0))
                group,color=("Guide","#398479") if key.endswith(("liner","backing")) else ("Arms and fittings","#687687")
                rows.append([key.removeprefix("guide-R-"),group,color,*renderer.triangulate(shape)])
        data[state]={"parts":rows,"height":850 if state=="original" else 813,"stroke":round(slide,1)}
    page=(template/"viewer.template.html").read_text(encoding="utf8")
    notes={
        "original":"At 327.8 mm of slide extension the cable center passes through the original liner at a height of 830.1 mm.",
        "revised":"The new guide ends at 813 mm. Rotate the assembly to see the folded backing and the arms carrying it back to the mast.",
        "service":"With the slide stowed, the guide moves 65 mm inward and then 670 mm forward. This view shows the first 92 mm of the forward movement."
    }
    substitutions=[
        ("Station receiver","Where the guide meets the power cable"),
        ("../../mission/moving-field.html","../conditioned-match/replay.html"),
        ("Follow the shelf from pickup to placement. Rotate the robot or isolate a subsystem to inspect how its parts fit.",
         "Compare the original right guide and the shorter replacement at the same slide position. Select a part or hide the cable to inspect the attachment."),
        ("Carriage pose","Guide comparison"),("Assembly review","Right guide"),("3D robot assembly","3D guide and power cable"),
        ("Whole robot","Full assembly"),("Remove bumpers and deck","Hide power cable"),
        ("(!p.name.startsWith('bumper')&&p.name!=='deck-main')","p.group!=='Power cable'"),
        ("Carriage height","Guide top"),("Slide extension","Slide position"),
        ("sceneCenter=[0,0,480];sceneSpan=1260;",
         "sceneCenter=pose==='original'?[248,-155,815]:pose==='service'?[100,-150,740]:[96,-179,746];sceneSpan=pose==='original'?160:405;"),
        ("const names={stowed:'Stowed',receive:'Receive',transport:'Carry',place:'Place'};",
         "const names={original:'Original crossing',revised:'Shorter guide',service:'Removal path'};const poseNotes="+json.dumps(notes)+";"),
        ("pose='place'","pose='revised'"),("model.place.surfaces","model.revised.surfaces"),("setPose('place')","setPose('revised')"),
        ("info[group]||'The lift changes height while the slide moves the shelf toward the station or trough. Use the pose controls to compare their motions.'","poseNotes[pose]"),
        ("Nominal CAD surfaces share the chassis frame. Purchased-part envelopes, cable routes and sensing clearances are design assumptions.",
         "The cable follows the project routing model. The mast is sectioned, and the liner stiffness and cable position need physical checks."),
        ("@@MESH@@",base64.b64encode(gzip.compress(json.dumps(data,separators=(",",":")).encode(),mtime=0)).decode())]
    for old,new in substitutions:
        if old not in page:
            raise ValueError("Viewer template changed at "+old[:40])
        page=page.replace(old,new)
    (HERE/"guide.html").write_bytes(page.encode())


if __name__=="__main__":
    build()
