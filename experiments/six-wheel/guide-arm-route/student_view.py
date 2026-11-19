"""Reopen three left-guide assemblies in the project's offline CAD surface viewer."""
import argparse
import base64
import gzip
import importlib.util
import json
from pathlib import Path
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from arm_route import ARM,HERE,ROOT,carriage,inputs,j


def build(raw):
    template=ROOT/"design/competition-package"
    spec=importlib.util.spec_from_file_location("public_surface",template/"build.py")
    renderer=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(renderer)
    kit=inputs()
    mast=BRepAlgoAPI_Common(kit["guide-L-machined-mast"],j.box(-75,-18,204,239,610,855)).Shape()
    data={}
    for mode,layout,height in (("straight","rise-+0",687),("raised","rise-+73",760),("returned","front-return",687)):
        folder=raw/layout
        changed=carriage.selected(folder,"route.step","revision.json","stepSha256","volumesMm3")
        model={**kit,**changed}
        rows=[["mast section","Mast","#8996a1",*renderer.triangulate(mast)]]
        for k,v in model.items():
            if not k.startswith("guide-L-") or k.endswith("machined-mast"):continue
            group,color=("Lower arm","#c06a31") if k==ARM else ("Guide faces","#43867c") if k.endswith(("liner","backing")) else ("Upper arm and fittings","#718394")
            rows.append([k.removeprefix("guide-L-"),group,color,*renderer.triangulate(v)])
        stops=[abs(r["finalBodyErrorMmDeg"][0]) for r in j.read(folder/"field.json")["runs"] if r["mode"]=="nominal"]
        data[mode]={"parts":rows,"height":height,"stroke":round(max(stops),2)}
    text=(template/"viewer.template.html").read_text(encoding="utf8")
    notes={"straight":"The lower-left arm hides a surveyed field fastener. The two modeled approaches stop 75 mm and 27 mm short.",
           "raised":"Both docking approaches reach the target. Raising the lower connection leaves a long unsupported liner. The static study predicts more than 1.2 mm of motion in both original loading cases.",
           "returned":"The closed section returns to the original lower attachment. Both docking approaches reach the target. Joint stiffness, camera calibration and pipe retention still require physical measurements."}
    changes={
        "Station receiver":"The guide arm in the camera’s way","../../mission/moving-field.html":"../mounted-guide-vision/viewer.html",
        "Follow the shelf from pickup to placement. Rotate the robot or isolate a subsystem to inspect how its parts fit.":
            "Rotate the left guide and select its lower arm. Compare how the mount changes what the docking camera sees and where the pipe pushes on the guide.",
        "Carriage pose":"Mount design","Assembly review":"Left guide","3D robot assembly":"3D guide and arm",
        "Whole robot":"Left assembly","Remove bumpers and deck":"Hide mast",
        "(!p.name.startsWith('bumper')&&p.name!=='deck-main')":"p.group!=='Mast'",
        "Carriage height":"Lower guide attachment","Slide extension":"Worst longitudinal docking error",
        "sceneCenter=[0,0,480];sceneSpan=1260;":"sceneCenter=[90,179,737];sceneSpan=375;",
        "const names={stowed:'Stowed',receive:'Receive',transport:'Carry',place:'Place'};":
            "const names={straight:'Straight arm',raised:'Raised attachment',returned:'Closed return'};const notes="+json.dumps(notes)+";",
        "pose='place'":"pose='returned'","model.place.surfaces":"model.returned.surfaces","setPose('place')":"setPose('returned')",
        "info[group]||'The lift changes height while the slide moves the shelf toward the station or trough. Use the pose controls to compare their motions.'":"notes[pose]",
        "Nominal CAD surfaces share the chassis frame. Purchased-part envelopes, cable routes and sensing clearances are design assumptions.":
            "Surfaces come from the reopened guide CAD. Docking errors refer to two modeled wheel calibrations. The mast is sectioned for visibility.",
        "@@MESH@@":base64.b64encode(gzip.compress(json.dumps(data,separators=(",",":")).encode(),mtime=0)).decode()}
    for old,new in changes.items():
        if old not in text:raise ValueError("The viewer template changed at "+old[:30])
        text=text.replace(old,new)
    (HERE/"compare.html").write_bytes(text.encode())


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("raw_designs",type=Path)
    build(p.parse_args().raw_designs)
