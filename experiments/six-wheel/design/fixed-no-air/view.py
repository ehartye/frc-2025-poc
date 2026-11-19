"""Render the project-authored service interface in its low, left-side chassis coordinates."""
import base64
import gzip
import json

from service import HERE, ROOT, BRepAlgoAPI_Common, cad, configuration, pin_parts
from assembly import original


def build():
    parts = configuration("low-place")
    crop = cad.brep(["box",[-138,265,378,91,34,40]])
    fixed = {}
    for key, ident, group, color in (
        ("rail section","service-pin-rail","Rail and bushing","#929da5"),
        ("bushing","service-pin-bushing-560","Rail and bushing","#465868"),
        ("carriage plate section","carriage-plate","Carriage plate","#26766f"),
    ):
        op = BRepAlgoAPI_Common(parts["backup-L-"+ident],crop)
        if not op.IsDone() or not cad.solid_count(op.Shape()):
            raise ValueError("Interface section failed")
        fixed[key] = (op.Shape(), group, color)
    data = {}
    for state in ("withdrawn","inserted","seated","long"):
        scene = fixed.copy()
        if state == "seated":
            shape, group, color = fixed["carriage plate section"]
            scene["carriage plate section"] = (cad.translated(shape,[0,0,-1]),group,color)
        for key, shape in pin_parts(282,395,-66 if state=="long" else -88).items():
            if state=="withdrawn":
                if key!="pin":
                    continue
                shape=cad.translated(shape,[-70,0,0])
            scene[key]=(shape,"Pit hardware","#ca7637" if key=="pin" else "#614128")
        data[state]={"height":560,"stroke":370,"parts":[[key,group,color,*original.triangulate(shape)]
                    for key,(shape,group,color) in scene.items()]}
    page = (ROOT/"design/competition-package/viewer.template.html").read_text(encoding="utf8")
    replacements = {
        "Station receiver":"Pit support at the low carriage",
        "../../mission/moving-field.html":"../../fixed-height-match/replay.html",
        "Follow the shelf from pickup to placement. Rotate the robot or isolate a subsystem to inspect how its parts fit.":
            "Follow the left support pin through its bushing and under the carriage plate. Compare the short pin with the longer control.",
        "Assembly review":"Service support detail",
        "Carriage pose":"Service support state",
        "3D robot assembly":"3D service support detail",
        "Whole robot":"Local assembly",
        "Remove bumpers and deck":"Hide carriage plate",
        "(!p.name.startsWith('bumper')&&p.name!=='deck-main')":"p.group!=='Carriage plate'",
        "sceneCenter=[0,0,480];sceneSpan=1260;":
            "sceneCenter=pose==='withdrawn'?[-131,282,396]:[-105,282,396];sceneSpan=pose==='withdrawn'?160:97;",
        "const names={stowed:'Stowed',receive:'Receive',transport:'Carry',place:'Place'};":
            "const names={withdrawn:'Pin withdrawn',inserted:'Pin retained',seated:'Carriage seated',long:'Long pin comparison'};",
        "pose='place'":"pose='inserted'",
        "pitch=.44":"pitch=-.40",
        "model.place.surfaces":"model.inserted.surfaces",
        "setPose('place')":"setPose('inserted')",
        "The lift changes height while the slide moves the shelf toward the station or trough. Use the pose controls to compare their motions.":
            "The plate settles 1 mm onto the pin. Hide the plate to see how the two keeper pieces restrain axial movement.",
        "Nominal CAD surfaces share the chassis frame. Purchased-part envelopes, cable routes and sensing clearances are design assumptions.":
            "The rail and carriage are sectioned to expose their working faces. Both supports must be qualified before service.",
        "@@MESH@@":base64.b64encode(gzip.compress(json.dumps(data,separators=(",",":")).encode(),mtime=0)).decode()
    }
    for old,new in replacements.items():
        if old not in page:
            raise ValueError("Viewer template changed at " + old[:40])
        page=page.replace(old,new)
    (HERE/"service.html").write_bytes(page.encode())


if __name__=="__main__":
    build()
