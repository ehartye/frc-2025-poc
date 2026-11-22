"""Measure the actual joint tip through a section of the alternate commercial motor case."""
from pathlib import Path
import hashlib
import json
import sys

import numpy as np
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.gp import gp_Dir, gp_Lin, gp_Pnt, gp_Trsf
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.TopLoc import TopLoc_Location

HERE = Path(__file__).resolve().parent

def read(name):
    return json.loads((HERE / name).read_bytes())

def transform(body, matrix):
    motion = gp_Trsf()
    motion.SetValues(*matrix[:3].ravel())
    return body.Moved(TopLoc_Location(motion))

def run(scripts, source, drawing):
    sys.path.insert(0, str(scripts))
    from fusion_authority import register_path, verify
    from mechanical_motion import bounds, common
    from showcase_contacts import load_solids

    register = register_path(HERE.parents[3])
    authority = verify(HERE.parents[3], register, register.parent / "issue-005/lineage.json")
    key = read("inset-oem-steering-key.json")
    body_path = HERE.parent / "private-inputs/matched-sds/corner.step"
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    if authority != key["authority"] or sha(source) != key["motorSHA256"] or sha(body_path) != key["donorStep"]:
        raise ValueError("Motor or module does not match the registered comparison")
    motor = next(iter(load_solids(source)[0].values()))
    screws = load_solids(body_path)[0]
    corners = read("inset-matched-corners.json")["newWorldMm"]
    mounts = read("inset-corner-local-fits.json")["motorCases"]
    pose = np.array(key["referenceShaftPoseMm"]) @ np.diag([-1.,-1.,1.,1.])
    contact = IntCurvesFace_ShapeIntersector()
    contact.Load(motor,1e-7)
    cases = []
    shapes = {}
    for mount in mounts:
        m = mount["case"]
        if ".36.1]" not in m["host"]:
            continue
        station = mount["corner"]
        name = m["screw"]
        xy = (np.linalg.inv(np.array(corners[station]) @ pose) @ [*m["underHeadCenterMm"],1])[:2]
        if min(np.linalg.norm(xy-np.array(p)) for p in [(25.4,0),(-25.4,0),(0,-25.4)]) > .002:
            raise ValueError("Changed designated mounting pattern")
        if name not in shapes:
            screw = transform(screws[name], np.linalg.inv(pose))
            span = bounds(screw)
            sections = {}
            for r in [0,1.5,1.99,2.02,2.4]:
                depth = []
                for a in np.arange(0,360,45):
                    x,y = xy + r*np.array([np.cos(np.deg2rad(a)),np.sin(np.deg2rad(a))])
                    contact.Perform(gp_Lin(gp_Pnt(x,y,.3),gp_Dir(0,0,-1)),0,75)
                    depth.append(sorted(-contact.Pnt(k).Z() for k in range(1,contact.NbPnt()+1)))
                sections[str(r)] = depth
            # Exact finite box across the entire end of the screw; test every motor body
            # feature below the 5 mm front plate, including internal components.
            box = BRepPrimAPI_MakeBox(gp_Pnt(xy[0]-5,xy[1]-5,-15),gp_Pnt(xy[0]+5,xy[1]+5,-5.000001)).Shape()
            rows = []
            for shift in [0.,.28,1.28]:
                move = np.eye(4); move[2,3] = -shift
                tip = transform(screw,move)
                from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
                part = BRepAlgoAPI_Common(tip,box)
                if not part.IsDone(): raise ValueError("Clip failed")
                rows.append(dict(overInsertionMm=shift,tipDepthMm=-span[2]+shift,
                                 tipBehindCaseMm3=common(tip,box),commonWithMotorBehindCaseMm3=
                                 [common(part.Shape(),motor),common(motor,part.Shape())]))
            # A rectangular enclosure cannot lose the tiny rounded screw tip during clipping.
            enclosures = []
            for zmin,zmax in [(-6.2,-5.00001),(-58.,-55.5)]:
                gauge = BRepPrimAPI_MakeBox(gp_Pnt(xy[0]-2.6,xy[1]-2.6,zmin),gp_Pnt(xy[0]+2.6,xy[1]+2.6,zmax)).Shape()
                enclosures.append(dict(zMm=[zmin,zmax],sideMm=5.2,commonMm3=[common(gauge,motor),common(motor,gauge)]))
            shapes[name] = dict(screw=name,sourceXYmm=xy.tolist(),nominalTipDepthMm=-span[2],enclosures=enclosures,
                                radialEdgesMm=sections,tipCases=rows)
        cases.append(dict(corner=station,screw=name,sourceXYmm=xy.tolist()))
    if len(cases)!=12 or len(shapes)!=3: raise ValueError("Incomplete mount set")
    return dict(authority=authority,motorSha256=sha(source),drawingSha256=sha(drawing),
                moduleSha256=sha(body_path),coldModuleAssembly=key["cold508"],axes=cases,sections=shapes,
                limits="Seven tapped 10-32 UNF ×5.5 mm holes are called out by the 2022-11-09 drawing. CAD front shell is 5.000 mm; measured internal features and the smooth pilot do not prove delivered threads, bottom lead, preload, wire clearance or internal motor construction.")
if __name__=="__main__":
    out = run(*map(Path,sys.argv[1:]))
    (HERE/"inset-v11-screw-tip.json").write_text(json.dumps(out,indent=1)+"\n")
