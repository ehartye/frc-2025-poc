"""Check a removable nutbar's floor-level service corridor against every original installed item."""
import argparse
import json
from pathlib import Path
import sys

from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.gp import gp_Pnt
from other_terminal_fit import transform, Z


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def box(bounds):
    return BRepPrimAPI_MakeBox(gp_Pnt(*bounds[:3]), gp_Pnt(*bounds[3:])).Shape()


def run(package):
    sys.path.insert(0, str(package / "scripts"))
    from fusion_authority import register_path, verify
    from mechanical_motion import bounds, common
    from showcase_contacts import load_solids, box_gap

    register = register_path(ROOT)
    lineage = register.parent / "issue-004/lineage.json"
    source = verify(ROOT, register, lineage)
    solids, sheets = load_solids(lineage.with_name("assembly.step"))
    if sheets or len(solids) != 504:
        raise ValueError("Wrong lower or incomplete solid roster")
    broad = {k: bounds(v) for k,v in solids.items()}
    # The bar clears the open tube by shifting 14 mm downward. Only while lowered do we move it
    # 0.50 mm away from the near sidewall. Translation starts with its trailing end beyond the rail.
    zlo = Z - 3.175 - 10 - 14
    zhi = Z - 3.175 + 12.5 - 14
    slide = box([-77, 321.95, zlo, 430, 339.75, zhi])
    side_shift = box([-77,321.45,zlo,77,339.75,zhi])
    source_port = "Port terminal cartridge:1+"
    config = json.loads((register.parent/"private/port-504-native.json").read_bytes())
    material_paths = [r for r in config["coldCartridge"] if r["path"].startswith(source_port)]
    port_shapes = set()
    for r in material_paths:
        matches = sorted((max(abs(a-b) for a,b in zip(r["boundsMm"], bb)), k) for k,bb in broad.items())
        if matches[0][0]>.1 or matches[1][0]<.5:
            raise ValueError("Ambiguous removable item")
        port_shapes.add(matches[0][1])
    if len(port_shapes)!=16:
        raise ValueError("Incomplete order")
    # Identify the access-side module by its actual center in the world. Product paths bear their
    # original supplier definition names irrespective of which corner received the component.
    module_parts = {}
    for k in solids:
        if " [1." not in k:
            continue
        suffix=k.rsplit(" [1.",1)[1]
        group=suffix.split(".")[0]
        module_parts.setdefault(group,[]).append(k)
    groups={k:v for k,v in module_parts.items() if len(v)==121}
    if len(groups)!=4:
        raise ValueError("Could not identify four full module rosters")
    results={}
    for side, angle, dx in (("port",0,0),("starboard",180,0),("front",-90,5),("rear",90,-5)):
        pose=transform(angle,dx)
        far=gp_Pnt(300,330,60).Transformed(pose.Transformation())
        def planar_distance(keys):
            x=(min(broad[k][0] for k in keys)+max(broad[k][3] for k in keys))/2
            y=(min(broad[k][1] for k in keys)+max(broad[k][4] for k in keys))/2
            return (x-far.X())**2+(y-far.Y())**2
        near=min(groups,key=lambda key:planar_distance(groups[key]))
        removable=set(groups[near]) | (port_shapes if side=="port" else set())
        stages={}
        for name,shape in (("lowered-slide",slide.Moved(pose)),("lowered-side-shift",side_shift.Moved(pose))):
            bb=bounds(shape)
            detail=[]
            for k,s in solids.items():
                if box_gap(bb,broad[k])>.001:
                    continue
                inter=common(shape,s)
                detail.append(dict(article=k,volumeMm3=inter,removedBeforeSlide=k in removable))
            stages[name]=dict(wholeScope=len(solids),broadClear=len(solids)-len(detail),details=detail,
                              retainedPositive=[row for row in detail if row["volumeMm3"]>.005 and not row["removedBeforeSlide"]])
        results[side]=dict(accessCornerSourceGroup=near,removedModuleBodies=len(groups[near]),stages=stages)
        print(side,[(n,len(s["retainedPositive"])) for n,s in stages.items()],flush=True)
    return dict(authority=source,results=results,scope="Continuous enclosing prisms for lowered longitudinal insertion and sideways centering, with the cartridge and one adjacent module removed. Positive prism overlap can overstate an actual article collision. Free slide through remaining exact parked material does not rate module withdrawal, final vertical raise, tool access, unmodeled tube radii or a future bumper.")


if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("package",type=Path)
    result=run(p.parse_args().package)
    (HERE/"cartridge-service.json").write_text(json.dumps(result,indent=2)+"\n")
