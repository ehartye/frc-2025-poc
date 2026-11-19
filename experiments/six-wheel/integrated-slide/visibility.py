"""Compare isolated old and new mount surfaces against the occupied two-eye depth buffers."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

from render import ROOT, HERE, scenes, digest


def run(artifacts,lamp,study,prior):
    model=scenes.load_assembly(ROOT/"selected-match/clearance.py")
    original=scenes.selected_poses(model,artifacts,lamp)
    sys.path.insert(0,str(ROOT/"sp1-mount"))
    from sp1_hardware import REMOVED,manufactured,supplied
    sys.path.insert(0,str(ROOT/"relieved-match"))
    from cameras import mount,optics,camera
    scenes.mount,scenes.camera,scenes.colors=mount,camera,{}
    registers=json.loads((ROOT/"selected-optics/mount.json").read_bytes())["registrations"]
    report={}
    for stroke,old in zip((0,370),original):
        old={k:old[k] for k in REMOVED}
        selected={**manufactured(stroke),**supplied(stroke)}
        for revision,parts,folder in (("original",old,prior),("selected",selected,study)):
            mesh=scenes.paint_world(parts)
            for mode in ("fixed","roof"):
                if mode=="roof":
                    optics.ORIGIN=np.array(registers["roof"]["assumedRectifiedOriginMm"])+[stroke,0,0]
                    optics.configure()
                    frames=["barrel-moving-"+str(stroke)]
                else:
                    scenes.fixed_pose(np.array(registers["fixed-datum-up"]["assumedRectifiedOriginMm"]))
                    frames=["first-fixed","return-fixed"] if stroke==0 else ["coral-A-fixed","coral-B-fixed"]
                isolated=camera.images_for_pose({"parts":mesh})
                for frame in frames:
                    with np.load(folder/"marked"/(frame+".npz")) as data:
                        details=[]
                        for (_,probe),side in zip(isolated,("left","right")):
                            full=data[side+"_z"]
                            on_part=probe<3490
                            nearer=probe[on_part]-full[on_part]
                            details.append({"projectedPixels":int(on_part.sum()),
                                "withinDepthTolerance":int((nearer<=.1).sum()),
                                "nearestPartBehindBackingMm":round(float(nearer.min()),4) if len(nearer) else None})
                        report[revision+"/"+frame]=details
    result={"rasterToleranceMm":.1,"originalMarkedSourceSha256":digest(prior/"marked/marks.json"),
            "selectedMarkedSourceSha256":digest(study/"marked/marks.json"),"eyesLeftRight":report}
    (HERE/"visibility.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for key in ("artifacts","lamp","study","prior"):
        p.add_argument(key,type=Path)
    a=p.parse_args()
    run(a.artifacts,a.lamp,a.study,a.prior)
