"""Project the old and new stationary motor hardware into issued endpoint depth buffers."""
import argparse
import sys
from pathlib import Path
import numpy as np

from cap_fit import ROOT, HERE, X, Z, FACE, cylinder, j, kit

sys.path.insert(0,str(ROOT/"integrated-slide"))
from visibility import scenes


def run(study):
    sys.path.insert(0,str(ROOT/"relieved-match"))
    from cameras import mount,optics,camera
    scenes.mount,scenes.camera,scenes.colors=mount,camera,{}
    baseline={"motor-body":cylinder(X,Z,-95,58.25,30)}
    eyes=j.read(ROOT/"selected-optics/mount.json")["registrations"]
    rows={}
    for name,group in (("original",baseline),("new",kit())):
        mesh=scenes.paint_world(group)
        for kind,frames in (("fixed",("first-fixed","return-fixed","coral-A-fixed","coral-B-fixed")),
                            ("roof",("barrel-moving-0","barrel-moving-370"))):
            for frame in frames:
                if kind=="fixed":
                    scenes.fixed_pose(np.array(eyes["fixed-datum-up"]["assumedRectifiedOriginMm"]))
                else:
                    delta=370 if frame.endswith("370") else 0
                    optics.ORIGIN=np.array(eyes["roof"]["assumedRectifiedOriginMm"])+[delta,0,0]
                    optics.configure()
                isolated=camera.images_for_pose({"parts":mesh})
                with np.load(study/"marked"/(frame+".npz")) as full:
                    out=[]
                    for side,(_,probe) in zip(("left","right"),isolated):
                        mask=probe<3490
                        gap=probe[mask]-full[side+"_z"][mask]
                        out.append({"projected":int(mask.sum()),"visibleWithin0_1mm":int((gap<=.1).sum()),
                                    "nearestBehindMm":float(gap.min()) if len(gap) else None})
                    rows[name+"/"+frame]=out
    j.write(HERE/"images.json",{"sourceRegister":j.digest(study/"controller-register.json"),"eyes":rows,
        "scope":"Nominal endpoint depth rasters. Between endpoints, altered roof backgrounds require re-registration."})


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("study",type=Path)
    run(p.parse_args().study)
