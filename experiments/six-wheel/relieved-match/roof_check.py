"""Challenge the measured carry pose with complete native robot occluders and cartridge travel."""
import argparse
import json
from pathlib import Path
import numpy as np
from cameras import HERE,ROOT,low_relief,mount,optics,camera,receiver_drives,mesh
from sight import background,bridge_fit,silhouette


def run(cache,lamp):
    nominal=background(cache/"relieved-roof-nominal.npz")
    static=background(cache/"relieved-roof-backed.npz")
    ref=[(g,b) for (g,_),(_,b) in zip(nominal,static)]
    _,groups,colors,_=mount.pose_layers(560,0,"red")
    results=[]
    for stroke in (0,185,370):
        parts=low_relief(lamp,stroke)
        for offset in (-.40,0.,1.40):
            full={}
            for name,shape in parts.items():
                if name=="camera-envelope":continue
                if receiver_drives.translates_with_grip(name):
                    shift=(1 if name.startswith("grip-L-") else -1)*offset
                    shape=mount.cad.translated(shape,(0,shift,0))
                full[name]=shape
            optics.ORIGIN=np.array([48.+stroke,0.,842.]);optics.configure()
            full_bg=camera.images_for_pose(mesh(full,groups,colors))
            optics.ORIGIN=np.array([48.,0.,842.]);optics.configure()
            for piece in ("coral-A","coral-B"):
                data=json.loads((HERE/(piece+"-pads-edge-250.json")).read_bytes())
                for p in data["minMaxRetainedCenterMm"]:
                    frame=silhouette(full_bg,np.array(p),np.array([0.,1.,0.]))
                    fit=bridge_fit.fit(frame,ref)
                    truth=[*p[:2],p[2]-557.15]
                    enclosed=all(fit[n][0]<=x<=fit[n][1] for n,x in zip(("x_mm","y_mm","bottom_mm"),truth)) if fit["status"]=="inside" else False
                    results.append({"strokeMm":stroke,"cartridgeTranslationMm":offset,"pipe":piece,"center":p,"status":fit["status"],"enclosed":enclosed})
    (HERE/"roof-challenge.json").write_text(json.dumps(results,indent=2)+"\n",encoding="utf8",newline="\n")
    print(len(results),sum(v["enclosed"] for v in results),flush=True)

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("cache",type=Path);p.add_argument("lamp",type=Path)
    a=p.parse_args();run(a.cache,a.lamp)
