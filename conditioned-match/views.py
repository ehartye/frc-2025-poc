"""Search attainable field-camera viewpoints for the issued reef tag and its paired faces."""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import numpy as np

def load_navigation():
    here = Path(__file__).resolve().parent
    spec = importlib.util.spec_from_file_location("conditioned_navigation",here/"navigation.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

if __name__ == "__main__":
    n = load_navigation()
    from vision import station_frame
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets",type=Path)
    args = p.parse_args()
    ctx = n.checked_scene(args.assets)
    out = []
    for sector in (-180, 220):
        for distance in (-1250,-950,-700,-450,-200,0):
            for angle in range(-30,31,5):
                pose = np.array([float(distance),float(sector),math.radians(angle)])
                tag,heads,_,_ = station_frame(pose,ctx)
                out.append({"xMm":distance,"yMm":sector,"yawDeg":angle,"tag":tag is not None,
                            "leftHeadCount":len(heads[0]),"rightHeadCount":len(heads[1])})
    (n.HERE/"views.json").write_text(json.dumps(out,indent=2)+"\n",encoding="utf8",newline="\n")
    for sector in (-180,220):
        print(sector,[(r["xMm"],r["yawDeg"]) for r in out if r["yMm"]==sector and r["tag"]],flush=True)
