"""Resolve the low chassis from the issued robot. Cache locally supplied supplier objects separately."""
import re,sys
from pathlib import Path
from OCP.BRepTools import BRepTools
from frame import ROOT,HERE,j
from module_placement import read,write
from inventory import RETIRE

def passive(key):
    if key.startswith(("front-","rear-","center-")):
        if re.match(r"(front|rear)-[LR]-(ear|pilot|weld-|screw-(90|130)$|head-washer-(90|130)$|nut-washer-(90|130)$|nut-(90|130)$)",key):
            return True
    if key.startswith(("motor-L","motor-R","inner-L-","inner-R-","gearbox-faceplate-","wheel-hardware-")):return False
    return not key.startswith(RETIRE)
def prepare(art,lamp):
    out=art/"swerve-package/robot-native";out.mkdir(parents=True,exist_ok=True)
    sys.path.insert(0,str(ROOT/"fixed-camera-route"))
    import scene
    starts,_=scene.get(art,lamp)
    bounds=read(art/"fixed-camera-route/scene.json")["bounds"]
    if len(starts)!=991:raise ValueError("Expected issued 991-body optics source")
    chosen={}
    for k,s in starts.items():
        b=bounds[k]
        if not passive(k) or b[2]>180 or b[5]<-1:continue
        if not any(b[i+3]>=-354 and b[i]<=354 for i in (0,1)):continue
        # Keep each solid for intersections of azimuth/shifted ear/rail interfaces.
        chosen[k]=s
    tied=read(ROOT/"deck-joints/fit.json")
    chosen.update(j.cad.read_named(ROOT/"deck-joints/deck-through-ties.step",[{"id":k} for k in tied["volumeMm3"]]))
    for k,s in chosen.items():
        BRepTools.Write_s(s,str(out/(k.replace("/","--")+".brep")))
    write(out/"index.json",{k:{"file":k.replace("/","--")+".brep","bounds":j.cad.bounds(s)} for k,s in chosen.items()})
if __name__=="__main__":prepare(Path(sys.argv[1]),Path(sys.argv[2]))
