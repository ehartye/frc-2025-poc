"""Verify retained setting pin, socket and probe access against the replacement arm."""
import argparse
from pathlib import Path
from OCP.BRepPrimAPI import BRepPrimAPI_MakeSphere
from OCP.gp import gp_Pnt
from arm_route import ARM, ROOT, carriage, inputs, j
from parts import MAST_BOLTS, xcyl


def run(folder):
    exchanged=carriage.selected(folder,"route.step","revision.json","stepSha256","volumesMm3")
    arm=exchanged[ARM]
    kit=inputs()
    setting=ROOT/"guide-setting"
    fit=j.read(setting/"fit.json")
    probe=j.read(setting/"probe.json")
    if probe["settingStepSha256"]!=fit["stepSha256"] or probe["settingStepSha256"]!=j.digest(setting/"setting.step"):
        raise ValueError("Reference guide changed")
    if fit["guideSha256"]!=j.digest(ROOT/"guide-mounts/guide-mounts.step"):
        raise ValueError("Reference mount changed")
    # The sole installed geometry change is the lower-left arm.
    rows={}
    for item,replacement in exchanged.items():
        if item==ARM:continue
        before=kit[item]
        difference=j.cad.volume(before)[0]+j.cad.volume(replacement)[0]-2*j.cad.intersection(before,replacement)
        if abs(difference)>.003:raise ValueError(("Additional installed part changed",item,difference))
    def witness(key,shape):
        volume=j.cad.intersection(shape,arm)
        rows[key]=round(volume,7)
        if volume>.001:raise ValueError((key,volume))
    for key,row in [*probe["mastDatums"].items(),
            *((station+"/"+side,r) for station,st in probe["stations"].items() for side,r in st["sides"].items())]:
        tip=row["tipCenterMm"]
        away=[-n for n in row["normalTowardMaterial"]]
        end=[v+30*n for v,n in zip(tip,away)]
        sweep=j.cad.brep(["cyl",[*tip,*away,1.5,30.]])
        for at in (tip,end):
            sweep=j.fuse(sweep,BRepPrimAPI_MakeSphere(gp_Pnt(*at),1.5).Shape())
        stem=j.cad.brep(["cyl",[*tip,*away,1.,65.]])
        witness("probe/"+key,j.fuse(stem,sweep))
    for side,sign in (("L",1),("R",-1)):
        for height in (687,787):
            y=sign*219;z=height+5
            witness(f"pin/{side}/{height}",j.fuse(xcyl(-25.5,y,z,2,63.5),xcyl(-14,y,z,3,63.5)))
            for bolt,(axis,delta) in enumerate(MAST_BOLTS):
                witness(f"driver/{side}/{height}/{bolt}",xcyl(-14,sign*axis,height+delta,6.25,80.))
    j.write(folder/"inspection.json",{"stepSha256":j.digest(folder/"route.step"),
        "referenceProbeSha256":j.digest(setting/"probe.json"),"referenceToolSha256":j.digest(setting/"fit.json"),
        "overlapWithRevisedArmMm3":rows})


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder",type=Path)
    run(p.parse_args().folder)
