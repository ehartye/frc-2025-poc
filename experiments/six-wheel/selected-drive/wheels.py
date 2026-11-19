"""Measure the named six-wheel/hex-hub field support geometry in the registered native shop STEP."""
import argparse
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"axle-retention"))
from measure import cad,read_shop,SHOP_STEP

def run(artifacts):
    shapes,_=read_shop(artifacts)
    rows={}
    for station in ("front","center","rear"):
        for side in ("L","R"):
            tag=station+"-"+side
            wheel=cad.bounds(shapes["wheel-"+tag])
            hub=cad.bounds(shapes["hub-"+tag])
            rows[tag]={"wheelBoundsMm":list(wheel),"hubBoundsMm":list(hub),
                       "outerRadiusMm":round((wheel[5]-wheel[2])/2,4),
                       "contactCenterMm":[round((wheel[i]+wheel[i+3])/2,4) for i in range(3)]}
    track=abs(rows["center-L"]["contactCenterMm"][1]-rows["center-R"]["contactCenterMm"][1])
    result={"stepSHA256":SHOP_STEP,"wheels":rows,"nominalTrackMm":track,
            "measuredLoadedRadiusesMm":None,"measuredEffectiveTrackMm":None}
    (HERE/"wheels.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("artifacts",type=Path)
    run(p.parse_args().artifacts)
