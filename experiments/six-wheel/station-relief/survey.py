"""Apply the existing stationary-camera error model to the proposed mechanical entry windows."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"station-localization"))
from attitude import outer, projected
from pose import EYE, SECOND_EYE
from study import edges, source_sha

def budget(eyes,side,angle):
    measurement=[projected(np.zeros(6),e) for e in eyes]
    lo,hi=0.,.8
    for _ in range(17):
        e=(lo+hi)/2
        result=edges(outer(measurement,eyes,2*e))
        if result and result["normalMm"]<=5 and result["sideMm"]<=side and result["headingDeg"]<=angle:
            lo=e
        else:hi=e
    return lo

if __name__=="__main__":
    a=argparse.ArgumentParser(description=__doc__);a.add_argument("--output",type=Path,required=True);p=a.parse_args()
    result={"sourceSha256":source_sha(ROOT/"station-localization/pose-study.json"),"atNominalAlignment":{}}
    for case in ((.5,.25),(.8,.25),(1.,.15)):
        result["atNominalAlignment"][str(case)]={label:budget(eyes,*case) for label,eyes in (("right",(EYE,)),("pair",(EYE,SECOND_EYE)))}
    p.output.write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8",newline="\n")
