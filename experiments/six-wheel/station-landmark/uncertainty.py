"""Construct indistinguishable stationary observations using the finite stereo projections."""
import json
import math
from pathlib import Path
import numpy as np
from panel import pixels


def counterexample(side_separation):
    choices=[]
    for heading_separation in np.linspace(-.3,.3,481):
        delta=np.array([0,side_separation/2,math.radians(heading_separation)/2])
        left,right=pixels(-delta),pixels(delta)
        # The midpoint image is inside both independent per-coordinate error boxes.
        choices.append((float(np.max(abs(right-left)))/2,float(heading_separation)))
    error,angle=min(choices)
    return {"sideSeparationMm":side_separation,"yawSeparationDeg":angle,"sharedObservationErrorPx":error}


if __name__=="__main__":
    result=[counterexample(width) for width in (.8,1,1.5,2,3)]
    Path(__file__).with_name("ambiguity.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf8",newline="\n")
