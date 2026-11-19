"""Sweep the attained stowed chassis and the five programmed peers through the assessment."""
import argparse
import json
import math
from pathlib import Path
import sys
import numpy as np
from views import load_navigation
n=load_navigation()
sys.path.insert(0,str(n.ROOT/"station-landmark"))
from field import screen,parked_peers
from transport import trajectory

def check(assets):
    plan=n.read(n.ROOT/"fixed-height-match/placements.json")["placements"]
    f,_,team,_=n.route.route.field.scenario(plan,inboard=True)
    stock=n.read(n.ROOT/"full-match/installed-stock.json")["stock"]
    opening=n.read(n.ROOT/"station-guide/optics-narrow.json")["trials"]
    results={}
    origin=18+plan[0]["drive"]
    for side in ("leftSmall","rightSmall"):
        for mode in ("shaft","gyro","corrected","corrected-mono"):
            times,pose=trajectory(assets,side,mode)
            gaps=screen(f,origin+times,pose,stock,team,origin+times[-1])
            hold=parked_peers(team,pose[-1],origin+times[-1])
            # Bound the stationary interval between the conditioned feed and the actual start of motion.
            wait=np.arange(21.10,origin,.01)
            wait=np.r_[wait,origin]
            wait_gap=screen(f,wait,np.tile(pose[0],(len(wait),1)),stock,team,origin)
            results[side+"-"+mode]={"outbound":gaps,"stationHold":wait_gap,"atRestPeerMm":hold,
                   "reachedStopFieldXYHeadingRad":pose[-1].tolist(),"approachThrough21p1":({k:n.read(n.HERE/"opening.json")[side]["clearance"][k] for k in ("minimumBarrierMm","minimumStockMm","minimumPeerMm")} if mode.startswith("corrected") else opening[side]["fieldMinimumMm"]),
                   "lastWheelAtWallS":origin+times[-1],"fieldAndStockSha256":n.sha(n.ROOT/"full-match/installed-stock.json")}
            print(side,mode,gaps["minimumBarrierMm"],gaps["minimumStockMm"],gaps["minimumPeerMm"],min(hold.values()),flush=True)
    return results

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("assets",type=Path);a=p.parse_args()
    (n.HERE/"occupancy.json").write_text(json.dumps(check(a.assets),indent=2)+"\n",encoding="utf8",newline="\n")
