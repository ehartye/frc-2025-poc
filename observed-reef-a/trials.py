"""Exercise the known signed approach poses with finite stowed objects and exposure delays."""
import argparse
import json
import math
from pathlib import Path
import numpy as np

from pipe_scene import HERE, ROOT, nav, robot_scene, carried
from features import VisibleFasteners
from coarse_vision import coarse_observation


def run(assets):
    root=assets/"observed-reef-a"
    root.mkdir(exist_ok=True)
    baseline=nav.read(ROOT/"conditioned-match/slow-navigation.json")
    envelope=nav.checked_scene(assets)
    rows=[]
    for side,entry in baseline.items():
        config=nav.read(ROOT/"station-landmark/routes.json")["cases"][side]
        for mode in ("nominal","wrong-face","dropout","stale","ambiguous","shared-survey-shift"):
            features=VisibleFasteners(envelope)
            folder=root/side
            folder.mkdir(exist_ok=True)
            r=nav.sector.tracecase("coral-A",entry["initial"],mode,envelope,folder,
                wheel_scale=config["wheelScale"],physical_track_mm=config["effectiveTrackMm"],
                observer=coarse_observation,features=features)
            r.pop("path")
            r.update(wheelCase=side,raysUsed={str(k):sum(n==k for n,_ in features.audit) for k in (2,3,4)})
            rows.append(r)
            print(side,mode,r["accepted"],r["reason"],r["decodedTagFrames"],r["pairedHeadFrames"],r["latestInterval"],r["finalBodyErrorMmDeg"],flush=True)
    (HERE/"field-trials.json").write_text(json.dumps(rows,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("assets",type=Path);run(p.parse_args().assets)
