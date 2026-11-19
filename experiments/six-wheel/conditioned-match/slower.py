"""Screen a rate-reduced loaded transit while retaining the fresh sector observation interlock."""
import argparse
import json
from pathlib import Path
from supply import n,run

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("assets",type=Path);args=p.parse_args()
    ctx=n.checked_scene(args.assets)
    result={}
    for side,station in n.read(n.HERE/"opening.json").items():
        result[side]=n.outbound("coral-A",station["sourceRobotMmDeg"],side,
                                args.assets/"conditioned-match"/side/"slow/A",ctx,use_gyro=True,pace=.85)
    (n.HERE/"slow-navigation.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf8",newline="\n")
    run(args.assets,True,True)
