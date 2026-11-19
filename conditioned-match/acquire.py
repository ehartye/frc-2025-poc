"""Run the existing Java two-port controller at the rematched, powered opening docks."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"shelf-contact"))
from ports import replay, Shelf, feed

def simulate(args):
    name,damping,dt,cls,jdk,cache,delta=args
    return replay((name,Shelf(dissipation_s_m=damping,stations=16),dt,cls,jdk,cache,True),attained_pose=delta)

def run(assets,jdk):
    inputs=json.loads((HERE/"opening.json").read_bytes())
    cache=assets/"conditioned-match"/"corrected-feeds";cache.mkdir(parents=True,exist_ok=True)
    expected=json.loads((ROOT/"station-landmark/feed.json").read_bytes())["programSha256"]
    with tempfile.TemporaryDirectory() as tmp:
        cls=Path(tmp);java=cls/"PortAcquisition.java"
        java.write_text(feed.port_program(True),encoding="utf8",newline="\n")
        if hashlib.sha256(java.read_bytes()).hexdigest()!=expected:raise ValueError("Unexpected intake policy")
        subprocess.run([str(jdk/"bin/javac"),"-d",str(cls),str(java),str(ROOT/"observable-acquisition/PortConsole.java")],check=True)
        jobs=[(side,d,dt,cls,jdk,cache,row["sourceRobotMmDeg"]) for side,row in inputs.items()
              for d,dt in ((4,.0000625),(4,.00003125),(1,.0000625)) if row["status"]=="observed stationary"]
        with ProcessPoolExecutor(max_workers=6) as pool:rows=list(pool.map(simulate,jobs))
    (HERE/"feeds.json").write_text(json.dumps(rows,indent=2)+"\n",encoding="utf8",newline="\n")
    for r in rows: print(r["case"],r["dtS"],r["shelf"]["dissipation_s_m"],r["state"],r["firstBandMs"],flush=True)

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("assets",type=Path);p.add_argument("jdk",type=Path);a=p.parse_args()
    run(a.assets,a.jdk)
