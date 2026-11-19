"""Build a local student replay from verified field coordinates and the mission ledger."""
import argparse
import json
from pathlib import Path
import numpy as np
from transport import trajectory,HERE

def rows(t,p):
    return [[round(t[i],4),*[round(float(v),6) for v in p[i]]] for i in sorted(set(range(0,len(t),20))|{len(t)-1})]

def run(assets):
    match=json.loads((HERE/"match.json").read_bytes())
    paths={}
    for side in ("leftSmall","rightSmall"):
        a=np.load(assets/"conditioned-match"/side/"corrected-opening.npz")
        ended=rows(a["t"],a["pose"])
        t,p=trajectory(assets,side,"corrected")
        paths[side]={"station":ended,"out":ended+rows(27.032172635314284+t,p)}
    payload=json.dumps({"match":match,"paths":paths},separators=(",",":"),allow_nan=False).replace("</","<\\/")
    page=(HERE/"replay.template.html").read_text(encoding="utf8").replace("REPLAY_DATA",payload)
    (HERE/"replay.html").write_text(page,encoding="utf8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("assets",type=Path);a=p.parse_args();run(a.assets)
