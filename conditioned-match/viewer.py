"""Build a local student replay from verified field coordinates and the mission ledger."""
import argparse
import json
from pathlib import Path
import numpy as np
from transport import trajectory,HERE,nav

def rows(t,p):
    return [[round(t[i],4),*[round(float(v),6) for v in p[i]]] for i in sorted(set(range(0,len(t),20))|{len(t)-1})]

def run(assets):
    match=json.loads((HERE/"match.json").read_bytes())
    paths={}
    for side in ("leftSmall","rightSmall"):
        opening=assets/"conditioned-match"/side/"corrected-opening.npz"
        if nav.sha(opening)!=nav.read(HERE/"opening.json")[side]["pathSha256"]:
            raise ValueError("Changed opening route")
        a=np.load(opening)
        ended=rows(a["t"],a["pose"])
        t,p=trajectory(assets,side,"slow")
        start=nav.read(nav.ROOT/"relieved-match/first-feed.json")["scheduledDriveWallS"]
        paths[side]={"station":ended,"out":ended+rows(start+t,p)}
    plan=nav.read(nav.ROOT/"fixed-height-match/placements.json")["placements"]
    field,_,teams,_=nav.route.route.field.scenario(plan,inboard=True)
    peers={}
    for name,robot in teams.items():
        if name=="red2":
            continue
        peers[name]=[[round(float(x),4) for x in robot.at(max(0.,min(wall,153.)-18.))[0]] for wall in np.arange(0.,156.25,.25)]
    stock=[]
    for piece in nav.read(nav.ROOT/"full-match/installed-stock.json")["stock"].values():
        x,_,z=piece["native_center_mm"]
        xy=field.field_from_native(x/25.4,z/25.4)
        stock.append([piece["group"],*[round(float(v),4) for v in xy]])
    geometry={"polygons":[np.asarray(poly).tolist() for _,poly in field.obstacles()],"stock":stock,"peers":peers}
    payload=json.dumps({"match":match,"paths":paths,"geometry":geometry},separators=(",",":"),allow_nan=False).replace("</","<\\/")
    page=(HERE/"replay.template.html").read_text(encoding="utf8").replace("REPLAY_DATA",payload)
    (HERE/"replay.html").write_text(page,encoding="utf8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("assets",type=Path);a=p.parse_args();run(a.assets)
