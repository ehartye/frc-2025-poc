"""Reopen the issued robot and jack. Detect material neighbors independently of joint labels."""

import argparse
import csv
import functools
import hashlib
import json
import math
import sys
from collections import Counter, defaultdict, deque
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "cad/shallow-cage"))
from installed import load_parts, volume
from clearance import bounds, read_assembly
from OCP.BRepExtrema import BRepExtrema_DistShapeShape


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def reopen():
    base = ROOT / "cad/swerve-captive"
    register = json.loads((base / "appearances.json").read_bytes())
    source = base / "public-robot.step"
    assert sha(source) == register["sourceSha256"]
    declared = {r["id"]: r for r in register["solids"]}
    items = []
    for group, name, shape in read_assembly(source):
        row = declared[name]
        if group == "Demonstration CORAL":
            continue
        items.append((name, group, row["role"], row["representation"], row["source"], shape))
    added, parts = load_parts(ROOT / "cad/shallow-cage")
    for name,(group,shape) in parts.items():
        src = "cad/shallow-cage/" + added["added"][group]["step"]
        rep = "installation-envelope" if "envelope" in name else "project-cad"
        items.append((name,group,"jack",rep,src,shape))
    assert len(items) == len({i[0] for i in items}) == 1030
    return items


def box_gap(a,b):
    return math.sqrt(sum(max(0.,b[i]-a[i+3],a[i]-b[i+3])**2 for i in range(3)))


def run(destination):
    items = reopen()
    boxes = [bounds(i[-1]) for i in items]
    contacts = defaultdict(list)
    nearest = {}
    pairs = []
    # 1 mm includes small running clearances. Larger finite gaps never count as touching.
    for i,a in enumerate(items):
        for j in range(i):
            b = items[j]
            if box_gap(boxes[i],boxes[j]) > 1.00001:
                continue
            dist=BRepExtrema_DistShapeShape(a[-1],b[-1])
            if not dist.IsDone(): raise ValueError((a[0],b[0]))
            gap=dist.Value()
            if gap<=1.00001:
                pairs.append((a[0],b[0],round(gap,5)))
                for current,other in [(a,b),(b,a)]:
                    if gap < nearest.get(current[0],(math.inf,None))[0]:
                        nearest[current[0]]=(gap,other[0])
                    if gap<=.020:
                        contacts[current[0]].append(other[0])
    def groups(graph):
        sets=[];remaining={r[0] for r in items}
        while remaining:
            component={min(remaining)}
            queue=deque(component)
            while queue:
                m=queue.popleft()
                for n in graph[m]:
                    if n not in component:
                        component.add(n);queue.append(n)
            remaining-=component
            sets.append(sorted(component))
        return sorted(sets,key=lambda g:(-len(g),g[0]))
    with (destination/"neighbors.csv").open("w",newline="",encoding="utf8") as f:
        w=csv.writer(f,lineterminator="\n");w.writerow(("first","second","gapMm"));w.writerows(pairs)
    with (destination/"installed-bodies.csv").open("w",newline="",encoding="utf8") as f:
        w=csv.writer(f,lineterminator="\n");w.writerow(("id","assembly","appearanceRole","representation","cadSource","volumeMm3","xMin","yMin","zMin","xMax","yMax","zMax","neighborsWithin20um","nearestWithin1mm","nearestGapMm"))
        for i,(name,group,role,rep,src,s) in enumerate(items):
            d,n=nearest.get(name,(None,""))
            w.writerow([name,group,role,rep,src,round(volume(s),3),*[round(x,4) for x in boxes[i]],len(contacts[name]),n,round(d,5) if d is not None else ""])
    clusters=groups(contacts)
    register={"baseStepSha256":sha(ROOT/"cad/swerve-captive/public-robot.step"),"jackNativeSha256":json.loads((ROOT/"cad/shallow-cage/reopen.json").read_bytes())["sha256"],
              "robotBodies":len(items),"pairCountWithin1mm":len(pairs),"componentsWithin20um":len(clusters),"largestComponentCount":len(clusters[0]),
              "isolatedAssemblies":clusters[1:],"scope":"One stowed geometry. Material contact can indicate support, incidental collision, a marking or a visual envelope. Fastening, running degrees of freedom, bolt preload, welds, static friction and wire terminations require separate inspection."}
    (destination/"audit.json").write_text(json.dumps(register,indent=2)+"\n",encoding="utf8")


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("destination",type=Path,default=HERE,nargs="?")
    run(parser.parse_args().destination)
