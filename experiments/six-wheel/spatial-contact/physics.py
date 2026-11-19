"""Join each attained state to the registered trough and time-resolved rack motion."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def panel_file(target, path):
    slope = target["floor_slope"]
    scale = math.hypot(1, slope)
    with path.open("w", newline="") as file:
        w = csv.writer(file, lineterminator="\n")
        w.writerow(["profile", 0, slope, target["floor_intercept_m"], target["lip_depth_m"], target["lip_top_m"]])
        for i, panel in enumerate(target["panels"]):
            points = [(x*scale,y) for x,y in panel["bounds_xy"]]
            edges = list(zip(points, points[1:] + points[:1]))
            sign = math.copysign(1, sum(x*v-y*u for (x,y),(u,v) in edges))
            for (x,y),(u,v) in edges:
                length = math.hypot(u-x,v-y)
                a,b = sign*(y-v)/length,sign*(u-x)/length
                w.writerow(["edge",i,a,b,-a*x-b*y])
            for x,y,r in panel["holes_xy_r"]:
                w.writerow(["hole",i,x*scale,y,r+target["panel_inset_m"]])


def run(cache, rack, jdk, hal=None, quick=False):
    rows = json.loads((HERE/"source.json").read_bytes())
    target = json.loads((ROOT/"mission/target.json").read_bytes())
    clocks = list(csv.DictReader((hal or ROOT/"integrated-match/events.csv").open()))
    classes = cache/"classes"
    classes.mkdir(parents=True, exist_ok=True)
    panel_file(target, cache/"panels.csv")
    plans = json.loads((ROOT/"fixed-height-match/placements.json").read_bytes())["placements"]
    if hal:
        registered = json.loads((HERE/"hal.json").read_bytes())["pathSHA256"]
        if set(registered.values()) != {digest(rack)}:raise ValueError("A single shared rack curve does not cover every run")
    cases = []
    for row in rows:
        pipe,mode = row["pipe"],row["mode"]
        cmd = next(r for r in clocks if r["scenario"]=="nominal" and r["event"]=="withdraw" and r["piece"]==pipe and r.get("approach",mode)==mode)
        plan = next(p for p in plans if p["id"]==pipe)
        lip_ms = 100 if pipe=="coral-A" else 119
        epoch_us = round((18+plan["lip"])*1e6-lip_ms*1000)
        wall = epoch_us/1e6 + row["endCameraMs"]/1000
        command = int(cmd["wall_ms"])/1000 - wall
        initial = [*row["positionM"], *row["velocityMps"], *row["axis"], *row["angularRadps"], *row["chassisAtReefXYm"],row["chassisHeadingRad"]]
        def add(label, start=command, duration=2.5,dt=.0002,n=9,mu=.35):
            cases.append([pipe+"-"+mode+"-"+label,*initial,start,duration,dt,n,mu])
        add("scheduled",duration=3 if quick else 156-wall)
        add("held", start=1e3)
        if mode=="nominal":
            add("late",start=command+.4)
            add("fine",dt=.0001,n=17)
            add("finer",dt=.00005,n=33)
            add("low-friction",mu=.10)
            add("high-friction",mu=.55)
    with (cache/"cases.csv").open("w",newline="") as f:
        csv.writer(f,lineterminator="\n").writerows(cases)
    subprocess.run([str(jdk/"bin/javac"),"-d",str(classes),str(HERE/"SpatialContact.java")],check=True)
    subprocess.run([str(jdk/"bin/java"),"-cp",str(classes),"SpatialContact",*[str(p) for p in (cache/"panels.csv",rack,cache/"cases.csv",cache/"physics.csv",cache/"trace.csv")]],check=True)
    results = list(csv.DictReader((cache/"physics.csv").open()))
    out={"targetSHA256":digest(ROOT/"mission/target.json"),"attainedSHA256":digest(HERE/"source.json"),"rackSHA256":digest(rack),
         "clockSHA256":digest(hal or ROOT/"integrated-match/events.csv"),"clockBasis":"historical motion timing sensitivity" if hal is None else "fresh corrected-path optical/controller replay",
         "cases":[{k:(v if k in ("case","outcome") else float(v)) for k,v in r.items()} for r in results]}
    (HERE/"physics.json").write_text(json.dumps(out,indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("cache","rack","jdk"):p.add_argument(name,type=Path)
    p.add_argument("--hal",type=Path);p.add_argument("--quick",action="store_true")
    a=p.parse_args()
    run(a.cache,a.rack,a.jdk,a.hal,a.quick)
