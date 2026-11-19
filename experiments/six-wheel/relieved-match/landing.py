"""Screen candidate rack timings against the actual finite tubes and the registered L1 trough."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"spatial-contact"))
from physics import panel_file


def run(cache,jdk,rack,issued=None,flat=False,field_trace=False,*,scenario="nominal",registration=HERE,output=None):
    cache.mkdir(exist_ok=True)
    read=lambda p:json.loads(p.read_bytes())
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    panel_file(read(ROOT/"mission/target.json"),cache/"panels.csv")
    if issued:
        registration=read(registration/"hal.json")
        if sha(issued)!=registration["events"]:
            raise ValueError("Unregistered controller events")
        for piece in ("coral-A","coral-B"):
            name=f"path-{scenario}-{piece}.csv"
            if sha(cache/name)!=sha(rack) or sha(rack)!=registration["rackPaths"][name]:
                raise ValueError("Rack traces disagree with the issued controller output")
    cases=[]
    for piece in ("coral-A","coral-B"):
        for resolution in (250,500):
            row=read(HERE/f"{piece}-{'coupled' if flat else 'pads-edge'}-{resolution}.json")
            state=[*row["positionM"],*row["velocityMps"],*row["axis"],*row["angularRadps"],*row["chassisAtReefXYm"],row["chassisHeadingRad"]]
            if field_trace:
                cases.append([f"{piece}/{resolution}/extended",*state,10000,.25,.0001,17,.35])
                continue
            for delay in (list(range(75 if flat else 20,161))+[240,520,10000] if issued is None else []):
                cases.append([f"{piece}/{resolution}/{delay}",*state,delay/1000,3,.00020,9,.35])
            if issued:
                commands=list(csv.DictReader(issued.open()))
                command=next((int(x["wall_ms"])/1000 for x in commands if x["scenario"]==scenario and x["piece"]==piece and x["event"]=="withdraw"),None)
                wait=10000 if command is None else command-row["wallS"]
                cases.append([f"{piece}/{resolution}/observed",*state,wait,156-row["wallS"],.00020,9,.35])
    label="field" if field_trace else "observed" if issued else "windows"
    local=cache/("landing-"+label)
    local.mkdir(exist_ok=True)
    with (local/"cases.csv").open("w",newline="") as f:
        csv.writer(f,lineterminator="\n").writerows(cases)
    subprocess.run([str(jdk/"bin/javac"),"-d",str(local),str(ROOT/"spatial-contact/SpatialContact.java")],check=True)
    extra=[".0005",".23"] if field_trace else ["sweep-y"] if issued else []
    subprocess.run([str(jdk/"bin/java"),"-cp",str(local),"SpatialContact",str(cache/"panels.csv"),str(rack),str(local/"cases.csv"),str(local/"results.csv"),str(local/"trace.csv"),*extra],check=True)
    rows=list(csv.DictReader((local/"results.csv").open()))
    location=output if output is not None else HERE/"flat-control" if flat else HERE
    with (location/("landing-"+label+".csv")).open("w",newline="") as f:
        out=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator="\n")
        out.writeheader()
        out.writerows(rows)
    print(sha(rack),[(p,r["outcome"],r.get("min_front_m")) for p in ("coral-A","coral-B") for r in rows if r["case"].startswith(p+"/250/") and r["case"].split("/")[-1] in ("75","115","160","10000","observed")],flush=True)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for n in ("cache","jdk","rack"):p.add_argument(n,type=Path)
    p.add_argument("--issued",type=Path)
    p.add_argument("--flat",action="store_true")
    p.add_argument("--field-trace",action="store_true")
    a=p.parse_args()
    run(a.cache,a.jdk,a.rack,a.issued,a.flat,a.field_trace)
