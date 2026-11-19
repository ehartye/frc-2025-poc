"""Replay a stationary post-buzzer bumper observation through the installed route checker."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"leave-park"))
from body_route import pin,write,read


def replay(plugin,out):
    out.mkdir(parents=True,exist_ok=True)
    nr=HERE/"native.json"
    observed=read(nr)
    body="bumper-cover-front-red"
    covers=list(observed["parkClothInZoneMm3"])
    kit=ROOT/"design/competition-package/shop-duplex/red"
    geometry=pin(kit/"stowed.step",out)
    robot=write(out,"robot",{"schema":"field-route/robot@1","scope":"subassembly","geometry":geometry,
                "catalog":pin(nr,out),"collection":{"path":["parkClothInZoneMm3"],"format":"keys"}})
    stock_source=ROOT/"full-match/installed-stock.json"
    boxes={k:[v/1000 for v in row["native_bounds_mm"]] for k,row in read(stock_source)["stock"].items()}
    field=write(out,"field",{"source":pin(stock_source,out),"placed":boxes})
    inventory=write(out,"inventory",{"schema":"field-route/inventory@1","clock":"teleop","startS":135,
             "fieldSha256":field["sha256"],"scope":{"kind":"bounded-study","name":"parked covers and installed stock","unresolved":[]},
             "collections":[{"path":["placed"],"format":"keys","initial":"placed","bounds":"value"}]})
    interval={"fromS":135,"toS":138,"carried":[]}
    sources={"robot":robot,"field":field,"transform":pin(ROOT/"mission/field-registration.json",out),
             "trajectory":pin(HERE/"match.json",out)}
    scene=write(out,"scenario",{"schema":"field-route/scenario@1","clock":"teleop","startS":135,"marginM":0,
             "robotBodies":covers,"pieces":[],"obstacles":list(boxes),"intervals":[interval],"sources":sources})
    basis={"clock":"teleop","scenarioSha256":scene["sha256"]}
    event={"id":"stationary-front-bumper","robotBody":body,"timeS":138}
    proof=write(out,"measure",{"schema":"field-route/body-observations@1",**basis,"geometrySha256":geometry["sha256"],
             "observations":[{**event,"criterion":"red front cloth overlaps own barge zone","unit":"mm3","measurement":pin(nr,out),
               "path":["parkClothInZoneMm3",body],"comparison":"above","threshold":0}]})
    detector=write(out,"detector",{"schema":"field-route/events@1",**basis,"sourceEvidence":proof,"events":[event]})
    x,y,_=observed["parkPose"]
    profile=read(ROOT/"kickoff/season-profile.json")["field"]
    cx=x-profile["length"]*.0254/2
    cz=profile["width"]*.0254/2-y
    whole=[cx-.425,0,cz-.425,cx+.425,1.059,cz+.425]
    sweep=write(out,"sweep",{"schema":"field-route/sweep@1",**basis,"sourceEvidence":pin(nr,out),"refinements":[],
          "intervals":[{**interval,"method":"continuous-swept-enclosures","moving":dict.fromkeys(covers,whole),"obstacles":boxes}]})
    req=write(out,"request",{"scenario":scene,"route":sweep,"detector":detector,"inventory":inventory})
    result=json.loads(subprocess.check_output(["node",str(plugin/"scripts/field-route-evidence.mjs"),str(out/req["file"])],text=True))
    row=result["events"][0]
    if (row["routeStatus"],row["wholeFieldStatus"])!=("supported","pending"):
        raise ValueError(row)
    return {"pluginVersion":read(plugin/"package.json")["version"],"route":row["routeStatus"],"wholeField":row["wholeFieldStatus"],
            "subject":body,"volumeMm3":row["bodyObservation"]["value"],"scope":"two stationary bumper covers, 24 installed stock pieces, TELEOP 135–138 s"}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("plugin",type=Path)
    p.add_argument("output",type=Path)
    args=p.parse_args()
    (HERE/"park-event.json").write_text(json.dumps(replay(args.plugin,args.output),indent=2)+"\n",encoding="utf-8",newline="\n")
