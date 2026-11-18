"""Run the installed Agent FRC field-route checker on the initial stationary floor-piece inventory."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess

from match import COORDS, MARKS, ROOT, corrected
from mission.heavy_field_clock import revised_motion
from mission.review_combined_departure import digest
import build_assembly as cad


def case(folder,poses,mode,plugin):
    folder.mkdir(parents=True,exist_ok=True)
    def ref(p):
        return {"file":os.path.relpath(p,folder).replace(os.sep,"/"),"sha256":digest(p)}
    def write(name,data):
        p=folder/(name+".json")
        p.write_text(json.dumps(data,separators=(",",":"),allow_nan=False)+"\n",encoding="utf8",newline="\n")
        return ref(p)
    registered=ROOT/"full-match"
    geometry=json.loads((registered/"red-assembly.json").read_text())
    native=json.loads((registered/"stack-negative.json").read_text())
    trial=json.loads((registered/"field-trial.json").read_text())
    f=corrected() if mode=="revised" else revised_motion()
    route=f.routes(-.18)[0]
    end=18+trial["hold_variants"]["0"]["scheduled_station_lip_arrivals_s"][0]
    shapes=cad.read_named(poses/"stowed.step",[{"id":k} for k in geometry["part_ids"]])
    if digest(poses/"stowed.step")!=native["stowed_sha256"]:
        raise ValueError("Negative source differs from the supplied robot")
    bounds={k:[v/1000 for v in cad.bounds(s)] for k,s in shapes.items()}
    ids=list(bounds)
    field={}
    for i,(x,y) in enumerate(COORDS):
        name=("red" if i>=3 else "blue")+"-"+("lower","middle","upper")[i%3]
        for piece,half,z1,z2 in (("coral",6.4,0,4.5),("algae",14.7,4.5,21)):
            field[name+"-"+piece]=[(x-half)*f.IN,(y-half)*f.IN,z1*f.IN,(x+half)*f.IN,(y+half)*f.IN,z2*f.IN]
    obstacles=list(field)
    selected=[o for o in obstacles if mode!="underdeclared" or not o.startswith("red-upper-")]
    # Caller-declared scene inventory plus independently pre-existing source register.
    sources={"field":write("expected-field",{"slab_obstacles":field,"source":ref(registered/"field-trial.json")}),
             "robot":ref(poses/"stowed.step"),"transform":ref(ROOT/"mission/field-registration.json"),
             "trajectory":write("trajectory",{"segments":route.segments,"stopped_from":route.duration,"first_lip_s":end,
                     "basis":"First station lip event is hypothetical on the rejected historical path."})}
    cuts=[0]+route.times+[end]
    slices=[{"fromS":a,"toS":b,"carried":[]} for a,b in zip(cuts,cuts[1:])]
    scenario=write("scenario",{"schema":"field-route/scenario@1","sources":sources,"clock":"match-wall-seconds",
        "startS":0,"marginM":0,"robotBodies":ids,"pieces":["coral-A"],"obstacles":selected,"intervals":slices})
    bound={"scenarioSha256":scenario["sha256"],"clock":"match-wall-seconds"}
    event=write("event",{"schema":"field-route/events@1",**bound,"sourceEvidence":ref(registered/"field-trial.json"),
        "events":[{"id":"first-station-lip","piece":"coral-A","timeS":end}]})
    refinements=[]
    observations=[]
    for i,part in enumerate(slices):
        s=route.segments[i] if i<len(route.segments) else {"kind":"station-feed","start":route.end,"end":route.end}
        a,b,h=s["start"]
        ex,ey,_=s["end"]
        moving={}
        for k,box in bounds.items():
            corners=[f.rot((x,y),h) for x in (box[0],box[3]) for y in (box[1],box[4])]
            if s["kind"]=="turn":
                radius=max(math.hypot(x,y) for x,y in corners)
                moving[k]=[a-radius,b-radius,box[2],a+radius,b+radius,box[5]]
            elif s["kind"]=="station-feed":
                # Known full robot outer footprint for every stowed/receive
                # and intermediate position; broad enclosure only.
                radius=.425*math.sqrt(2)
                moving[k]=[a-radius,b-radius,0,a+radius,b+radius,1.059]
            else:
                xs,ys=zip(*corners)
                moving[k]=[min(a,ex)+min(xs),min(b,ey)+min(ys),box[2],max(a,ex)+max(xs),max(b,ey)+max(ys),box[5]]
        covered={k:field[k] for k in selected if mode!="missing-sweep" or not k.startswith("red-upper-")}
        observations.append({**part,"method":"continuous-swept-enclosures","moving":moving,"obstacles":covered})
        for o,box in covered.items():
            # Refine the full 850-mm body, a superset of every native solid.
            poly=[(box[0],box[1]),(box[3],box[1]),(box[3],box[4]),(box[0],box[4])]
            if s["kind"]=="turn":
                dx=max(box[0]-a,0,a-box[3]);dy=max(box[1]-b,0,b-box[4])
                gap=math.hypot(dx,dy)-.425*math.sqrt(2)
            else:
                hull=[f.rot((x,y),h) for x,y in ((-.425,-.425),(.425,-.425),(.425,.425),(-.425,.425))]
                hit,gap=f.sweep.sweep((a,b),(ex,ey),hull,poly)
                if hit is not None:
                    gap=0
            for k,body in moving.items():
                overlaps=all(body[j+3]>=box[j] and box[j+3]>=body[j] for j in range(3))
                if not overlaps:
                    continue
                pair={**bound,"fromS":part["fromS"],"toS":part["toS"],"moving":k,"obstacle":o,
                      "backend":"registered-planar-sweep","sourceEvidence":sources["trajectory"]}
                contacts=[v for v in native["hits"] if v["moving"]==k and v["obstacle"].removesuffix("-axis-y")==o
                          and part["fromS"]<=v["timeS"]<=part["toS"]]
                if contacts and mode!="revised":
                    v=contacts[0]
                    pair.update(method="material-intersection",timeS=v["timeS"],volumeMm3=v["volumeMm3"],
                                toleranceMm3=.05,backend="OpenCascade",sourceEvidence=ref(registered/"stack-negative.json"))
                elif gap>0:
                    pair.update(method="continuous-lower-bound",minGapM=gap)
                else:
                    continue
                refinements.append(write("pair-"+str(len(refinements)),pair))
    sweep=write("sweep",{"schema":"field-route/sweep@1",**bound,"sourceEvidence":sources["trajectory"],
                         "intervals":observations,"refinements":refinements})
    write("request",{"scenario":scenario,"route":sweep,"detector":event})
    exe=plugin/"scripts/field-route-evidence.mjs"
    result=subprocess.run(["node",str(exe),str(folder/"request.json")],capture_output=True,text=True)
    data=json.loads(result.stdout)
    row=data["events"][0]
    return {"plugin":json.loads((plugin/".codex-plugin/plugin.json").read_text())["version"],
            "exit_code":result.returncode,"declared_robot_ids":len(ids),"registered_floor_ids":len(obstacles),
            "declared_floor_ids":len(selected),"event":row["detectorEvent"]["id"],"status":row["routeStatus"],
            "native_hits":len(row["contacts"]),
            "gaps":len(row.get("gaps",[])),"full_result":data}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for a in ("poses","plugin","working","output"):
        p.add_argument("--"+a,type=Path,required=True)
    a=p.parse_args()
    rows={}
    for mode in ("revised","prior","missing-sweep","underdeclared"):
        row=case(a.working/mode,a.poses,mode,a.plugin)
        full=row.pop("full_result")
        (a.working/mode/"result.json").write_text(json.dumps(full,indent=2),encoding="utf8")
        rows[mode]=row
    a.output.write_text(json.dumps(rows,indent=2)+"\n",encoding="utf8",newline="\n")
