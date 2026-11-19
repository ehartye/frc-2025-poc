"""Build project-owned body observations and replay them through installed Agent FRC."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
KIT=ROOT/"design/competition-package/shop-duplex/red"


def read(path):
    return json.loads(path.read_bytes())


def pin(path,base):
    return {"file":Path(os.path.relpath(path,base)).as_posix(),"sha256":hashlib.sha256(path.read_bytes()).hexdigest()}


def write(base,name,data):
    path=base/(name+".json")
    path.write_bytes((json.dumps(data,separators=(",",":"),allow_nan=False)+"\n").encode())
    return pin(path,base)


def peer_box(rect,profile):
    # Metres in the official native field frame: X along length, Y up, Z across width.
    mid_x=profile["length"]*.0254/2
    mid_z=profile["width"]*.0254/2
    return [rect[0]-mid_x,-.01,mid_z-rect[3],rect[2]-mid_x,1.2,mid_z-rect[1]]


def separate(a,b):
    return any(a[i+3]<b[i] or b[i+3]<a[i] for i in range(3))


def replay(plugin,out,trial,adverse=False):
    source=HERE/"native-park.json"
    data=read(source)
    row=data["cases"][trial]
    ids=list(read(KIT/"part-frames.json"))
    out.mkdir(parents=True,exist_ok=True)
    source_ref=pin(source,out)
    geometry=pin(KIT/"stowed.step",out)
    if geometry["sha256"]!=data["robot_step_sha256"] or len(ids)!=data["body_count"]:
        raise ValueError("Robot geometry does not match the native trial")
    robot=write(out,"robot",{"schema":"field-route/robot@1","scope":"assembly","geometry":geometry,
          "catalog":pin(KIT/"part-frames.json",out),"collection":{"path":[],"format":"keys"}})
    boxes={k:[v/1000 for v in b] for k,b in data["local_hardware_native_boxes_mm"].items()}
    profile=read(ROOT/"kickoff/season-profile.json")
    boxes["red1"]=peer_box(row["south_red1_world_xy"] if adverse else row["reserved_red1_world_xy"],profile["field"])
    field=write(out,"field",{"hardwareSource":data["field_step_sha256"],"obstacles":boxes})
    inventory=write(out,"inventory",{"schema":"field-route/inventory@1","fieldSha256":field["sha256"],"clock":"match",
             "startS":0,"scope":{"kind":"bounded-study","name":"local static field hardware and reserved partner",
             "unresolved":[]},"collections":[{"path":["obstacles"],"format":"keys","initial":"placed","bounds":"value"}]})
    interval={"fromS":0,"toS":153,"carried":[]}
    scenario=write(out,"scenario",{"schema":"field-route/scenario@1","clock":"match","startS":0,"marginM":0,
             "robotBodies":ids,"pieces":[],"obstacles":list(boxes),"intervals":[interval],"sources":{"robot":robot,
             "field":field,"transform":pin(ROOT/"mission/field-registration.json",out),"trajectory":source_ref}})
    bound={"clock":"match","scenarioSha256":scenario["sha256"]}
    subject="bumper-cover-front-red"
    event={"id":"end-bumper-in-zone","robotBody":subject,"timeS":153}
    observer=write(out,"observer",{"schema":"field-route/body-observations@1",**bound,"geometrySha256":geometry["sha256"],
             "observations":[{**event,"criterion":"front bumper cover overlaps red zone at the buzzer","unit":"mm3",
             "measurement":source_ref,"path":["cases",trial,"end_park_cover_mm3",subject],"comparison":"above","threshold":0}]})
    detector=write(out,"detector",{"schema":"field-route/events@1",**bound,"sourceEvidence":observer,"events":[event]})
    swept=[v/1000 for v in row["native_swept_box_mm"]]
    refinements=[]
    ref_base={**bound,"sourceEvidence":source_ref,"backend":"OpenCascade","fromS":0,"toS":153}
    # A distance proven for the enclosing chassis prism is a lower bound for each contained body.
    for k,box in boxes.items():
        if k=="red1" or separate(swept,box):continue
        for body in ids:
            refinements.append(write(out,"bound-"+str(len(refinements)),{**ref_base,"moving":body,"obstacle":k,
                "method":"continuous-lower-bound","minGapM":row["static_field_lower_bound_mm"]/1000}))
    if adverse:
        for body,volume in row["initial_overlap_against_south_partner_mm3"].items():
            if volume>.01:
                refinements.append(write(out,"contact-"+body,{**ref_base,"timeS":0,"moving":body,"obstacle":"red1",
                    "method":"material-intersection","volumeMm3":volume,"toleranceMm3":.01}))
    sweep=write(out,"sweep",{"schema":"field-route/sweep@1",**bound,"sourceEvidence":source_ref,"refinements":refinements,
             "intervals":[{**interval,"method":"continuous-swept-enclosures","moving":dict.fromkeys(ids,swept),"obstacles":boxes}]})
    req=write(out,"request",{"scenario":scenario,"route":sweep,"detector":detector,"inventory":inventory})
    program=plugin/"scripts/field-route-evidence.mjs"
    result=json.loads(subprocess.check_output(["node",str(program),str(out/req["file"])],text=True))
    obs=result["events"][0]
    expected="infeasible" if adverse else "supported"
    if obs["routeStatus"]!=expected:raise ValueError((trial,obs["routeStatus"],obs["gaps"][:1]))
    return {"routeStatus":obs["routeStatus"],"wholeFieldStatus":obs["wholeFieldStatus"],
            "body":subject,"volumeInZoneMm3":obs["bodyObservation"]["value"],"contacts":obs["qualifiedContacts"]}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plugin",type=Path)
    parser.add_argument("output",type=Path,help="directory for generated project inputs and compact result")
    args=parser.parse_args()
    plugin=args.plugin.resolve()
    cases={key:replay(plugin,args.output/key,trial,adverse) for key,trial,adverse in
           [("issued","issued",False),("shorter","shorter",False),("near-tape","near-tape",False),
            ("retained-partner","shared-105",False),("occupied-start","issued",True)]}
    report={"plugin":read(plugin/"package.json")["version"],"cases":cases}
    write(args.output,"review",report)
    print(json.dumps(report,indent=2))


if __name__=="__main__":
    main()
