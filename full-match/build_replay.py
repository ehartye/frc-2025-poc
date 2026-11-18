"""Build a standalone student replay from the actual programmed six-robot trajectories."""
import json
from pathlib import Path
from match import heavy, corrected, trial, basis, COORDS

HERE=Path(__file__).resolve().parent
f=corrected()
rec,sources=basis(f)
scenarios={}
for label,options in (("Scheduled departure",{}),("Partner remains at start",{"vacate_lane":False}),
                      ("Return to station",{"park":False})):
    result,team=trial(f,rec,sources,**options)
    auto=f.routes(-.18)[0]
    rows=[]
    for i in range(307):
        wall=i/2
        bots=[]
        for key,program in team.items():
            if wall<18:
                if key=="red2":
                    pose=auto.at(min(wall,auto.duration))[0]
                    action="Autonomous station approach" if wall<auto.duration else "Stopped at station"
                else:
                    pose,action=program.initial,"Hold starting position"
            else:
                pose,_,action=program.at(wall-18)
            bots.append([round(pose[0]/f.IN,2),round(pose[1]/f.IN,2),round(pose[2]*180/3.141592654,2)])
            if key=="red2":
                operation=action
        rows.append([bots,operation])
    scenarios[label]={"frames":rows,"bots":list(team),"result":result}
data={"scenarios":scenarios,"stacks":COORDS,"solids":[[name,[[round(x/f.IN,2),round(y/f.IN,2)] for x,y in poly]]
         for name,poly in f.obstacles()],"target":[f.LIP[0]/f.IN,f.LIP[1]/f.IN]}
old=heavy.revised_motion().routes(-.18)[0]
data["legacy"]=[[round(v/f.IN,2) for v in old.at(i/50)[0][:2]] for i in range(int(old.duration*50)+1)]
data["hit"]=COORDS[-1]
installed=json.loads((HERE/"installed-stock.json").read_text())
data["reefBalls"]=[]
for key,obj in installed["stock"].items():
    if obj["group"]=="reef-ball":
        x,up,z=obj["native_center_mm"]
        fx,fy=f.field_from_native(x/25.4,z/25.4)
        data["reefBalls"].append([key,round(fx/f.IN,3),round(fy/f.IN,3),round(up),installed["ball_radius_mm"]/25.4])
template=(HERE/"replay-template.html").read_text(encoding="utf8")
(HERE/"replay.html").write_text(template.replace("__DATA__",json.dumps(data,separators=(",",":"))),encoding="utf8",newline="\n")
