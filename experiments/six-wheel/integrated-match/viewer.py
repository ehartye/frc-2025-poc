"""Build the self-contained student field replay from the qualified schedules and timed observations."""
import csv
import json
import math
from pathlib import Path
from field import ROOT,HERE,read,fieldcase,field
from match import COORDS

ACTIONS={"feed":"Assign station pipe","seat":"Confirm stationary seat","carry":"Authorize loaded drive","release":"Run release profile",
         "withdraw":"Request withdrawal","motion":"Measure rack motion","stow":"Confirm stow","hold":"Missing camera frame"}

def build():
    all_events=list(csv.DictReader((HERE/"events.csv").open()))
    receipts=read(HERE/"match.json")
    contacts=read(HERE/"contact.json")
    data={"stacks":COORDS,"modes":{},"reefBalls":[]}
    plans=read(ROOT/"fixed-height-match/placements.json")["placements"]
    for mode in ("nominal","lateB","single"):
        f,auto,team,delivered=fieldcase(mode,contacts,all_events)
        leg=receipts["cases"][mode]
        rows=[r for r in all_events if r["scenario"]==mode]
        timeline=[{"t":0,"name":"Start on the tape","detail":"All six robots occupy their registered starting positions."},
                  {"t":auto.duration,"name":"Leave complete","detail":"Red 2 has reached the station. Its full modeled hull clears the starting line."}]
        for r in rows:
            timeline.append({"t":int(r["wall_ms"])/1000,"pipe":r["piece"],"event":r["event"],
                             "name":ACTIONS[r["event"]],"detail":r["witness"]})
        pipe_positions={}
        for p in plans:
            k=p["id"]
            if k not in contacts[mode]:continue
            state=contacts[mode][k]
            pipe_positions[k]=[(f.LIP[0]+state["finalXZ"][0])/f.IN,(f.LIP[1]+p["sector"])/f.IN]
            if state["admitted"]:
                timeline.append({"t":state["lastObservedWallS"]-state["stillPairedAtEndS"]+2,"pipe":k,"event":"supported","name":"Confirm trough support",
                                 "detail":"The contact replay sustains floor and lip support, with the pipe clear of the robot."})
            else:
                timeline.append({"t":state["lastObservedWallS"],"pipe":k,"event":"recontact","name":"Shelf touches pipe",
                                 "detail":"The shelf remained extended. This pipe receives no scoring credit."})
        if leg["park"]:
            timeline.append({"t":18+leg["finishTeleopS"],"name":"Park in the barge zone","detail":"Red 2 stops at X = 372 in with its front bumper in the zone."})
        timeline.append({"t":153,"name":"Match clock expires","detail":"All powered motion ends. The assessment clock continues for three seconds."})
        timeline.append({"t":156,"name":"Assess the field","detail":f"{leg['runtimeDeliveries']} supported L1 placement(s); modeled total {leg['conditionalPoints']} points."})
        timeline.sort(key=lambda x:x["t"])
        frames=[]
        for i in range(625):
            wall=i/4;frame=[]
            for role,prog in team.items():
                if wall<18:pose=(auto.at(min(wall,auto.duration))[0] if role=="red2" else prog.initial)
                else:pose=prog.at(min(135,wall-18))[0]
                frame.extend([round(pose[0]/f.IN,2),round(pose[1]/f.IN,2),round(math.degrees(pose[2]),1)])
            frames.append(frame)
        data["modes"][mode]={"pts":leg["conditionalPoints"],"finish":leg["finishTeleopS"],"park":leg["park"],"count":leg["runtimeDeliveries"],
             "gap":round(1000*leg["pairBound"][0],1),"timeline":timeline,"bots":list(team),"frames":frames,"pipeXY":pipe_positions}
    data["solids"]=[[k,[[round(x/f.IN,2),round(y/f.IN,2)] for x,y in pts]] for k,pts in f.obstacles()]
    stock=read(ROOT/"full-match/installed-stock.json")
    for ident,p in stock["stock"].items():
        if p["group"]=="reef-ball":
            x,h,z=p["native_center_mm"];xx,yy=f.field_from_native(x/25.4,z/25.4)
            data["reefBalls"].append([ident,xx/f.IN,yy/f.IN,h,stock["ball_radius_mm"]/25.4])
    return (HERE/"viewer.template.html").read_text(encoding="utf-8").replace("__STUDY__",json.dumps(data,separators=(",",":"),allow_nan=False))


if __name__=="__main__":
    (HERE/"replay.html").write_text(build(),encoding="utf-8",newline="\n")
