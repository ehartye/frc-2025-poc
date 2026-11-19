"""Build the standalone student replay from the registered low-carriage schedules."""
import json
import math
from pathlib import Path
from rehearse import HERE, ROOT, read, scenario
from match import COORDS, heavy


def main():
    sources=read(HERE/"placements.json")["placements"]
    results=read(HERE/"match.json")
    modes=(("Issued interlock holds at station","stationHold",{"station_hold":True}),
           ("Reserved lane, assumed withdrawal","reserved",{}),("Partner keeps the lane","blocked",{"partner_clears":False}),
           ("Second pipe arrives late","lateSecond",{"stock":(0,112)}),
           ("Return to station","returnInstead",{"park":False}))
    scenarios={}
    for label,key,options in modes:
        f,auto,team,deliveries=scenario(sources,**options)
        r=results[key]
        frames=[]
        for i in range(313):
            wall=i/2
            bots=[]
            for role,p in team.items():
                if wall<18:
                    pose=auto.at(min(wall,auto.duration))[0] if role=="red2" else p.initial
                    action="AUTO station approach" if wall<auto.duration else "Hold at station"
                else:
                    pose,_,action=p.at(min(135,wall-18))
                if role=="red2":
                    focal_action=action if wall<153 else "Post-buzzer scoring assessment"
                bots.append([round(pose[0]/f.IN,2),round(pose[1]/f.IN,2),round(math.degrees(pose[2]),2)])
            frames.append([bots,focal_action])
        note=(f"First body overlap at TELEOP {r['firstOverlap'][2]:.2f} s. The endgame route is blocked."
              if r["firstOverlap"] else
              f"{len(deliveries)} modeled deliveries. Red 2 finishes at TELEOP {r['finishTeleopS']:.2f} s. "
              f"Conditional opportunity: {r['conditionalPoints']} points. The issued observer authorizes zero L1 deliveries.")
        scenarios[label]={"frames":frames,"bots":list(team),"result":{
            "qualified_supports":[{"support_s":s["supported"],"sector_m":s["sector"]} for s in deliveries],
            "message":note}}
    data={"scenarios":scenarios,"stacks":COORDS,"solids":[[k,[[round(x/f.IN,2),round(y/f.IN,2)] for x,y in pts]]
          for k,pts in f.obstacles()],"target":[z/f.IN for z in f.LIP],"reefBalls":[],"hit":COORDS[-1]}
    original=heavy.revised_motion().routes(-.18)[0]
    data["legacy"]=[[round(v/f.IN,2) for v in original.at(i/50)[0][:2]] for i in range(int(original.duration*50)+1)]
    field=read(ROOT/"full-match/installed-stock.json")
    for name,obj in field["stock"].items():
        if obj["group"]=="reef-ball":
            x,h,z=obj["native_center_mm"]
            xx,yy=f.field_from_native(x/25.4,z/25.4)
            data["reefBalls"].append([name,round(xx/f.IN,3),round(yy/f.IN,3),round(h),field["ball_radius_mm"]/25.4])
    text=(ROOT/"full-match/replay-template.html").read_text(encoding="utf-8")
    start=text.index(" const hit=s.result.first_pair;")
    stop=text.index("\n}",start)
    text=text[:start]+' document.querySelector("#note").textContent=s.result.message;'+text[stop:]
    text=text.replace("Two pipes and a park","Fixed carriage, two station feeds").replace("153","156")
    text=text.replace('t<18?"DISABLED":"TELEOP"', 't<18?"DISABLED":t<153?"TELEOP":"ASSESSMENT"')
    text=text.replace("t<18?t-15:t-18","t<18?t-15:t<153?t-18:t-153")
    text=text.replace("Field study and assumptions","Low-carriage study and assumptions")
    return text.replace("__DATA__",json.dumps(data,separators=(",",":")))


if __name__=="__main__":
    (HERE/"replay.html").write_text(main(),encoding="utf-8",newline="\n")
