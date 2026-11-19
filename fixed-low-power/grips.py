"""Record motor-demand envelopes on the issued controller and finite-pipe contact clocks."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from unittest.mock import patch

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"design/fixed-height"))
import handoff
import ports
from size_drives import motor, R_GRIP, UP5


def record(jdk):
    deliveries=json.loads((ROOT/"fixed-height-match/placements.json").read_bytes())["placements"]
    chosen={(0,.0005):deliveries[0],(19,.00025):deliveries[1]}
    ticks={}
    def add(time,stage,side,speed,acc,load):
        tick=math.floor(time/.020+1e-9)
        row=ticks.setdefault(tick,{"stage":stage,"L":[0.,0.],"R":[0.,0.]})
        if max(abs(speed),abs(acc),abs(load))<1e-8:
            return
        effort=motor(abs(load)+.4,abs(speed),R_GRIP,UP5,.80,abs(acc),3e-6,8)
        prior=row[side]
        prior[0]=max(prior[0],effort["phase_A"])
        prior[1]=max(prior[1],effort["terminal_V"])
    original=handoff.HeldContact.forces
    def sampled(offset,stage):
        def capture(law,speed,vertical,normals,requests,limit,dt,required):
            prev=law.belt_speed.copy()
            out=original(law,speed,vertical,normals,requests,limit,dt,required)
            now=law.previous[0]+offset
            for i,key in enumerate(("L","R")):
                add(now,stage if stage=="intake" else law.phase,key,law.belt_speed[i],(law.belt_speed[i]-prev[i])/dt,out[0][i])
            return out
        return capture
    java=ports.run
    def intake(proc,case,dt,events):
        entry=chosen.get((case.phase_ms,dt))
        if not entry:
            return {}
        offset=entry["lip"]-(100+case.phase_ms)/1000
        with patch.object(handoff.HeldContact,"forces",sampled(offset,"intake")):
            outcome=java(proc,case,dt,events)
        speed=0.
        for e in events:
            ms=e[2]; target=float(e[-2])
            if ms>=100+case.phase_ms:
                break
            interval=min(.020,(100+case.phase_ms-ms)/1000)
            change=max(-handoff.CANDIDATE.belt_accel*interval,min(handoff.CANDIDATE.belt_accel*interval,target-speed))
            for key in ("L","R"):
                add(offset+ms/1000,"unloaded-counterfeed",key,speed+change,change/interval,0.)
            speed+=change
        return outcome
    with patch.object(ports,"run",intake):
        actual=ports.main(jdk,.65)
    frozen=json.loads((ROOT/"design/fixed-height/ports-065.json").read_bytes())
    for (phase,dt),entry in chosen.items():
        key=f"{phase}ms-{dt:g}"
        attained=actual["cases"][key]
        if json.loads(json.dumps(attained))!=frozen["cases"][key]:
            raise ValueError("Controller or input differs from the registered feed")
        source=next(b for b in handoff.spin.DATA["identified_transfers"] if b["id"]==entry["id"])
        with patch.object(handoff.HeldContact,"forces",sampled(entry["seat"]-1.1,"route")):
            outcome=handoff.one(source,"RAMP_090",attained)
        if abs(outcome["after-separation"]["targetContactTeleopS"]-entry["impact"])>1e-7:
            raise ValueError("Placement time differs from the issued trial")
    return {"compiledJavaSha256":actual["compiledSourceSha256"],"periodS":.020,
            "ticks":ticks,"peakPerBelt":{side:max(row[side][0] for row in ticks.values()) for side in ("L","R")}}


if __name__=="__main__":
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument("jdk",type=Path)
    cli.add_argument("output",type=Path)
    args=cli.parse_args()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    result=record(args.jdk)
    args.output.write_text(json.dumps(result,separators=(",",":"),allow_nan=False)+"\n",encoding="utf-8",newline="\n")
    print(result["peakPerBelt"])
