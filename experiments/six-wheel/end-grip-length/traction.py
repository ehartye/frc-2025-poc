"""Use simultaneous modeled belt speed, acceleration and traction at each held-control plant step."""
import argparse
import json
from pathlib import Path
import sys
from unittest.mock import patch
from grip_study import ROOT,HERE,Trial,work,study
sys.path.insert(0,str(ROOT/"design/receiver-drives"))
from size_drives import motor,UP5,R_GRIP

def study_drive(jdk):
    force=(1.97+.32)*4.4482216152605
    cases=[Trial("issued",original=True),Trial("mixed-ideal",spring=force,cassette=True)]
    cases += [Trial(n,cut=cut,spring=force,cassette=True,indexed_center=cut/2,pad=.5,open_until_ms=0,friction=mu,
                    side=side,yaw=angle) for n,cut,mu,side,angle in (
        ("short-matched",-6.35,.4,0,0),("long-matched",6.35,.4,0,0),("weak",0,.08,0,0),
        ("long-misaligned",6.35,.4,-1,-.25))]
    findings={}
    original=study.DirectBelt.forces
    for c in cases:
        peaks={j:{"phaseA":0.,"terminalV":0.,"busA":0.} for j in (1e-6,3e-6,7e-6)}
        maximum=[0.,0.]
        def observe(self,speed,vertical,normals,requests,limit,dt,normal):
            before=list(self.belt_speed)
            result=original(self,speed,vertical,normals,requests,limit,dt,normal)
            for side,grip in enumerate(result[0]):
                belt=self.belt_speed[side]
                accel=abs(belt-before[side])/dt
                maximum[side]=max(maximum[side],abs(grip))
                for j,peak in peaks.items():
                    electrical=motor(abs(grip)+.4,abs(belt),R_GRIP,UP5,.80,accel,j,8)
                    peak["phaseA"]=max(peak["phaseA"],electrical["phase_A"])
                    peak["terminalV"]=max(peak["terminalV"],electrical["terminal_V"])
                    peak["busA"]=max(peak["busA"],electrical["bus_A_at_9p5V"])
            return result
        with patch.object(study.DirectBelt,"forces",observe):
            result=work(jdk,[c])[0]
        findings[c.name]={"outcome":result["optical"]["outcome"],"peakBeltN":maximum,"perMotor":peaks}
    return findings
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--jdk",type=Path,required=True);p.add_argument("--output",type=Path,required=True)
    a=p.parse_args();a.output.write_text(json.dumps(study_drive(a.jdk),indent=2)+"\n",encoding="utf-8",newline="\n")
