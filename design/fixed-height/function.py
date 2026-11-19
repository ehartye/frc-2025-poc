"""Compare issued source/target phases with a fixed carriage. Software hypotheses only."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from contextlib import ExitStack
from unittest.mock import patch

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/"axial-spin"))
import pipe_spin_route as spin
sys.path.insert(0,str(ROOT/"coupled-belt-clock"))
from replay import HeldContact,Trace
from spin_cases import CANDIDATE
from scripts import receiver_contact as station


def trial(height,locked,dt=.0005,low_feed=False,id_='coral-A',profile=None,held_intake=False):
    source=next(entry for entry in spin.DATA["identified_transfers"] if entry["id"]==id_)
    field=spin.corrected()
    issued=field.RegisteredMission
    class FixedRoute(issued):
        def pose(self,t):
            name,x,y,yaw,_,stroke=super().pose(t)
            return name,x,y,yaw,height/1000-.060,0 if locked or (low_feed and t<self.drive_end) else stroke

    stroke=0 if locked or low_feed else .240
    state=station.departure(spin.ramp(spin.DATA["feed"],CANDIDATE.ramp_mu)["speed"])
    ideal_time=station.time_at_center_height(state,height/1000-.060+spin.R)
    ideal_x,_=station.flight_center(state,ideal_time)
    u=.444525-stroke-ideal_x
    with patch.object(spin,"SHELF",height/1000-.060+spin.R),patch.object(spin,"U_ORIGIN",.444525-stroke), \
         patch.object(spin,"corrected",lambda:SimpleNamespace(RegisteredMission=FixedRoute)):
        with ExitStack() as patches:
            if profile:
                clock=Trace(profile)
                patches.enter_context(patch.object(spin,"AxialContact",lambda cfg,omega: HeldContact(cfg,omega,clock,acquisition=held_intake)))
            result,run,_=spin.trial(CANDIDATE,source,dt)
    pick=result["pickup"]
    case={"heightMm":height,"strokeLockedRetracted":locked,"sourceId":source["id"],"retractedStationActivePlace":low_feed,
          "commandProfile":profile or "proportional-model","useLoggedIntake":held_intake,"dt":dt,"freeFlightPlaneTimeS":ideal_time,"freeFlightShelfLocalMm":1000*u,
          "flatShelfIntervalMm":[-100,159],"capture":pick["status"],
          "contactOrMissLocalMm":1000*(pick.get("u") if "u" in pick else pick.get("impact",{}).get("local_u_m",0)),
          "outcome":result["status"],"assessment":None}
    if run:
        case["firstContact"]=run["outcome"]
        case["lastTargetPose"]=run["trace"][-1]["target_xyz_m"]
        case["lastTargetVelocityXY"]=run["trace"][-1]["target_vxvy_mps"]
        if result.get("target"):
            case["assessment"]=result["target"]
    return case


def main():
    rows=[trial(height,locked) for height in (560,670,940) for locked in (True,False)]
    rows.extend(trial(560,False,low_feed=True,id_=key) for key in ("coral-A","coral-B"))
    for profile in ("HISTORICAL","RAMP_090"):
        rows.append(trial(560,False,low_feed=True,profile=profile))
        rows.append(trial(560,False,low_feed=True,profile=profile,held_intake=True))
    (HERE/"function.json").write_text(json.dumps(rows,indent=2)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__":
    main()
