"""Register the continuously held B state to the one-clock departure for a same-robot successor."""
import csv
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent


def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def run(cache):
    plan=read(ROOT/"fixed-height-match/placements.json")["placements"][1]
    epoch=round((18+plan["lip"])*1e6-119000)
    wall=18+plan["drive"]
    local=wall*1000-epoch/1000
    head=read(HERE/"custody.json")["hold"]["run"]
    features=read(HERE/"hold-feed.json");route=read(HERE/"return.json")
    comparisons=read(HERE/"coupled-clearance.json")["cases"]
    pads=read(HERE/"pads.json")["results"]
    results=[]
    for s in features:
        key=s["name"];mode=key[len("hold-bias-"):-len("-250")]
        pose=route["cases"][mode]; u=cache/(key+".npy")
        if sha(u)!=s["trajectorySHA256"]:raise ValueError("Changed trajectory")
        tr=np.load(u)
        if not tr[0,0]<local<tr[-1,0]:raise ValueError("B departure outside held observation")
        at=np.array([np.interp(local,tr[:,0],col) for col in tr[:,1:].T]);at[3:6]/=np.linalg.norm(at[3:6])
        t=next(r for r in head if r["case"]==key and r["challenge"]=="nominal")
        late=next(r for r in head if r["case"]==key and r["challenge"]=="late-at-departure")
        results.append({"approach":mode,"pipe":"coral-B","nativeBodies":597,"counterfeedMps":.65,"reverseProbeMps":-.14,"epochWallUs":epoch,
                        "stationChassisOffsetMmDeg":pose["stationRobotDxDyYawMmDeg"],"scheduledDriveWallS":wall,"scheduledDriveCameraMs":local,
                        "positionRobotMm":at[:3].tolist(),"axisRobot":at[3:6].tolist(),"velocityRobotMps":at[6:9].tolist(),"angularRobotRadps":at[9:12].tolist(),
                        "firstCustodyWallS":epoch/1e6+int(t["joined_ms"])/1000,"desktopDomainAtLastTick":t["ready_at_scheduled_tick"]=="true",
                        "expiredAtLastTick":late["ready_at_scheduled_tick"]!="true","competitionPermission":t["competition_drive"]=="true",
                        "minimumPadGapMm":min(r["continuousLowerMm"] for r in pads if key.replace("hold","probe") in r["case"]),
                        "minimumCheekGapAt025Mm":min(r["at025mmIndentationMm"] for r in comparisons if key.replace("hold","probe") in r["case"]),
                        "incomingStationPeriodMs":19,"rawStateSha256":s["trajectorySHA256"]})
    evidence=[HERE/n for n in ("return.json","relief-dock.json","probe-native.json","coupled-clearance.json","pads.json","hold-native.json","hold-feed.json","hold-images.json","custody.json","qualification.json")]
    evidence.extend([ROOT/"station-relief/cheeks.step",ROOT/"station-relief/shims.step",ROOT/"spatial-contact/match.json"])
    summary={"robot":"fixed-low H560 / red / X48 optics / zero station stroke / stop and cheek relief","cases":results,
             "integration":"A separate full mission must start, score the first pipe and return using this same assembled revision before a nine-point claim.",
             "pins":{p.relative_to(ROOT).as_posix():sha(p) for p in evidence}}
    (HERE/"handoff.json").write_text(json.dumps(summary,indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":run(Path(sys.argv[1]))
