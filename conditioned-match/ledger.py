"""Join the observed station, admitted cargo, field, and deferred inventory on field wall time."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent

def read(path):
    return json.loads(path.read_bytes())

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def build():
    opening=read(HERE/"opening.json")
    feeds=read(HERE/"feeds.json")
    seat=read(HERE/"seat-images.json")
    roof=read(HERE/"roof.json")
    field=read(HERE/"occupancy.json")
    nav=read(HERE/"navigation.json")
    coupled=read(HERE/"elastic.json")["cases"]
    cargo=read(HERE/"corrected-cargo.json")
    placements=read(ROOT/"fixed-height-match/placements.json")["placements"]
    java_origin=18+placements[0]["lip"]-.100
    with (ROOT/"relieved-match/events.csv").open() as f:
        archived=[{"piece":r["piece"],"event":r["event"],"wallS":int(r["wall_ms"])/1000} for r in csv.DictReader(f) if r["scenario"]=="nominal"]
    cases=[]
    for side,approach in opening.items():
        if approach["status"]!="observed stationary" or approach["wallS"]>=21.1 or approach["power"]["screened"]["firstReferenceInhibitWallS"] is not None:
            raise ValueError("Revised feed lacks the registered, stopped and powered chassis")
        for mode in ("effective","lower-damping","elastic-stock","elastic-insert","elastic-damped"):
            if mode in ("effective","lower-damping"):
                dt=.00003125 if mode=="effective" else .0000625
                d=4 if mode=="effective" else 1
                r=next(x for x in feeds if x["case"]==side and x["dtS"]==dt and x["shelf"]["dissipation_s_m"]==d)
                local=r["lastTickMs"];state=r["state"];pipe_sha=r["trajectorySha256"];hard=r["stop"];loss=r["loss"]
            else:
                matches=[x for x in coupled if x["case"]==side and (
                    (mode=="elastic-stock" and x["hardware"]=="stock" and x["stowDamperSlopeNsPerM"]==0) or
                    (mode=="elastic-insert" and x["hardware"]=="insert" and x["stowDamperSlopeNsPerM"]==0) or
                    (mode=="elastic-damped" and x["stowDamperSlopeNsPerM"]>0))]
                if not matches:continue
                r=matches[0];local=r["lastJavaMs"];state=r["lastState"];pipe_sha=r["trajectorySha256"];hard=r["hardwareStop"];loss=r["loss"]
            events=[(0.,"start","Both named pipes are in station stock."),(15.,"leave","The entire bumper clears the starting tape."),
                    (approach["wallS"],"dock","The fixed stereo confirms the stationary lower-panel pose."),
                    (21.10,"feed A","The human feeds coral-A. Coral-B remains reserved at the station.")]
            permitted=False
            if state=="IN_BAND" and mode=="effective":
                if seat[side]["trajectorySha256"]!=pipe_sha or roof[side]["sourceSha256"]!=pipe_sha:raise ValueError("Changed pipe identity at optical handoff")
                proven=all(x["containsSimulatedCenter"] for x in seat[side]["frames"]) and not roof[side]["firstFailedJavaMs"]
                events.append((java_origin+r["firstBandMs"]/1000,"ports A","The same tube enters the dual-port seat state."))
                permitted=proven and roof[side]["lastUsableJavaMs"] is not None
                events.append((java_origin+5.455,"optical custody","Stereo and roof coverage agree." if permitted else "At least one camera gate withholds travel."))
            if permitted:
                t=18+placements[0]["drive"]
                events.append((t,"conditional drive","The supplied pipe state continues under the effective contact law."))
                group=[x for x in cargo if x["wheelCase"]==side and x["mode"]=="corrected"]
                if not group or any(x.get("feedTrajectorySha256")!=pipe_sha for x in group):raise ValueError("Unregistered cargo propagation")
                for x in group:
                    if x.get("loss") or x.get("stop"):
                        events.append((x["endWallS"],"cargo limit",f'{x["dtS"]*1e6:g} microsecond cargo continuation reaches a physical model limit.'))
                stop=field[side+"-corrected"]["lastWheelAtWallS"]
                events.append((stop,"reef hold","The field camera never confirms the scoring approach. Rack remains stowed."))
            else:
                reason=loss or ("cartridge stop" if hard else r.get("fault") or "independent custody unavailable")
                events.append((java_origin+local/1000,"station hold",str(reason)))
            events.extend([(next(x["wallS"] for x in archived if x["piece"]=="coral-A" and x["event"]=="release"),"placement A withheld","No verified placement or rack withdrawal."),
                (18+placements[1]["release"],"feed B withheld","No certified return from sector A. Coral-B stays in station stock."),
                (156.,"assessment","Zero confirmed pipe placements. No late park credit.")])
            cases.append({"wheelCase":side,"material":mode,"sourceBasis":"software","pipeA":pipe_sha,"portState":state,"released":[],"secondPipe":"coral-B stays at station",
                "location":("conditioned field stop" if permitted else "station hold"),"detectionOnlyDrive":permitted,
                "conditionalLeavePoints":3,"placementPoints":0,"parkPoints":0,"hardwareQualified":False,
                "events":[{"wallS":v[0],"action":v[1],"evidence":v[2]} for v in sorted(events,key=lambda x:x[0])]})
    return {"clock":"field wall seconds; TELEOP starts at 18, final assessment 156","archivedTwoTripSchedule":archived,"cases":cases,
            "preselectedSouth":{"leave":3,"park":2,"placement":0,"scope":"source bounded pre-match alternative","sourceSha256":digest(ROOT/"station-approach/results.json")}}

if __name__=="__main__":
    (HERE/"match.json").write_text(json.dumps(build(),indent=2)+"\n",encoding="utf8",newline="\n")
