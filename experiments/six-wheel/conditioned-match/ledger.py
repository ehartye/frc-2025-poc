"""Join the observed station, admitted cargo, field, and deferred inventory on field wall time."""
import math
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
    machined=read(HERE/"machined-images.json")
    coupled=read(HERE/"elastic.json")["cases"]
    cargo=read(HERE/"slow-cargo.json")
    carry_eye=read(HERE/"slow-roof.json")
    electrical=read(HERE/"slow-power.json")
    placements=read(ROOT/"fixed-height-match/placements.json")["placements"]
    java_origin=18+placements[0]["lip"]-.100
    with (ROOT/"relieved-match/events.csv").open() as f:
        archived=[{"piece":r["piece"],"event":r["event"],"wallS":int(r["wall_ms"])/1000} for r in csv.DictReader(f) if r["scenario"]=="nominal"]
    scheduled={(row["piece"],row["event"]):row["wallS"] for row in archived}
    cases=[]
    for side,approach in opening.items():
        if approach["status"]!="observed stationary" or approach["wallS"]>=21.1 or approach["power"]["screened"]["firstReferenceInhibitWallS"] is not None:
            raise ValueError("Revised feed lacks the registered, stopped and powered chassis")
        for mode in ("effective","lower-damping","elastic-stock","elastic-insert","elastic-damped","machined-damped"):
            if mode in ("effective","lower-damping"):
                dt=.00003125 if mode=="effective" else .0000625
                d=4 if mode=="effective" else 1
                r=next(x for x in feeds if x["case"]==side and x["dtS"]==dt and x["shelf"]["dissipation_s_m"]==d)
                local=r["lastTickMs"];state=r["state"];pipe_sha=r["trajectorySha256"];hard=r["stop"];loss=r["loss"]
            else:
                matches=[x for x in coupled if x["case"]==side and (
                    (mode=="elastic-stock" and x["hardware"]=="stock" and x["stowDamperSlopeNsPerM"]==0) or
                    (mode=="elastic-insert" and x["hardware"]=="insert" and x["stowDamperSlopeNsPerM"]==0) or
                    (mode=="elastic-damped" and x["stowDamperSlopeNsPerM"]>0 and "machined" not in x["id"] and abs(x["runningGapMm"]-.10)<1e-8) or
                    (mode=="machined-damped" and "machined" in x["id"] and abs(x["stepS"]-.00003125)<1e-10))]
                if not matches:continue
                r=matches[0];local=r["lastJavaMs"];state=r["lastState"];pipe_sha=r["trajectorySha256"];hard=r["hardwareStop"];loss=r["loss"]
            events=[(0.,"start","Both named pipes are in station stock."),(15.,"leave","The entire bumper clears the starting tape."),
                    (approach["wallS"],"dock","The fixed stereo confirms the stationary lower-panel pose."),
                    (21.10,"feed A","The human feeds coral-A. Coral-B remains reserved at the station.")]
            permitted=False
            valid_through=False
            if state=="IN_BAND" and mode=="effective":
                if seat[side]["trajectorySha256"]!=pipe_sha or roof[side]["sourceSha256"]!=pipe_sha:raise ValueError("Changed pipe identity at optical handoff")
                proven=all(x["containsSimulatedCenter"] for x in seat[side]["frames"]) and not roof[side]["firstFailedJavaMs"]
                events.append((java_origin+r["firstBandMs"]/1000,"ports A","The same tube enters the dual-port seat state."))
                permitted=proven and roof[side]["lastUsableJavaMs"] is not None
                events.append((java_origin+5.455,"optical custody","Stereo and roof coverage agree." if permitted else "At least one camera gate withholds travel."))
            if state=="IN_BAND" and mode=="machined-damped":
                image=machined[side]
                if image["trajectorySha256"]!=pipe_sha:
                    raise ValueError("Changed machined pipe at optical handoff")
                events.append((java_origin+r["firstBandJavaMs"]/1000,"ports A","The two shelf ports report a seat on the machined contact model."))
                for finish,trial in image["finishes"].items():
                    witnessed=all(f["containsSimulatedCenter"] and f["status"]=="position-witness" for f in trial["frames"])
                    text="Fixed stereo encloses the nominal pipe center." if witnessed else "Fixed stereo does not establish independent custody."
                    events.append((java_origin+5.455,"fixed stereo: "+finish,text))
            if permitted:
                t=18+placements[0]["drive"]
                events.append((t,"conditional drive","The supplied pipe state continues under the effective contact law."))
                group=[x for x in cargo if x["wheelCase"]==side and x["mode"]=="slow"]
                if not group or any(x.get("feedTrajectorySha256")!=pipe_sha for x in group):raise ValueError("Unregistered cargo propagation")
                if electrical[side]["screened"]["firstReferenceInhibitWallS"] is not None:
                    raise ValueError("Electrical inhibit on the selected slow transit")
                eye=carry_eye[side]
                high=next(x for x in group if x["dtS"]==.0000625)
                if eye["sourceSha256"]!=high["relativePipeSha256"]:raise ValueError("Carry exposure used a different pipe")
                if eye["firstFailedJavaMs"]:
                    events.append((java_origin+5*math.ceil((eye["firstFailedJavaMs"][0]+15)/5)/1000,"cargo view lost","The first missing roof witness ends the demonstrated continuous custody. The subsequent chassis and pipe positions are unverified."))
                for x in group:
                    if x.get("loss") or x.get("stop"):
                        events.append((x["endWallS"],"cargo limit",f'{x["dtS"]*1e6:g} microsecond cargo continuation reaches a physical model limit.'))
                stop=field[side+"-slow"]["lastWheelAtWallS"]
                valid_through=not eye["firstFailedJavaMs"] and not any(x.get("loss") or x.get("stop") for x in group)
                events.append((stop,"reef hold" if valid_through else "projected reef stop",
                    "The field camera never confirms the scoring approach. Rack remains stowed." if valid_through else
                    "Wheel-only projection ends without sector registration. Earlier cargo limits withhold credit for this position."))
            else:
                reason=loss or ("cartridge stop" if hard else r.get("fault") or "independent custody unavailable")
                if mode=="machined-damped" and state=="IN_BAND":
                    reason="Independent moving-roof custody and loaded continuation on the machined elastic shelf remain unverified."
                events.append((java_origin+local/1000,"station hold",str(reason)))
            events.extend([(scheduled["coral-A","release"],"placement A withheld","No verified placement or rack withdrawal."),
                (scheduled["coral-A","stow"],"return withheld","The rack never extended. The controller has no confirmed departure from an A placement."),
                (scheduled["coral-B","feed"],"feed B withheld","No certified return from sector A. Coral-B stays in station stock."),
                (scheduled["coral-B","release"],"placement B withheld","The second pipe remains at the station."),
                (156.,"assessment","Zero confirmed pipe placements. No late park credit.")])
            cases.append({"wheelCase":side,"material":mode,"sourceBasis":"software","pipeA":pipe_sha,"portState":state,"released":[],"secondPipe":"coral-B stays at station",
                "location":("conditional reef stop" if permitted and valid_through else "loaded path interrupted" if permitted else "station hold"),"detectionOnlyDrive":permitted,
                "conditionalLeavePoints":3,"placementPoints":0,"parkPoints":0,"hardwareQualified":False,
                "events":[{"wallS":v[0],"action":v[1],"evidence":v[2]} for v in sorted(events,key=lambda x:x[0])]})
    return {"clock":"field wall seconds; TELEOP starts at 18, final assessment 156","archivedTwoTripSchedule":archived,"cases":cases,
            "preselectedSouth":{"leave":3,"park":2,"placement":0,"scope":"source bounded pre-match alternative","sourceSha256":digest(ROOT/"station-approach/results.json")}}

if __name__=="__main__":
    (HERE/"match.json").write_text(json.dumps(build(),indent=2)+"\n",encoding="utf8",newline="\n")
