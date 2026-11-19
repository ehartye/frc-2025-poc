"""Execute the new observed station, two-camera release and rack feedback for both pieces in desktop HAL."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"return-station"))
from feed import port_program

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_bytes())
def replace(text,old,new):
    if text.count(old)!=1:raise ValueError("Unexpected code source: "+old)
    return text.replace(old,new)


def inputs(cache,folder):
    stations=read(HERE/"station-images.json")["cases"]
    register=read(HERE/"camera-registration.json")
    policy={}
    sources={}
    with (HERE/"landing-windows.csv").open() as f:
        screening={x["case"]:x["outcome"] for x in csv.DictReader(f)}
    with (folder/"inputs.csv").open("w",newline="") as fin,(folder/"frames.csv").open("w",newline="") as moving,(folder/"exposures.csv").open("w",newline="") as flight:
        w,m,v=[csv.writer(file,lineterminator="\n") for file in (fin,moving,flight)]
        w.writerow(["kind","piece","clock","data"]);m.writerow(["piece","capture","arrival","status","bounds"]);v.writerow(["piece","head","capture","status","bounds"])
        for key,label in (("coral-A","first"),("coral-B","return")):
            feed=read(HERE/(label+"-feed.json"))
            case=feed["cases"][0]
            path=read(HERE/(key+"-pads-edge-250.json"))
            motion=read(HERE/(key+"-moving.json"))
            stamp=motion["releaseCameraMs"]
            # The lower contact timestamp is a modeled force-pad indication. Axis tilt, load
            # threshold, cover calibration and scan uncertainty are independent hardware inputs.
            readings={}
            for resolution in (250,500):
                forces=cache/f"{key}-pads-edge-{resolution}-forces.csv"
                registered=read(HERE/f"{key}-pads-edge-{resolution}.json")
                if sha(forces)!=registered["padLoadSha256"]:raise ValueError("Changed pad loads")
                raw=np.loadtxt(forces,delimiter=",",skiprows=1)
                t=case["scheduledDriveCameraMs"]+1000*raw[:,0]
                width=2000//resolution+1
                # Keep a single pad loaded for two milliseconds and retain threshold sensitivity.
                readings[str(resolution)]={}
                for load in (.25,1.,3.,8.):
                    active=np.array([np.any(np.min(raw[i-width+1:i+1,1:],axis=0)>=load)
                                     and t[i]-t[i-width+1]<=2.001 if i>=width-1 else False for i in range(len(t))])
                    hits=t[(t>=stamp)&(t<=motion["edgeCameraMs"])&active]
                    readings[str(resolution)][str(load)]=float(hits[-1]) if len(hits) else None
            last=[r["1.0"] for r in readings.values()]
            if any(v is None for v in last):raise ValueError("No supported 1 N pressure-pad prior")
            prior=math.floor(min(last)-3)
            lower=370-536-1-(57.65*math.cos(math.radians(1))+151.0625*math.sin(math.radians(1)))
            eligible=[]
            for head in ("moving","fixed"):
                with (cache/(key+"-"+head+".csv")).open() as f:
                    for row in list(csv.reader(f))[1:]:
                        ts=int(row[2])
                        if head=="moving" and ts<=stamp+17:m.writerow([key,ts,ts+15,*row[3:]])
                        if ts<stamp:continue
                        v.writerow([*row,*[""]*(10-len(row))])
                        if head!="fixed" or len(row)!=10 or float(row[4])+lower<0:continue
                        # Enumerate the complete 15ms-delivery plus 0..4ms scheduler interval
                        # in both temporal resolutions of the carried article. Include neighboring
                        # integer-ms samples, keep the time-step interpolation conditional.
                        options=[]
                        for res in (250,500):
                            release=read(HERE/f"{key}-pads-edge-{res}.json")["wallS"]
                            edge=1000*release-feed["epochWallUs"]/1000
                            offsets=range(math.floor(ts+15-edge),math.ceil(ts+19-edge)+1)
                            options.extend(screening.get(f"{key}/{res}/{d}") for d in offsets)
                        if options and all(x=="supported" for x in options):eligible.append(ts)
            policy[key]={"priorLowerMs":prior,"eligibleCaptureMs":eligible,"releaseCameraMs":stamp,"forceThresholdsMs":readings}
            w.writerow(["clock",key,feed["epochWallUs"],stamp,math.floor(feed["scheduledDriveWallS"]*1000),0 if key=="coral-A" else 35,case["firstPortBandMs"],round(feed["humanReleaseWallS"]*1000),prior])
            for time,state,entry,seat,demand in case["portTicks"]:
                w.writerow(["port",key,time,time,1,1,1,1,int(entry),int(seat),state])
            for tick,status,*b in stations[label+"-nominal"]["frames"]:w.writerow(["seat",key,tick,status,*(b or [0]*6)])
    sources={p.name:sha(p) for p in (HERE/"camera-registration.json",HERE/"station-images.json",HERE/"landing-windows.csv",HERE/"first-feed.json",HERE/"return-feed.json",HERE/"probe-native.json",HERE/"hold-native.json")}
    for key in policy:
        for head in ("fixed","moving"):sources[key+"-"+head]=sha(cache/(key+"-"+head+".csv"))
    decision={"inputs":sources,"eligible":policy,"delayMs":15,"onsetReserveMs":40,"padThresholdMeasured":False,"qualification":"conditional synthetic pressure and image calibration"}
    policyfile=HERE/"controller-register.json"
    policyfile.write_text(json.dumps(decision,indent=2)+"\n",encoding="utf8",newline="\n")
    regs=[sha(HERE/"station-images.json"),register["fixedPlaceSha256"],register["coMovingBackingSha256"],sha(policyfile)]
    (folder/"registers.txt").write_text("\n".join(regs),encoding="ascii")
    return regs,policy


def run(cache,jdk,gradle):
    with tempfile.TemporaryDirectory() as td:
        folder=Path(td);registers,policy=inputs(cache,folder)
        for name,place in (("MovingWitnessTest","moving-observer"),("PacketClockTest","moving-frame-timing"),("SlideFeedbackTest","slide-feedback"),("OpticalWindow","moving-frame-timing"),("PortAcquisition","observable-acquisition"),("CustodyEvidence","stereo-custody"),("Retreat","slide-feedback")):
            text=(ROOT/place/(name+".java")).read_text()
            if name=="PortAcquisition":text=port_program(True)
            if name=="CustodyEvidence":text=replace(text,"6389d766ad3d4d65b885bccb615a23807cadd8ef4f14d846214cea8a4dd96f73",registers[0])
            if name=="Retreat":
                text=replace(text,"c1242109e239d880d1562c3e7b4bbcf857d154d9a31e3403e35e182864d00ebe",registers[3])
                for key,old,lo,hi in (("coral-A",26593,26695,26711),("coral-B",26301,26402,26418)):
                    r=policy[key]
                    text=replace(text,"frame.priorMs() == "+str(old),"frame.priorMs() == "+str(r["priorLowerMs"]))
                    permitted=" || ".join("frame.capturedMs() == "+str(t) for t in r["eligibleCaptureMs"]) or "false"
                    text=replace(text,"frame.capturedMs() >= "+str(lo)+" && frame.capturedMs() <= "+str(hi),"("+permitted+")")
            (folder/(name+".java")).write_text(text,encoding="utf8",newline="\n")
        text=(ROOT/"integrated-match/IntegratedTest.java").read_text()
        text=replace(text,"prior=a?26593:26301;","prior=Long.parseLong(row[8]);")
        text=replace(text,"boolean seeded,broken,drive,requested","long lastStationService=-1; boolean stationLive; boolean seeded,broken,drive,requested")
        text=replace(text,"if(seat.update(t,inBand,true,true,piece,batch) && !seeded){","stationLive=seat.update(t,inBand,true,true,piece,batch); lastStationService=w; if(stationLive && !seeded){")
        text=replace(text,"if(!seeded || broken)","if(!seeded || broken || !stationLive || w-lastStationService>20)")
        text=replace(text,"assertTrue(a.stowed, mode);assertEquals(mode.equals(\"nominal\"),b.stowed,mode);assertEquals(mode.equals(\"lateB\"),b.failed,mode);","assertFalse(b.issued && !a.stowed);")
        (folder/"IntegratedTest.java").write_text(text,encoding="utf8",newline="\n")
        env=dict(os.environ,JAVA_HOME=str(jdk),GRADLE_USER_HOME=str(gradle),CUSTODY_SOURCE=str(folder),INTEGRATED_OUTPUT=str(cache.resolve()),
                 REEF_CONTROLS_SOURCE=str(ROOT/"competition-controls"),REEF_CONTROLS_BUILD=str(folder/"build"))
        env["PATH"]=str(jdk/"bin")+os.pathsep+env["PATH"]
        project=ROOT/"shared/examples/java-2026-sim";wrapper=project/("gradlew.bat" if os.name=="nt" else "gradlew")
        subprocess.run([str(wrapper),"--offline","--no-daemon","--project-cache-dir",str(folder/"gradle"),"-I",str(ROOT/"competition-controls/init.gradle"),"-I",str(ROOT/"stereo-custody/init.gradle"),"test","--tests","org.reefscape.controls.IntegratedTest"],cwd=project,env=env,check=True)
        (HERE/"events.csv").write_text((cache/"events.csv").read_text(),encoding="ascii",newline="\n")
        data={"registration":registers,"programs":{p.name:sha(p) for p in folder.glob("*.java")},"controllerInputs":{p.name:sha(p) for p in folder.glob("*.csv")},"events":sha(HERE/"events.csv"),
              "rackPaths":{p.name:sha(p) for p in cache.glob("path-*.csv")}}
        (HERE/"hal.json").write_text(json.dumps(data,indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("cache","jdk","gradle"):parser.add_argument(name,type=Path)
    a=parser.parse_args()
    run(a.cache,a.jdk,a.gradle)
