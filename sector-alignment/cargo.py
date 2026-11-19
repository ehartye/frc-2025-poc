"""Continue each attained pipe through the measured chassis path on the issued game clock."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"design/fixed-height"))
import handoff

def interpolate(samples,t):
    scaled=max(0.,min(t*200,len(samples)-1.));i=int(scaled);u=scaled-i
    if i==len(samples)-1:return samples[-1]
    p0,p1=samples[i],samples[i+1]
    m0=(samples[min(len(samples)-1,i+1)]-samples[max(0,i-1)])/2
    m1=(samples[min(len(samples)-1,i+2)]-samples[i])/2
    return (2*u**3-3*u*u+1)*p0+(u**3-2*u*u+u)*m0+(-2*u**3+3*u*u)*p1+(u**3-u*u)*m1

def case(row,cache,park=False):
    spin=handoff.spin;f=spin.corrected()
    pipe=row["pipe"];sector=spin.SECTORS[pipe];path=cache/(pipe+"-"+row["mode"]+".csv")
    if hashlib.sha256(path.read_bytes()).hexdigest()!=row["motionSHA256"]:raise ValueError("unregistered wheel path")
    samples=np.loadtxt(path,delimiter=",",skiprows=1)[:,1:]
    park_at=1.772158270517589+1.5
    destination=json.loads((ROOT/"leave-park/near-tape.json").read_bytes())["cases"]["near-tape"]["finish"]
    nominal=f.Route((f.LIP[0]-.490,f.LIP[1]+sector,0.))
    nominal.segment("drive",425*f.IN-nominal.start[0])
    nominal.go((425*f.IN,destination[1]));nominal.go(tuple(destination[:2]))
    at_park=f.Route((f.LIP[0]-.490+samples[-1,0]/1000,f.LIP[1]+samples[-1,1]/1000,samples[-1,2]))
    for seg in nominal.segments:at_park.segment(seg["kind"],seg["demand"])
    class Measured(f.RegisteredMission):
        def pose(self,t):
            phase,x,y,h,_,stroke=super().pose(t)
            at=t-(self.drive_end-self.carry.segments[-1]["dt"])
            if at<0:
                return phase,x,y,h,.5,0
            # Nominal registered arrival connects continuously to the upstream drive and turn.
            # Per millisecond cubic sample table preserves velocity between 5 ms wheel samples.
            u=min(len(table)-1,max(0,round(at*2000)));q=table[u]
            _,ex,ey,ea,_,_=endpose
            if park and at>=park_at:
                pos,_=at_park.at(at-park_at)
                dx,dy=f.rot((pos[0]-f.LIP[0]+.490,pos[1]-f.LIP[1]-sector),-f.NORMAL)
                return "loaded-park-and-hold",ex+dx,ey+dy,ea+pos[2],.5,0.
            dx,dy=f.rot((q[0]/1000,(q[1]/1000-sector)),-f.NORMAL)
            return phase,ex+dx,ey+dy,ea+q[2],.5,0 if t<self.drive_end else stroke
    route=Measured(sector)
    endpose=f.RegisteredMission.pose(route,route.drive_end)
    if park:
        plan=next(p for p in json.loads((ROOT/"fixed-height-match/placements.json").read_bytes())["placements"] if p["id"]==pipe)
        # The integrator adds one second to terminal_time. Carry through the 156 s assessment.
        origin_wall=18+plan["drive"]-route.before_drive
        route.terminal_time=156-origin_wall-1+.0005
    table=np.array([interpolate(samples,t/2000) for t in range((len(samples)-1)*10+1)])
    port=json.loads((ROOT/"design/fixed-height/ports-065.json").read_bytes())["cases"]["0ms-0.0005" if pipe=="coral-A" else "19ms-0.00025"]
    seat=port["first_in_band"];_,cx,cy,angle,top,_=route.pose(1.1)
    u=seat["u_mm"]/1000; speed=seat["speed_mps"]
    law=handoff.HeldContact(handoff.CANDIDATE,seat["spin_rad_s"],handoff.Trace("RAMP_090"),False)
    run=spin.mission.integrate("reference",.0005,spin.M,pipe,sector,not park,route,law.normals,
            seat["from_lip_ms"]/1000-1.1,1.1,(cx+u*math.cos(angle),cy+u*math.sin(angle),top+spin.R),
            (speed*math.cos(angle),speed*math.sin(angle)),angle,0,"resume-from-modeled-java-in-band",law)
    times={r["event"]:r["time_s"] for r in run["events"]};rows=run["trace"]
    last=rows[-1]
    pre=[]
    for r in rows:
        if r["time_s"]>times.get("place",1e9):break
        x,y,z=r["xyz_m"];cx,cy,h=r["chassis_xy_heading"];dx=x-cx;dy=y-cy
        pre.append([1000*(dx*math.cos(h)+dy*math.sin(h)-r["slide_m"]),1000*(-dx*math.sin(h)+dy*math.cos(h)),1000*z-557.15])
    pre=np.array(pre)
    result={"pipe":pipe,"mode":row["mode"],"sourcePort":port["first_in_band"],"outcome":run["outcome"],"eventsFromLipS":times,
        "spinAtTarget":law.spin,"lastXYZm":last["target_xyz_m"],"lastVelocityXYmps":last["target_vxvy_mps"],"lastRobotRelativeYawRad":last["robot_relative_yaw_rad"],
        "custodyCenterMinMaxMm":[pre.min(axis=0).tolist(),pre.max(axis=0).tolist()],"poseSha256":row["motionSHA256"]}
    if park:
        signed_speeds=last["target_vxvy_mps"]
        result["stowedPark"]={"nativeClockStop":run["outcome"],"finalPipeVelocityXYmps":signed_speeds,"sampleCount":len(rows),
            "durationS":at_park.duration,"sourceId":pipe,"stayedOnShelf":run["outcome"]=="time-limit-unresolved" and "loaded-park-and-hold" in times and not "place" in times,
            "assessmentWallS":origin_wall+last["time_s"]-(seat["from_lip_ms"]/1000-1.1),
            "finalMaximumDisplacementLastSecondMm":1000*max(math.dist(v["xyz_m"],last["xyz_m"]) for v in rows if v["time_s"]>=last["time_s"]-1)}
        return result
    if run["outcome"]=="paired-trough-first-contact":
        try:
            _,_,moment,state,y,_=spin.settle.incoming("reference",sector,pipe,run)
            result["postContactAdmission"]={"status":"eligible","time":moment,"state":state,"y":y}
        except ValueError as exc:
            result["postContactAdmission"]={"status":"withheld","reason":str(exc)}
    # Continue from the attained release edge. The historical free-flight adapter compares
    # to a shelf fixed 490 mm from the target even when the observed chassis stops elsewhere.
    # Keep its adverse control above and check the actual stationary shelf in chassis axes.
    edge=next((v for v in run["events"] if v["event"]=="left-powered-shelf-with-same-piece"),None)
    if edge:
        r=min(rows,key=lambda n:abs(n["time_s"]-edge["time_s"]))
        p=np.array(edge["target_xyz_m"]); vel=np.array([*r["target_vxvy_mps"],0.])
        h=samples[-1,2];body=np.array([-.49+samples[-1,0]/1000,samples[-1,1]/1000])
        yaw=h+r["robot_relative_yaw_rad"];min_gap=1e9;clear=None
        contacts=spin.mission
        for n in range(1,3001):
            t=n*.00005
            rel=p+vel*t-np.array([0,0,.5*contacts.G*t*t])
            d=rel[:2]-body;u=d[0]*math.cos(h)+d[1]*math.sin(h)
            gap=contacts.shelf_separation(u,rel[2],yaw-h,.535,.5);min_gap=min(min_gap,gap)
            if gap>=.001 and clear is None:clear=t
            if gap<-.000001:reason="registered-shelf-recontact";break
            # The source function checks local trough, riser and finite panel geometry. The
            # separately tested chassis-frame shelf is removed from this field-only query.
            reason,foot=contacts.free_contacts(contacts.M["targetLipX_m"]+rel[0],rel[1],rel[2],yaw,-100.,.5)
            if reason:break
        result["registeredFreeFlight"]={"outcome":reason,"elapsedS":t,"oneMmShelfClearS":clear,"minShelfGapMm":1000*min_gap,
            "worldPipeYawDeg":math.degrees(yaw),"targetVelocityXYmps":vel[:2].tolist(),"touchXYZm":rel.tolist()}
        if reason=="paired-trough-first-contact":
            # The documented downstream reduction pins both out-of-plane velocity and alignment.
            # Test the source's present admissibility threshold explicitly for the attained state.
            result["downstream2dVelocityLimitMps"]=1e-5
            result["outOfPlaneVelocityWithinLimit"]=bool(abs(vel[1])<=1e-5)
    return result

def run(cache):
    results=json.loads((HERE/"results.json").read_bytes())
    out=[case(r,cache) for r in results if r["mode"]=="nominal" and r["accepted"]]
    retained=case(next(r for r in results if r["mode"]=="nominal" and r["pipe"]=="coral-A"),cache,park=True)
    (HERE/"park-custody.json").write_text(json.dumps(retained,indent=2)+"\n",encoding="utf8",newline="\n")
    (HERE/"pipe.json").write_text(json.dumps(out,indent=2)+"\n",encoding="utf8",newline="\n")
    for r in out:print(r["pipe"],r["outcome"],r.get("postContactAdmission"))
if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("cache",type=Path);run(p.parse_args().cache)
