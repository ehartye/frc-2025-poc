"""Re-evaluate the source-fed pipe along ten admitted final approaches and preserve six-component release state."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from unittest.mock import patch
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"sector-alignment"))
import cargo
spin=cargo.handoff.spin

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def run(motions,cache):
    cache.mkdir(parents=True,exist_ok=True)
    f=spin.corrected()
    rows=json.loads((ROOT/"sector-alignment/results.json").read_bytes())
    result=[]
    for row in rows:
        if not row["accepted"] or row["truthOutsideDeclaredCalibration"]:continue
        pipe,mode=row["pipe"],row["mode"]
        initial=row["initial"]
        native=spin.mission.integrate
        trace={}
        def simulate(*args,**kwargs):
            args=list(args)
            route=args[6];pose=route.pose
            start=route.drive_end-route.carry.segments[-1]["dt"]
            def drift(t):
                phase,x,y,h,height,stroke=pose(t)
                if start-2<t<start:
                    u=(t-(start-2))/2;gain=u**3*(10-15*u+6*u*u)
                    dx,dy=f.rot((initial[0]*gain/1000,initial[1]*gain/1000),-f.NORMAL)
                    x+=dx;y+=dy;h+=gain*math.radians(initial[2])
                return phase,x,y,h,height,stroke
            route.pose=drift
            res=native(*args,**kwargs)
            trace["run"]=res
            return res
        with patch.object(spin.mission,"integrate",simulate):
            previous=cargo.case(row,motions)
        if previous["registeredFreeFlight"]["outcome"]!="paired-trough-first-contact":raise ValueError("No first contact")
        run=trace["run"];path=run["trace"]
        edge=next(e for e in run["events"] if e["event"]=="left-powered-shelf-with-same-piece")
        end=next(i for i,s in enumerate(path) if s["time_s"]>=edge["time_s"]-1e-7)
        sample=path[end];prior=path[end-1]
        hfinal=math.radians(previous["registeredFreeFlight"]["worldPipeYawDeg"])
        # Spin around the transverse +Y axis is negative for positive counterclockwise
        # roll in the project's X/Z drawing. Body yaw is differentiated in the same
        # unwrapped world frame; contact speed and angular direction remain signed.
        yawrate=(sample["yaw_rad"]-prior["yaw_rad"])/(sample["time_s"]-prior["time_s"])
        axis=np.array([-math.sin(hfinal),math.cos(hfinal),0.])
        omega=-previous["spinAtTarget"]*axis+np.array([0.,0.,yawrate])
        vel=np.array([*sample["target_vxvy_mps"],0.])
        pos=np.array(edge["target_xyz_m"])
        sector=spin.SECTORS[pipe]
        port=previous["sourcePort"]
        origin=100 if pipe=="coral-A" else 119
        prefix=(row["pipe"]+"-"+row["mode"])
        # Relative-to-shelf camera positions from the same dynamic run. Preserve signed
        # orientation around vertical and extend the free interval using its attained velocity.
        full=[]
        for rr in path[:end+1]:
            xx,yy,z=rr["xyz_m"];cx,cy,h=rr["chassis_xy_heading"]
            dx,dy=xx-cx,yy-cy
            full.append([origin+1000*rr["time_s"],1000*(dx*math.cos(h)+dy*math.sin(h)),
                         1000*(-dx*math.sin(h)+dy*math.cos(h)),1000*z,-math.degrees(rr["robot_relative_yaw_rad"]),1000*rr["slide_m"]])
        final=row["finalBodyErrorMmDeg"];target=np.array([-.49+final[0]/1000,sector+final[1]/1000])
        heading=math.radians(final[2]);flight=previous["registeredFreeFlight"]["elapsedS"]
        eclock=origin+1000*edge["time_s"]
        for dt in np.arange(.0005,flight+.001,.0005):
            d=pos+vel*dt-np.array([0,0,.5*spin.mission.G*dt*dt])
            xx,yy=d[:2]-target
            full.append([eclock+dt*1000,1000*(xx*math.cos(heading)+yy*math.sin(heading)),
                         1000*(-xx*math.sin(heading)+yy*math.cos(heading)),1000*d[2],
                         -math.degrees(hfinal+yawrate*dt-heading),370.])
        tr=cache/(prefix+".npy");np.save(tr,np.array(full))
        record={"pipe":pipe,"mode":mode,"poseSHA256":row["motionSHA256"],"trajectorySHA256":sha(tr),"carryRangeMm":previous["custodyCenterMinMaxMm"],
                "endFromLipMs":1000*edge["time_s"],"endCameraMs":eclock,"releaseCameraMs":origin+1000*previous["eventsFromLipS"]["place"],
                "fieldTouchCameraMs":eclock+flight*1000,"positionM":pos.tolist(),"velocityMps":vel.tolist(),"axis":axis.tolist(),"angularRadps":omega.tolist(),
                "yawRateRadps":yawrate,"rollXZRadps":previous["spinAtTarget"],"chassisAtReefXYm":target.tolist(),"chassisHeadingRad":heading,
                "sourceFirstContact":previous["registeredFreeFlight"],"inheritedPoseEntry":"smooth 2 s finite disturbance ending at the recorded terminal start"}
        print(pipe,mode,'lateral mm/s',round(1000*vel[1],6),'yaw deg',round(math.degrees(hfinal),6),flush=True)
        result.append(record)
    (HERE/"source.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf8",newline="\n")
if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("motions",type=Path);p.add_argument("cache",type=Path);a=p.parse_args();run(a.motions,a.cache)
