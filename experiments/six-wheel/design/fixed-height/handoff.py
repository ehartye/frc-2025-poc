"""Propagate two attained Java optical-model seats without resetting their local state or clock."""
import json
import math
from pathlib import Path
from function import CANDIDATE,ROOT,HERE,spin,HeldContact,Trace


def one(piece,profile,port):
    sector=spin.SECTORS[piece["id"]]
    module=spin.corrected()
    class FixedLow(module.RegisteredMission):
        def pose(self,t):
            phase,x,y,a,_,stroke=super().pose(t)
            return phase,x,y,a,.5,0 if t<self.drive_end else stroke
    route=FixedLow(sector)
    ready=port["first_in_band"]
    _,cx,cy,yaw,top,_=route.pose(1.1)
    u=ready["u_mm"]/1000
    velocity=ready["speed_mps"]
    xyz=(cx+u*math.cos(yaw),cy+u*math.sin(yaw),top+spin.R)
    vxy=(velocity*math.cos(yaw),velocity*math.sin(yaw))
    contact=HeldContact(CANDIDATE,ready["spin_rad_s"],Trace(profile),False)
    # IN_BAND follows a 240 ms dwell at a zero command; the modeled drive has stopped.
    # Route's 1.1 s station-dwell endpoint starts its already-issued departure schedule.
    # Time stamps are lip-relative and preserve the separately measured instant at that endpoint.
    shift=ready["from_lip_ms"]/1000-1.1
    run=spin.mission.integrate("reference",.0005,spin.M,piece["id"],sector,True,route,contact.normals,
                              shift,1.1,xyz,vxy,yaw,0,"resume-from-modeled-java-in-band",contact)
    result={"sourceId":piece["id"],"command":profile,"startState":ready,"sourceClockAtFirstTraceS":run["trace"][0]["time_s"],
            "sourceWorldXYZ":xyz,"firstContactOutcome":run["outcome"],"spinAtTarget":contact.spin}
    if run["outcome"]=="paired-trough-first-contact":
        lip=piece["human_release_teleop_s"]+spin.DATA["feed"]["free_drop_s"]+spin.ramp(spin.DATA["feed"],CANDIDATE.ramp_mu)["time"]
        for mode in ("fixed","after-separation"):
            # 110 ms is a sensitivity taken from the substitute observer at this release pose.
            # The new whole-robot signal is not implemented and no authorization is inferred.
            kw=dict(sector=sector,piece_id=piece["id"],upstream=run,withdrawal=mode,initial_roll=contact.spin,
                    dt=.0002,inertia_ratio=spin.K,wait_after_true_separation_s=.110)
            response=spin.settle.continue_contact(**kw)
            result[mode]={"status":response["stop"],"minShelfGapMm":response["min_shelf_clearance_mm"],
                          "targetContactTeleopS":lip+response["start_from_station_s"]}
    return result


if __name__=="__main__":
    rows=[]
    for file in ("ports.json","ports-065.json"):
        record=json.loads((HERE/file).read_bytes())
        for piece,key in zip(spin.DATA["identified_transfers"],("0ms-0.0005","19ms-0.00025")):
            for profile in ("HISTORICAL","RAMP_090"):
                result=one(piece,profile,record["cases"][key])
                result["acquisitionCounterfeedMps"]=record["counterfeedMps"]
                rows.append(result)
    (HERE/"handoff.json").write_text(json.dumps(rows,indent=2)+"\n",encoding="utf-8",newline="\n")
