"""Reproduce the post-contact sensitivity runs and native R6 shelf witnesses."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts import station_l1_settle as sim
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone


def main():
    folder=ROOT/"cad/station-receiver"
    spec=importlib.util.spec_from_file_location("shelf_witness",folder/"verify_contacts.py")
    probe=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(probe)
    source=folder/"shelf-r6.step"
    reader=STEPControl_Reader()
    if reader.ReadFile(str(source))!=IFSelect_RetDone or reader.TransferRoots()!=1:
        raise ValueError("Could not reopen the receiver shelf STEP")
    rows=[]
    for motion,mu,roll in (("fixed",.35,0),("late",.35,0),("after-separation",.35,0),
                          ("after-separation",0,0),("after-separation",.35,-10),
                          ("after-separation",.35,10)):
        for step in (.0002,.0001,.00005):
            r=sim.continue_contact(withdrawal=motion,mu=mu,initial_roll=roll,dt=step)
            trace=r.pop("trace")
            r["trace_sha256"]=hashlib.sha256(json.dumps(trace,separators=(",",":")).encode()).hexdigest()
            if step==.0001:
                times=[(trace[0][0],trace[0][1],trace[0][2]),
                       (r["elapsed_s"],r["last_xz_vxvz_spin"][0],r["last_xz_vxvz_spin"][1])]
                records=[]
                for t,x,z in times:
                    trigger={"fixed":float("inf"),"late":r["start_from_station_s"]+1.2,
                             "after-separation":r["clear_witness_from_station_s"]+.05}[motion]
                    front=.045-min(.37,max(0,r["start_from_station_s"]+t-trigger)*.15)
                    rec=probe.evaluate(reader.OneShape(),(front-x)*1000,(z-.5)*1000)
                    records.append({"elapsed_s":t,**rec})
                if records[0]["gap_mm"]<=1 or ((records[-1]["overlap_mm3"]>0)!=(r["stop"]=="receiver-recontact")):
                    raise ValueError("Native receiver geometry disagrees with terminal envelope")
                r["native_receiver_witnesses"]=records
            rows.append(r)
    pins={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in
          [source,folder/"verify_contacts.py",ROOT/"scripts/station_l1_settle.py",
           ROOT/"scripts/station_l1_mission.py",ROOT/"mission/candidate.json",ROOT/"mission/target.json",
           ROOT/"scripts/station_l1_phase.py",Path(__file__)]}
    print(json.dumps({"schema":"station-l1-post-contact/1","source_sha256":pins,"runs":rows},indent=2,allow_nan=False))


if __name__=="__main__":
    main()
