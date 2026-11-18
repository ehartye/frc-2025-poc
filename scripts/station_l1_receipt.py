"""Regenerate a source-pinned compact receipt; full sample trajectories are replayable by CLI."""
import hashlib
import json
from pathlib import Path

if __package__:
    from . import station_l1_mission as sim, station_l1_phase as phase
else:
    import station_l1_mission as sim
    import station_l1_phase as phase


def digest(data):
    return hashlib.sha256(data).hexdigest()


def receipt():
    scenarios = []
    for case in ("reference","light-pipe","weak-grip","slow-release","sensor-timeout","occupied-target"):
        for dt in (.002,.001,.0005):
            result = sim.run(case,dt)
            trace = result.pop("trace")
            scenarios.append({"case":case,"dt_s":dt,"mass_kg":result["mass_kg"],
                              "outcome":result["outcome"],"last":result["events"][-1],
                              "events":result["events"] if case=="reference" and dt==.002 else None,
                              "peak_supported":result["peak_supported"],
                              "samples":len(trace),"trace_digest":digest(json.dumps(trace,sort_keys=True,separators=(",",":")).encode())})
    files = ["mission/candidate.json","mission/target.json","cad/station-receiver/design.json",
             "scripts/station_edge_contact.py","scripts/receiver_contact.py","scripts/station_receiver_impact.py",
             "scripts/station_l1_mission.py","scripts/station_l1_phase.py","scripts/assess_coral.py",
             "scripts/station_l1_receipt.py"]
    reports = [phase.rehearse(),phase.rehearse(witnessed_clear=True),phase.rehearse(15,True,"AUTO")]
    clocks = [{"stop":r["stop"],"phase":r["phase"],"elapsed_s":r["elapsed_s"],
               "synthetic_clear":r["synthetic_empty_and_sector_clear_observations"],
               "pieces":r["pieces"],"assessments":r["assessments"]} for r in reports]
    return {"schema":"station-l1-mission-receipt/1",
            "source_sha256":{p:digest((sim.ROOT/p).read_bytes()) for p in files},
            "runs":scenarios,"clock_results":clocks}


if __name__=="__main__":
    print(json.dumps(receipt(),indent=2,allow_nan=False))
