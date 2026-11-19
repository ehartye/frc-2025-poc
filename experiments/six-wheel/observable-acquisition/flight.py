"""Check the inbound robot-space sweep for the centered source used by the selected acquisition runs."""
import csv
import io
import json
import math
from types import SimpleNamespace
from unittest.mock import patch
import study
from sweep import HERE, ROOT, cad, corridor


def run():
    source = json.loads((HERE/"results.json").read_bytes())
    poses = ROOT/"design/competition-package/shop-duplex/red"
    with (HERE/"ticks.csv").open() as stream:
        ticks = list(csv.DictReader(stream))
    frames = json.loads((poses/"part-frames.json").read_bytes())
    parts = cad.read_named(poses/"receive.step", [{"id": k} for k in frames])
    boxes = {k:cad.bounds(v) for k,v in parts.items()}
    groups = {}
    for name, case in source["conditions"].items():
        for step, trial in source["cases"][name].items():
            if not case["counterfeed"] or trial["outcome"] != "IN_BAND": continue
            rows = [r for r in ticks if r["case"] == name and int(r["step_us"]) == round(float(step)*1e6)]
            lip = 100 + case["phase_ms"]
            cutoff = lip + trial["first_contact_ms_mm"][0]
            if any(float(r["belt_mps"]) != .45 for r in rows if int(r["ms"]) <= cutoff):
                raise ValueError("pre-contact speed differs from the tested profile")
            key = (case["depth_mm"],case["grip_mu"],case["preload"],case["ramp_mu"],step)
            groups.setdefault(key, (name,case,rows,trial))
    results = []
    original = study.AxialContact.normals
    for key, (name,case,rows,trial) in groups.items():
        path = []
        def observe(contact,u,v,yaw,phase,time,zrel=0):
            if zrel > 1e-8: path.append((time, (u+.240)*1000,(study.route.SHELF+zrel)*1000))
            return original(contact,u,v,yaw,phase,time,zrel)
        commands = ["{state},{active},{belt_mps},{reason}".format(**r) for r in rows]
        driver = SimpleNamespace(stdin=io.StringIO(),stdout=io.StringIO("\n".join(commands)+"\n"))
        with patch.object(study.AxialContact,"normals",observe):
            rebuilt = study.run(driver, study.Case(**case), float(key[-1]), [])
        if rebuilt != trial:
            # JSON serializes the impact pair as an array.
            if json.loads(json.dumps(rebuilt)) != trial: raise ValueError("plant replay changed")
        t,u = trial["first_contact_ms_mm"]
        path.append(((t+100+case["phase_ms"])/1000,u+240,937.15))
        # Bound the smooth trajectory between samples by a chord sag using a 35 m/s²
        # outer acceleration (gravity plus both 10 N end contacts at mu=.4, divided by mass).
        overlaps = {}
        block = max(1,round(.01/float(key[-1])))
        for i in range(0,len(path)-1,block):
            chunk = path[i:i+block+1]
            ta,x0,z0 = chunk[0]
            tb,x1,z1 = chunk[-1]
            if tb-ta < 1e-10: continue
            error = max(math.hypot(x-x0-(x1-x0)*(t-ta)/(tb-ta),z-z0-(z1-z0)*(t-ta)/(tb-ta)) for t,x,z in chunk)
            extra = error + 35000*float(key[-1])**2/8 + 1e-6
            for k,v in corridor(parts,boxes,(x0,z0),(x1,z1),extra).items():
                overlaps[k] = max(overlaps.get(k,0),v)
        results.append({"depth_mm":key[0],"step_s":float(key[-1]),"padded_outer_overlap_mm3":overlaps})
    return {"results_sha256":study.hashlib.sha256((HERE/"results.json").read_bytes()).hexdigest(),
            "pose_sha256":study.hashlib.sha256((poses/"receive.step").read_bytes()).hexdigest(),
            "inbound_groups":results}


if __name__ == "__main__":
    (HERE/"flight.json").write_text(json.dumps(run(),indent=2)+"\n",encoding="utf-8",newline="\n")
