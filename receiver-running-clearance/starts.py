"""Bind terminal receiver loads to the actual A executions and their wall clocks."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

def digest(p):
    with p.open("rb") as f:return hashlib.file_digest(f,"sha256").hexdigest()

def capture(source,raw,timesteps):
    cases={}
    for side in ("leftSmall","rightSmall"):
        for step in timesteps:
            record=source/f"cargo-{side}-{step:g}.json"
            d=json.loads(record.read_bytes())
            traj=raw/(d["id"]+".npy")
            if (digest(traj)!=d["trajectorySha256"] or d["wheelCase"]!=side or d["firstFault"] is not None
                or d["lostPipe"] is not None or not d["fieldPermitted"] or d["mode"]!="nominal"):
                raise ValueError("Wrong or unqualified arrival")
            last=np.load(traj,mmap_mode="r")[-1]
            if len(last)!=59 or not np.array_equal(last,d["lastPipe"]): raise ValueError("Saved terminal state differs")
            field=source/f"field-{side}.json"
            key=f"{side}-{step:g}"
            cases[key]=dict(wheelSide=side,feedStepUs=step,carryStepS=d["stepS"],wallS=d["finalWallS"],
                            sourceRecordSha256=digest(record),rawSha256=d["trajectorySha256"],runId=d["id"],
                            fieldSourceSha256=digest(field),programQuerySha256=digest(source/"plant.py"),
                            roofFrames=d["roofFrames"],p=last[1:4].tolist(),a=last[4:7].tolist(),
                            v=last[7:10].tolist(),omega=last[10:13].tolist(),idlers=last[13:15].tolist(),
                            z=last[15:36].tolist(),zdot=last[36:57].tolist(),plunger=last[57:59].tolist())
    return dict(cases=cases,scope="Attained stowed cargo on the original hardware. Restart on each changed section is a conditional fixture and requires independent acquisition/observer transfer.")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("source","raw","output"):p.add_argument(name,type=Path)
    p.add_argument("--feed-us",type=float,nargs="+",default=[31.25])
    a=p.parse_args();r=capture(a.source,a.raw,a.feed_us)
    a.output.write_bytes((json.dumps(r,indent=2,allow_nan=False)+"\n").encode())
