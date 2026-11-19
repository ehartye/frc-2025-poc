"""Verify the published red-station scalar witnesses against the exact user-supplied FIRST STEP."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re


def verify(path):
    contract=json.loads(Path(__file__).with_name("field-registration.json").read_text())
    raw=path.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=contract["firstStepSha256"]:
        raise ValueError("A different revision requires an independent registration")
    text=raw.decode("latin1")
    def entity(key):
        m=re.search(rf"^#{key}\s*=\s*(.*?);",text,re.M|re.S)
        if m is None:
            raise ValueError(f"Missing STEP entity {key}")
        return m[1]
    def values(key):
        nums=re.search(r"\(\s*([-+\.0-9Ee,\s]+)\s*\)\s*\)",entity(key))
        return [float(x) for x in nums[1].split(",")]
    links=[(407553,"'NAUO34'"),(324125,"#407553"),(49616,"#324125"),(49616,"#103936"),
           (103936,"#271002"),(271002,"#333476"),(333476,"#277908"),(333476,"#151718"),
           (17987,"GE-25000_Red 3 Corner")]
    for key,token in links:
        if token not in entity(key):
            raise ValueError(f"Broken station occurrence chain at {key}")
    station=contract["redStation"]
    if math.dist(values(277908),station["nativeOriginIn"])>1e-6 or math.dist(values(151718),station["humanSideNativeBasis"])>1e-8:
        raise ValueError("Native station origin or normal differs")
    return {"station":"NAUO34","source_sha256":contract["firstStepSha256"],
            "parent_origin_in":values(277908),"human_side_direction":values(151718)}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("step",type=Path)
    print(json.dumps(verify(p.parse_args().step),indent=2))
