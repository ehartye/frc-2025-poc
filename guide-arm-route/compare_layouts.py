"""Collect matched CAD and two-wheel docking controls while retaining their source identities."""
import argparse
import json
from pathlib import Path
from field_view import HERE, digest, read

LAYOUTS=("rise-+0","rise--36","rise--22","rise-+18","rise-+32","rise-+48","rise-+55","rise-+65",
         "rise-+73","rise-+77","straight-narrow","wide-corridor","narrow-corridor","shallow-corridor",
         "front-return","central-return","rear-return","deep-return")


def collect(raw):
    rows={}
    for name in LAYOUTS:
        folder=raw/name
        meta=read(folder/"revision.json")
        optical=read(folder/"field.json")
        fit=read(folder/"clearance.json")
        step=digest(folder/"route.step")
        if any(row["stepSha256"]!=step for row in (meta,optical,fit)):
            raise ValueError("Incompatible inputs for "+name)
        nominal=[r for r in optical["runs"] if r["mode"]=="nominal"]
        if sorted(r["wheel"] for r in nominal)!=["leftSmall","rightSmall"]:
            raise ValueError("Both signed approaches are required")
        rows[name]={"stepSha256":step,"volumeDeltaMm3":meta["armVolumeChangeMm3"],
            "rasterSha256":optical["imageSha256"],"intersectionMm3":fit["intersectionMm3"],
            "cableIntrusionMm":{k:v for k,v in fit["nearCableClearanceMm"].items() if v<-.001},
            "wheelCases":{r["wheel"]:{"motionSha256":r["motionSHA256"],"stoppedErrorMmDeg":r["finalBodyErrorMmDeg"],
                "permitted":r["accepted"],"reason":r["reason"]} for r in nominal}}
    return rows


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("raw",type=Path)
    rows=collect(p.parse_args().raw)
    (HERE/"layouts.json").write_text(json.dumps(rows,indent=2)+"\n",encoding="utf8",newline="\n")
