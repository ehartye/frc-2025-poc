"""Carry increased preload through the historical centered two-pipe, source-to-target calculation."""
import argparse
import hashlib
import json
from dataclasses import replace
from pathlib import Path
from capture import CANDIDATE, DATA, ROOT
from pipe_spin_route import trial

def run():
    rows={}
    for piece in DATA["identified_transfers"]:
        state,_,_=trial(replace(CANDIDATE,preload=11.5),piece,.0005)
        rows[piece["id"]]={k:state[k] for k in ("status","target","pickup","belt_motor_envelope",
                                  "release_spin_rad_s","min_shelf_normal_N") if k in state}
    return {"sourceSha256":{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/"axial-spin/pipe_spin_route.py",ROOT/"axial-spin/pipe_axial_contact.py",ROOT/"sourced-match/source-545.json")}, "scope":"historical centered route with 11.5 N grip preload; changed dock traffic and release hardware unqualified","cases":rows}

if __name__=="__main__":
    a=argparse.ArgumentParser(description=__doc__);a.add_argument("--output",type=Path,required=True)
    p=a.parse_args()
    p.output.write_text(json.dumps(run(),indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
