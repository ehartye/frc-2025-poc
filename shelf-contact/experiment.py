"""Distinguish declared floor constitutive hypotheses and independent numerical refinements."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict,replace
import json
from pathlib import Path
from contact import CompliantPipe,Shelf,ROOT,HERE,feed
from guide import Guide

GUIDE=Guide(throat_mm=-.3,stiffness_n_mm=10,damping_n_s_m=200,upper_mm=850,x_width_mm=40,x_offset_mm=20)
POSES=json.loads((ROOT/"station-guide/optics-narrow.json").read_bytes())["trials"]


def source(name):
    row=POSES["nominal" if name=="shiftedHumanSource" else name]
    n,y,h=row["finalStationMmDeg"]
    return [-n,-y,h],3. if name=="shiftedHumanSource" else 0.


def early(job):
    name,dt,k,a,stations=job
    delta,shift=source(name)
    pipe=CompliantPipe(delta,shift,GUIDE,Shelf(k,a,stations=stations))
    pipe.belt=.65
    while .330-pipe.time>1.e-11 and not pipe.loss and not pipe.hard:
        pipe.moves(.65,min(dt,.330-pipe.time))
    return {"case":name,"dt":dt,"shelf":asdict(pipe.shelf),"time":pipe.time,"hardStop":pipe.hard,
            "loss":pipe.loss,"peakCartridgeMm":float(pipe.peak),"peakCompressionMm":1000*pipe.peak_shelf_compression,
            "peakShelfN":pipe.peak_shelf_n,"firstNormalMps":pipe.first_impact_speed,"shelfImpulseNs":pipe.shelf_impulse_ns,
            "finalUmm":1000*pipe.p[0],"shelfDissipationJ":pipe.normal_dissipation_j,
            "shelfFrictionJ":pipe.friction_dissipation_j}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("out",type=Path);args=parser.parse_args()
    cases=("nominal","leftSmall","rightSmall","shiftedHumanSource","badRegistration")
    jobs=[(n,dt,400_000.,a,8) for a in (0.,1.,4.,12.) for n in cases for dt in (.00025,.000125,.0000625)]
    jobs.extend((n,.000125,k,4.,8) for k in (100_000.,1_600_000.) for n in cases)
    with ProcessPoolExecutor(max_workers=5) as pool:rows=list(pool.map(early,jobs))
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(rows,indent=2)+"\n",encoding="utf8",newline="\n")
    for a in (0.,1.,4.,12.):
        print(a,[(n,[(round(r["peakCartridgeMm"],3),round(r["peakCompressionMm"],3)) for r in rows
                    if r["case"]==n and r["shelf"]["dissipation_s_m"]==a and r["shelf"]["stiffness_n_m"]==400_000.]) for n in cases],flush=True)
