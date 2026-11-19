"""Bound a finite rack acceleration while preserving both end times of the low-match retreat."""
import json
from pathlib import Path
import sys
from unittest.mock import patch

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"design/fixed-height"))
import handoff

TRAVEL=.370
DURATION=TRAVEL/.150
RISE=.200


def motion(t,rise=RISE):
    v=TRAVEL/(DURATION-rise)
    a=v/rise
    if t<=0:
        return 0.,0.,0.
    if t<rise:
        return .5*a*t*t,a*t,a
    if t<DURATION-rise:
        return v*(t-rise/2),v,0.
    if t<DURATION:
        left=DURATION-t
        return TRAVEL-.5*a*left**2,a*left,-a
    return TRAVEL,0.,0.


def run():
    sources=json.loads((ROOT/"fixed-height-match/placements.json").read_bytes())["placements"]
    ports=json.loads((ROOT/"design/fixed-height/ports-065.json").read_bytes())
    output={}
    for s,key in zip(sources,("0ms-0.0005","19ms-0.00025"),strict=True):
        module=handoff.spin.settle
        settle=module.continue_contact
        native_gap=module.shelf_gap
        results={}
        def evaluate(**kwargs):
            issued=settle(**kwargs)
            if kwargs["withdrawal"]!="after-separation":
                return issued
            results["issuedCompressedVelocityStep"]={"stop":issued["stop"],"gapMm":issued["min_shelf_clearance_mm"]}
            for rise in (.04,RISE,.50):
                step=kwargs["dt"]
                count=0
                def modified(x,z,front,top):
                    nonlocal count
                    elapsed=issued["start_from_station_s"]+count*step-issued["clear_witness_from_station_s"]-kwargs["wait_after_true_separation_s"]
                    count+=1
                    ideal=min(TRAVEL,max(0,elapsed)*.15)
                    stroke,_,_=motion(elapsed,rise)
                    return native_gap(x,z,front+ideal-stroke,top)
                with patch.object(module,"shelf_gap",modified):
                    held=settle(**kwargs,duration=3.0)
                results[str(rise)]={"stop":held["stop"],"gapMm":held["min_shelf_clearance_mm"],
                                     "terminalStillS":held["ending_still_double_contact_s"],"ticks":count}
            return issued
        piece=next(p for p in handoff.spin.DATA["identified_transfers"] if p["id"]==s["id"])
        with patch.object(module,"continue_contact",evaluate):
            case=handoff.one(piece,"RAMP_090",ports["cases"][key])
        if not abs(case["after-separation"]["targetContactTeleopS"]-s["impact"])<1e-8:
            raise ValueError("Issued timing mismatch")
        output[s["id"]]=results
    return output


if __name__=="__main__":
    (HERE/"retreat.json").write_text(json.dumps(run(),indent=2)+"\n",encoding="utf-8",newline="\n")
