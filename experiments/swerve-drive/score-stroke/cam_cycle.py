"""Exercise actual stow, extension, dwell and return with the revised shelf and fixed caps."""
import argparse
import json
from pathlib import Path
from cam_model import activate,candidate_gaps
from loaded_rack import HERE,SIX,digest,read
import numpy as np
import dynamics
import dynamics_run as rack


def cycle(_, dt):
    duration=6.6
    t=np.arange(round(duration/dt)+1)*dt
    s,v,a=dynamics.profile(t,3.)
    back=t>=3.3
    delta,ret,acc=dynamics.profile(t[back]-3.3,3.)
    s[back],v[back],a[back]=.37-delta,-ret,-acc
    return duration,t,s,v,a


def run(station,out,sign,dt,issued_fixture=False):
    # The prior registered source geometry is independently validated before
    # substituting the candidate's measured section and candidate-specific floor.
    registered=rack.registration()
    receipt=activate()
    stock=registered[0]["movingStock"]
    for key,delta in receipt["volumeDeltaMm3"].items():
        stock[key]["massKg"]+=delta*stock[key]["densityKgM3"]*1e-9
    original_read=dynamics.read
    def candidate_read(path):
        if path==dynamics.HERE/"geometry.json":
            return registered[0]
        return original_read(path)
    dynamics.read=candidate_read
    rack.registration=lambda:registered
    rack.button_gaps=candidate_gaps
    rack.sample_motion=cycle
    if issued_fixture:
        entry=station/f"{sign}-15.625us"/f"{sign}-15.625us-service.npz"
        expected=read(entry.parent/"service.json")["outgoing"]
    else:
        packet=read(station/f"cam-station-{sign}-31.25us-0.1mm.json")
        if packet["state"]!="IN_BAND" or packet["loss"] or packet["hardStop"] or packet["javaLastMs"]!=5460:
            raise ValueError("Candidate station pickup did not retain this pipe")
        entry,=station.glob(f"cam-station-{sign}-31.25us-0.1mm/{sign}/*/state.npz")
        expected=packet["pipeStateSha256"]
    if expected!=digest(entry):
        raise ValueError("Station state changed")
    with np.load(entry) as f:
        state={key:f[key] for key in ("p","a","z","zdot")}
    name=f"{sign}-cam-cycle-{dt:g}us"+("-issued-fixture" if issued_fixture else "")
    result=rack.trial(state,name,out,dt*1e-6,motion=True)
    with np.load(out/(name+".npz")) as f:
        t,s,b=f["timeS"],f["strokeM"],f["buttonN"]
        mask=s>.121
        result.update(fullStrokeMm=float(1000*s.max()),timeAtFullS=float(t[np.argmax(s)]),
                      maxButtonForceBeyondRunnerN=float(np.max(b[mask])),
                      maxReturnButtonReactionN=float(np.max(b[t>3.3])),
                      minimumFullStrokePitchRad=float(f["q"][(t>=3)&(t<=3.3),5].min()))
    result.update(stationStateSha256=digest(entry),camStepSha256=receipt["candidateStepSha256"],issuedFixture=issued_fixture,
                  samePipeHeldForBothDirections=True,stationToDockMotion=None)
    (out/(name+".json")).write_text(json.dumps(result,indent=2,allow_nan=False)+"\n",encoding="utf8")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("station",type=Path);p.add_argument("out",type=Path)
    p.add_argument("sign",choices=("plus","minus"));p.add_argument("dt",type=float)
    p.add_argument("--issued-fixture",action="store_true",help="Prescribed isolated placement of the original pickup, irrespective of candidate pickup outcomes")
    args=p.parse_args()
    run(args.station,args.out,args.sign,args.dt,args.issued_fixture)
