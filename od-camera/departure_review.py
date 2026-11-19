"""Check the new total assembly at the two delivered outside-diameter trajectories."""
from dataclasses import replace
from bridge import *
from installed import tube
from departure import audit

def cases():
    base=motion.Case(withdraw=0)
    return {"center":base,"long":replace(base,lateral=.010,normal=.010,yaw=4,station_yaw=1.,length=.307975,lag=.025,target_error=.015)}

def run(kit):
    new=configuration(370,0,0,kit)
    gates={k:v for k,v in gate.local().items() if k not in GATE_REMOVE}
    phase={k:s for k,s in new.items() if k not in gates}
    result={}
    for k,c in cases().items():
        d=motion.run(c,trace=True);tr=d["trace"];out=d["eventsS"]["exit"]
        bound=audit(c,tr,out,phase,gates,tube)
        # Frame-normal front-plane exclusion is sufficient only while the measured tube is wholly beyond this plane.
        tip=max(cad.bounds(v)[3] for v in configuration(370,112,0,kit).values())
        yaw=math.radians(c.yaw)
        support=.5*(c.diameter*math.cos(yaw)+c.length*abs(math.sin(yaw)))
        vals=[(r[0],370+1000*(r[1]-support)-tip) for r in tr if r[0]>=out]
        result[k]={"native":bound,"gateSurfaceExitS":out,"settledS":d["endS"],"tipMm":tip,
                   "maximumFrontPlaneMarginMm":max(v for _,v in vals),"settledFrontPlaneGapMm":vals[-1][1],
                   "firstPlaneOverOneMmS":next((t for t,v in vals if v>1),None),"lastPlaneOverOneMmS":next((t for t,v in reversed(vals) if v>1),None),"peakPlaneAtS":max(vals,key=lambda row:row[1])[0],
                   "sampledOverOneMmIntervals":sum(v>1 for _,v in vals)}
    return {"bridgeSha":sha(HERE/"revised-bridge.step"),"originalODSha":sha(ROOT/"od-gate/split-gate.step"),
            "sourceWholeRegister":sha(ROOT/"design/fixed-no-air/whole-robot.json"),"cases":result}
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--lamp",type=Path,required=True);args=p.parse_args()
    (HERE/"departure.json").write_text(json.dumps(run(source(args.lamp)),indent=2)+"\n",encoding="utf8",newline="\n")
