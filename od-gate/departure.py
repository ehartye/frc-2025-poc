"""Minimum all-body distance over each segment of the stored 2 ms release trajectory."""
import math
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from dock_tolerance import separation
from gate_cad import cad

def audit(c,trace,end,phase,layer,boxes,tube):
    # Hood is down, slide held at 370 mm; translation is exact in each named local frame.
    if end is None or c.withdraw or end <= c.close_time+c.close_duration:
        raise ValueError("Requires completed closed hood and fixed slide source")
    native={k:("chassis",s) for k,s in phase.items()}|{k:(f,s) for k,(f,_,s) in layer.items()}
    bounds={k:cad.bounds(s) for k,(_,s) in native.items()}
    before=next(i for i,row in enumerate(trace) if row[0]>=end-.004)
    tr=trace[before:]
    def distance(row):
        t,x,z,vx,vz,w,left,right,_=row
        piece=tube(1000*x,c.lateral*1000,1000*z,c.yaw,c)
        ref={"base":piece,"hood":piece,"chassis":cad.translated(piece,(370,0,0)),
             "L":cad.translated(piece,(0,85-1000*left,0)),"R":cad.translated(piece,(0,-85-1000*right,0))}
        br={k:cad.bounds(s) for k,s in ref.items()}
        best=(math.inf,"")
        for lower,n,f in sorted((separation(br[f],bounds[n]),n,f) for n,(f,_) in native.items()):
            if lower>best[0]+1e-8:break
            query=BRepExtrema_DistShapeShape(ref[f],native[n][1])
            if not query.IsDone():raise ValueError("Unresolved whole-body distance: "+n)
            best=min(best,(query.Value(),n))
        return best
    # Evaluate every 4 ms at departure, 10 ms in flight, 40 ms once nearly at rest.
    indices=[0]
    while indices[-1]<len(tr)-1:
        t=tr[indices[-1]][0]
        jump=2 if t<end+.075 else 5 if t<end+.8 else 20
        indices.append(min(len(tr)-1,indices[-1]+jump))
    points=[distance(tr[i]) for i in indices]
    brackets=[]
    for j,(i,k) in enumerate(zip(indices,indices[1:])):
        arc=0.
        for a,b in zip(tr[i:k],tr[i+1:k+1]):
            piece=math.hypot(b[1]-a[1],b[2]-a[2])
            gate=max(abs(b[6]-a[6]),abs(b[7]-a[7]))
            arc+=1000*(piece+gate)
        # With bounded total variation, at least one endpoint is within half the arc.
        # The tube is axisymmetric about its spin axis and the registered yaw is fixed.
        lower=max(0.,min(points[j][0],points[j+1][0])-arc/2)
        brackets.append((tr[i][0],tr[k][0],lower))
    last=max((i for i,b in enumerate(brackets) if b[2]<=1.),default=-1)
    tail=brackets[last+1:]
    return {"bodyCount":len(native),"firstHeldOneMmClearS":tail[0][0] if tail else None,
            "minimumSubsequentMm":min((b[2] for b in tail),default=None),
            "lastBoundAtOrBelowOneMm":brackets[last] if last>=0 else None,
            "terminalMinimumMm":points[-1][0],"terminalNearestPart":points[-1][1],
            "evaluatedIntervals":len(brackets),"endedS":tr[-1][0],
            "scope":"Linear interpolation of registered 2 ms center/gate states, fixed yaw, complete final robot. No physical observation."}
