"""Test independent bilateral optical geometry and clocked custody holds."""
import json
import math
from dataclasses import replace
from motion import Case,run,HERE
from gate_cad import base,shapes,cad,sha
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.gp import gp_Dir,gp_Lin,gp_Pnt

def range_mm(c,row,lane):
    _,x,z,*_=row
    yaw=math.radians(c.yaw);y=lane/1000-c.lateral;r=c.diameter/2
    along=-x*(-math.sin(yaw))+y*math.cos(yaw)
    radial=-x*math.cos(yaw)+y*math.sin(yaw)
    if abs(along)>=c.length/2 or abs(radial)>=r:return None
    return 696-1000*(z+math.sqrt(r*r-radial*radial))

def consent(samples,clock,piece_id):
    if not piece_id or len(samples)<4:return False
    window=samples[-4:]
    if clock-window[-1]["time"]>.030 or window[-1]["time"]>clock:return False
    if any(not .015<=b["time"]-a["time"]<=.025 for a,b in zip(window,window[1:])):return False
    for row in window:
        if row["piece"]!=piece_id or not all(row[k] for k in ("hoodDown","leftHome","rightHome","driveQuiet","loadKnown")):
            return False
        if any(not v or not 2<=v<=24 for v in row["rangeMm"]):return False
    return True

def check_rays():
    geom=base(30)|shapes(30)
    result={}
    for name,y in (("L",110),("R",-110)):
        limits=[]
        # Center and eight rays on a 9-degree half cone. Do not claim the entire beam cone from these samples.
        rays=[(0,0,-1)]
        for n in range(8):
            a=2*math.pi*n/8;s=math.sin(math.radians(9))
            rays.append((s*math.cos(a),s*math.sin(a),-math.cos(math.radians(9))))
        for dx,dy,dz in rays:
            line=gp_Lin(gp_Pnt(30,y,695.99),gp_Dir(dx,dy,dz))
            hits=[]
            for k,part in geom.items():
                if k=="optical-range-"+name:continue
                b=cad.bounds(part)
                if b[3]<14 or b[0]>46 or b[4]<y-16 or b[1]>y+16 or b[5]<595 or b[2]>696:continue
                q=IntCurvesFace_ShapeIntersector();q.Load(part,.001);q.Perform(line,.001,100)
                if not q.IsDone():raise ValueError(k)
                if q.NbPnt():hits.append((q.WParameter(1),k))
            limits.append(min(hits) if hits else (100,None))
        result[name]={"shortestEmptyRayMm":min(x for x,_ in limits),"sampleCount":len(limits)}
    return result

def build():
    result={}
    for name,c in (("center",Case()),("long-yawed",Case(yaw=4,lateral=.010,length=.307975)),
                   ("short-yawed",Case(yaw=-4,lateral=-.010,length=.295275))):
        data=run(c,True);rows=[]
        for row in data["trace"]:
            t=row[0]
            if not .8<=t<3.46 or round(t*1000)%20:continue
            vals=[range_mm(c,row,y) for y in (110,-110)]
            rows.append(dict(time=t,rangeMm=vals,piece="station-A-01",hoodDown=True,leftHome=True,rightHome=True,
                             driveQuiet=t>=.85,loadKnown=data["seatedS"] is not None and t>=data["seatedS"]))
        first=next((rows[i]["time"] for i in range(4,len(rows)) if consent(rows[:i+1],rows[i]["time"],"station-A-01")),None)
        ranges=[v for row in rows for v in row["rangeMm"] if v is not None]
        row=rows[-1];clock=row["time"]
        tests={"valid":consent(rows,clock,row["piece"]),"stale":consent(rows,clock+.04,row["piece"]),
               "crossedIdentity":consent(rows,clock,"other")}
        for key,change in (("oneBlind",{"rangeMm":[row["rangeMm"][0],None]}),("loadUnknown",{"loadKnown":False}),
                           ("gateHalfHome",{"rightHome":False}),("stillDriving",{"driveQuiet":False})):
            tests[key]=consent(rows[:-1]+[row|change],clock,row["piece"])
        result[name]={"firstConditionalCustodyS":first,"rangeMm":[min(ranges),max(ranges)],"controls":tests}
    return {"sourceNative":sha(HERE/"split-gate.step"),"apertureDegrees":18,"clearRayProbes":check_rays(),
            "stationCases":result,"coherentFalseOrStuckBothRangesWithFalseLoad":"indistinguishable; pre-run empty check, independent physical hood reaction and operator abort required"}
if __name__=="__main__":
    (HERE/"observer.json").write_text(json.dumps(build(),indent=2)+"\n",encoding="utf8",newline="\n")
