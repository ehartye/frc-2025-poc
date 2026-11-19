"""Trace independent OD range probes to the nearest pipe or blocking native member."""
from departure_review import *
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.gp import gp_Pnt,gp_Dir,gp_Lin

def cast(origin,direction,geometry,limit=100.):
    line=gp_Lin(gp_Pnt(*origin),gp_Dir(*direction))
    end=[v+d*limit for v,d in zip(origin,direction)]
    bounds=[min(v,e)-.01 for v,e in zip(origin,end)]+[max(v,e)+.01 for v,e in zip(origin,end)]
    best=(limit,"no-return")
    for k,(s,b) in geometry.items():
        if any(bounds[i]>b[i+3] or b[i]>bounds[i+3] for i in range(3)):continue
        q=IntCurvesFace_ShapeIntersector();q.Load(s,.001);q.Perform(line,.001,limit)
        if not q.IsDone():raise ValueError("Unresolved range ray: "+k)
        best=min([best]+[(q.WParameter(j),k) for j in range(1,q.NbPnt()+1)])
    return best

def run(kit):
    results=[]
    for label,c in cases().items():
        data=motion.run(c,True);tr=data["trace"]
        for stage,stroke,t in (("station",30,.92),("carry",185,2.),("target-armed",370,3.40),("late-release",370,4.10)):
            row=min(tr,key=lambda a:abs(a[0]-t));tt,x,z,_,_,_,left,right,_=row
            geom=configuration(stroke,(left-.085)*1000,0,kit)
            for n in tuple(geom):
                if n.startswith("optical-range-"):geom.pop(n)
            geom={k:(s,cad.bounds(s)) for k,s in geom.items()}
            wall=tube(x*1000+stroke,c.lateral*1000,z*1000,c.yaw,c)
            sample={"case":label,"stage":stage,"strokeMm":stroke,"timeS":tt}
            for side,y in (("L",110),("R",-110)):
                origin=(stroke,y,695.99)
                rays=[(0.,0.,-1.)]+[(math.sin(math.radians(9))*math.cos(a*math.pi/4),
                     math.sin(math.radians(9))*math.sin(a*math.pi/4),-math.cos(math.radians(9))) for a in range(8)]
                sample[side]={"empty":[cast(origin,d,geom) for d in rays],
                              "filled":[cast(origin,d,geom|{"finite-wall":(wall,cad.bounds(wall))}) for d in rays]}
            results.append(sample)
    return {"bridgeSha":sha(HERE/"revised-bridge.step"),"source":sha(ROOT/"od-gate/observer.json"),
            "samples":results,"method":"Two 9-ray cones at actual closed hood poses. Fixed return limit 100 mm, range window 2–24 mm. Physical signal calibration, stale data and shared faults remain open."}
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--lamp",type=Path,required=True);a=p.parse_args()
    (HERE/"ranges.json").write_text(json.dumps(run(source(a.lamp)),indent=2)+"\n",encoding="utf8",newline="\n")
