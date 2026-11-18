"""Registered rest-to-rest differential transit, exact straight sweeps and conservative turning disks."""
import bisect
import json
import math
from pathlib import Path

if __package__:
    from . import kickoff_route_clearance as sweep
else:
    import kickoff_route_clearance as sweep

ROOT=Path(__file__).resolve().parents[1]
REG=json.loads((ROOT/"mission/field-registration.json").read_text())
PROFILE=json.loads((ROOT/"kickoff/season-profile.json").read_text())
IN=.0254
TAU=2*math.pi


def rot(p,a):
    x,y=p
    return x*math.cos(a)-y*math.sin(a), x*math.sin(a)+y*math.cos(a)


def add(p,q):
    return p[0]+q[0],p[1]+q[1]


def sub(p,q):
    return p[0]-q[0],p[1]-q[1]


def angle(p):
    return math.atan2(p[1],p[0])


def short(a):
    return math.atan2(math.sin(a),math.cos(a))


def field_from_native(x,z):
    return ((PROFILE["field"]["length"]/2+x)*IN,(PROFILE["field"]["width"]/2-z)*IN)


station=REG["redStation"]
STATION=field_from_native(station["nativeOriginIn"][0],station["nativeOriginIn"][2])
HUMAN=station["humanSideNativeBasis"]
NORMAL=angle((-HUMAN[0],HUMAN[2]))
LIP=field_from_native(REG["redReef"]["lipOriginNativeMm"][0]/25.4,0)
DOCK=add(STATION,rot((.434525+station["extraStandOffM"],0),NORMAL))
STAGE=add(STATION,rot((1.25,0),NORMAL))
NORTH=tuple(x*IN for x in REG["waypointsIn"]["northTransit"])
DRIVE=REG["drive"]


def wheel_profile(length, speed, accel):
    peak=min(speed,math.sqrt(length*accel))
    rise=peak/accel
    coast=max(0,length/peak-rise)
    return peak,rise,coast,2*rise+coast


class Route:
    def __init__(self,start):
        self.start=tuple(start)
        self.end=self.start
        self.segments=[]
        self.times=[]
        self.duration=0.0

    def segment(self,kind,demand):
        if abs(demand)<1e-10:
            return
        x,y,h=self.end
        if kind=="turn":
            wheel=abs(demand)*DRIVE["trackM"]/2
            v=DRIVE["maximumTurnRadS"]*DRIVE["trackM"]/2
            a=DRIVE["turnWheelAccelMps2"]
            end=(x,y,h+demand)
        else:
            wheel=abs(demand)
            v=DRIVE["maximumWheelMps"]
            a=DRIVE["maximumWheelAccelMps2"]
            end=(x+demand*math.cos(h),y+demand*math.sin(h),h)
        peak,rise,coast,total=wheel_profile(wheel,v,a)
        self.segments.append(dict(kind=kind,demand=demand,start=self.end,end=end,
                                  wheel=wheel,a=a,peak=peak,rise=rise,coast=coast,dt=total))
        self.end=end
        self.duration+=total
        self.times.append(self.duration)

    def face(self,h):
        self.segment("turn",short(h-self.end[2]))

    def go(self,point,reverse=False):
        diff=sub(point,self.end)
        if math.hypot(*diff)<1e-9:
            return
        self.face(angle(diff)+(math.pi if reverse else 0))
        self.segment("drive",(-1 if reverse else 1)*math.hypot(*diff))

    def at(self,t):
        if t<=0:
            return self.start,(0,0)
        if t>=self.duration:
            return self.end,(0,0)
        i=bisect.bisect_right(self.times,t)
        s=self.segments[i]
        a=s["a"]
        elapsed=t-(self.times[i-1] if i else 0)
        if elapsed<s["rise"]:
            speed=a*elapsed
            distance=a*elapsed**2/2
        elif elapsed<s["rise"]+s["coast"]:
            speed=s["peak"]
            distance=s["peak"]*(elapsed-s["rise"]/2)
        else:
            left=s["dt"]-elapsed
            speed=a*left
            distance=s["wheel"]-a*left**2/2
        f=distance/s["wheel"]
        pose=tuple(x+f*(y-x) for x,y in zip(s["start"],s["end"]))
        speed=math.copysign(speed,s["demand"])
        wheels=(-speed,speed) if s["kind"]=="turn" else (speed,speed)
        return pose,wheels


def routes(sector):
    reef=(LIP[0]-.49,LIP[1]+sector)
    west=(REG["waypointsIn"]["westApproachX"]*IN,reef[1])
    blue=PROFILE["rehearsal"]["starts"][REG["startSlot"]["slot"]-1]
    enter=Route(((PROFILE["field"]["length"]-blue[0])*IN,blue[1]*IN,math.pi))
    enter.go(NORTH)
    enter.go(STAGE)
    enter.go(DOCK)
    carry=Route(enter.end)
    carry.go(STAGE,reverse=True)
    carry.go(NORTH)
    carry.go(west)
    carry.go(reef)
    back=Route(carry.end)
    back.go(west,reverse=True)
    back.go(NORTH)
    back.go(STAGE)
    back.go(DOCK)
    return enter,carry,back


def obstacles():
    out=[(l["name"],[(x*IN,y*IN) for x,y in sweep.regular_landmark(l)]) for l in PROFILE["landmarks"]]
    for o in PROFILE["rehearsal"]["obstacles"]:
        if o["kind"]=="overlay" and o["lower"]*IN>1.020:
            continue
        out.append((o["name"],[(x*IN,y*IN) for x,y in o["poly"]]))
    return out


def audit(route, bounds, extra=()):
    x1,y1,_,x2,y2,_=bounds
    body=[(x*.001,y*.001) for x,y in [(x1,y1),(x2,y1),(x2,y2),(x1,y2)]]
    radius=max(math.hypot(*p) for p in body)
    planes=[("blue wall",(0,0),(1,0)),("red wall",(PROFILE["field"]["length"]*IN,0),(-1,0)),
            ("lower rail",(0,0),(0,1)),("upper rail",(0,PROFILE["field"]["width"]*IN),(0,-1))]
    for plane in PROFILE["rehearsal"]["planes"]:
        spot=tuple(v*IN for v in plane["at"])
        normal=rot((1,0),math.radians(plane["inwardDeg"]))
        if "tag 2" in plane["name"]:
            spot=add(STATION,rot((station["bumperFaceRobotwardM"],0),NORMAL))
            normal=rot((1,0),NORMAL)
        planes.append((plane["name"],spot,normal))
    results=[]
    for s in route.segments:
        a,b,h=s["start"]
        ex,ey,_=s["end"]
        shape=[rot(c,h) for c in body]
        gaps=[]
        for name,spot,n in planes:
            d=min((x-spot[0])*n[0]+(y-spot[1])*n[1] for x,y in ((a,b),(ex,ey)))
            support=-radius if s["kind"]=="turn" else min(x*n[0]+y*n[1] for x,y in shape)
            gaps.append((d+support,name))
        for name,poly in [*obstacles(),*extra]:
            if s["kind"]=="turn":
                # Distance to edge uses a circumscribed rotating disk. Include polygon interior.
                signs=[(d[0]-c[0])*(b-c[1])-(d[1]-c[1])*(a-c[0]) for c,d in sweep.edges(poly)]
                dist=0 if min(signs)>=0 or max(signs)<=0 else min(sweep.edge_distance((a,b),c,d) for c,d in sweep.edges(poly))
                gap=dist-radius
            else:
                at,gap=sweep.sweep((a,b),(ex,ey),shape,poly)
                if at is not None:
                    gap=0
            gaps.append((gap,name))
        closest=min(gaps)
        results.append({"kind":s["kind"],"seconds":s["dt"],"closest":closest[1],"gap_m":closest[0]})
    return results

class RegisteredMission:
    def __init__(self,sector):
        self.sector=sector
        self.additional_stand_off_mm=station["extraStandOffM"]*1000
        self.approach,self.carry,self.return_leg=routes(sector)
        self.before_drive=5.1
        self.drive_end=self.before_drive+self.carry.duration
        self.terminal_time=self.drive_end+1.5+3+1.5

    def pose(self,t):
        def smooth(value):
            v=max(0,min(1,value))
            return v**3*(10-15*v+6*v*v)
        elapsed=t
        name="seat"
        top=.880
        stroke=.240
        if t>=1.1:
            name="retract"
            stroke=.240*(1-smooth((t-1.1)/2))
        if t>=3.1:
            name="lower"
            top=.880-.270*smooth((t-3.1)/2)
        if t>=self.before_drive:
            name="drive-and-turn"
        if t>=self.drive_end:
            name="lower-at-target"
            top=.610-.110*smooth((t-self.drive_end)/1.5)
        if t>=self.drive_end+1.5:
            name="extend-at-target"
            stroke=.370*smooth((t-self.drive_end-1.5)/3)
        if t>=self.drive_end+4.5:
            name="place"
        p,_=self.carry.at(t-self.before_drive)
        x,y=rot(sub(p,STATION),-NORMAL)
        # Difference of cumulative headings, never a wrapped numerical signal.
        h=p[2]-self.carry.start[2]+math.pi
        return name,x,y,h,top,stroke

    def target_position(self,x,y,z):
        field=add(STATION,rot((x,y),NORMAL))
        return field[0]-LIP[0],field[1]-LIP[1],z

    def target_velocity(self,vx,vy):
        return rot((vx,vy),NORMAL)
