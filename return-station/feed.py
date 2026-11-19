"""Deliver the second tube into each attained, stationary receiver and drive the Java port controller."""
import argparse
from contextlib import ExitStack
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"observable-acquisition"))
import study
from pipe_axial_contact import M,R,RI,HALF,I,G,ramp
from dataclasses import replace
cfg=study.CANDIDATE
IP=M*(3*(R*R+RI*RI)+4*HALF*HALF)/12
UP=np.array([0.,0.,1.])
TOP=.500
EDGE=.165


class Pipe:
    def __init__(self,delta,feed_side=0,relief=False):
        self.inward=.35 if relief else 1.5
        self.outward=1.4 if relief else 1.5
        self.preload=11.5 if relief else cfg.preload
        dx,dy,heading=delta
        h=math.radians(heading);c,s=math.cos(h),math.sin(h)
        src=ramp(study.route.DATA["feed"],cfg.ramp_mu)
        lip=study.route.station.departure(src["speed"])
        if lip["mode"]!="immediate-separation":raise ValueError("Missing chute departure")
        x,z=study.route.station.flight_center(lip,0)
        u=.444525-x-dx/1000
        v=(feed_side-dy)/1000
        self.p=np.array([u*c+v*s,-u*s+v*c,z])
        self.v=np.array([-lip["velocity_x_mps"]*c,lip["velocity_x_mps"]*s,lip["velocity_z_mps"]])
        self.a=np.array([s,c,0.])
        self.l=-I*src["spin"]*self.a
        self.belt=0.;self.touch=None;self.peak=0;self.hard=None;self.peak_tilt=0;self.low=1;self.max_pen=0
        self.first_belts=[None,None];self.time=0.;self.peak_load=0.;self.max_axis_rotation=0
        self.loss=None

    def angular(self,moment):
        return moment/IP+self.a*((1/I-1/IP)*float(self.a@moment))

    def footprint(self,side):
        """Exact clipped horizontal annulus strips on the current (inclined) end-cap plane."""
        ax,ay,az=self.a
        cap=side*HALF*self.a
        h=2*.0075/cfg.slices
        points=[]
        for j in range(cfg.slices):
            z=TOP+R-.0075+(j+.5)*h-(self.p[2]+cap[2])
            # Solve radial material in the oblique cap plane at this absolute belt height.
            A=1+(ax/ay)**2; B=ax*az*z/(ay*ay); C=z*z*(1+(az/ay)**2)
            def roots(radius):
                ds=B*B-A*(C-radius*radius)
                return None if ds<=0 else ((-B-math.sqrt(ds))/A,(-B+math.sqrt(ds))/A)
            outer=roots(R)
            if outer is None:continue
            inner=roots(RI)
            pairs=[outer] if inner is None else [(outer[0],inner[0]),(inner[1],outer[1])]
            for lo,hi in pairs:
                lo=max(lo,-.0845-self.p[0]-cap[0]);hi=min(hi,.0955-self.p[0]-cap[0])
                if hi<=lo:continue
                x=(lo+hi)/2
                r=cap+np.array([x,-(ax*x+az*z)/ay,z])
                points.append((r,(hi-lo)*h/abs(ay)))
        return points

    def moves(self,target,dt):
        self.belt+=min(cfg.belt_accel*dt,max(-cfg.belt_accel*dt,target-self.belt))
        force=np.array([0.,0.,-M*G]);torque=np.zeros(3);w=self.angular(self.l)
        for i,side in enumerate((-1,1)):
            points=self.footprint(side)
            area=sum(s for _,s in points)
            for r,area_cell in points:
                travel=1000*(side*(self.p[1]+r[1])-HALF)
                if travel< -self.inward:continue
                self.peak=max(self.peak,travel)
                if travel>self.outward and self.hard is None:self.hard=[self.time,side,travel]
                vel=self.v+np.cross(w,r)
                normal=max(0,self.preload+2*travel+37.52*side*vel[1])*area_cell/max(cfg.area_for_preload,area)
                if normal and self.first_belts[i] is None:self.first_belts[i]=self.time
                slip_x,slip_z=vel[0]-self.belt,vel[2]
                scale=-cfg.belt_mu*normal/max(cfg.slip_scale,math.hypot(slip_x,slip_z))
                f=np.array([scale*slip_x,-side*normal,scale*slip_z])
                self.peak_load=max(self.peak_load,np.linalg.norm(f))
                force+=f;torque+=np.cross(r,f)
        self.v+=force/M*dt;self.l+=torque*dt
        self.floor(dt)
        w=self.angular(self.l)
        self.p+=self.v*dt
        self.a+=dt*np.cross(w,self.a)
        self.a/=np.linalg.norm(self.a)
        self.time+=dt;self.low=min(self.low,self.p[0])
        self.peak_tilt=max(self.peak_tilt,abs(math.degrees(math.asin(self.a[2]))))
        if self.touch is not None and not -.100<=self.p[0]<=.165:self.loss="outside-flat-shelf"

    def floor(self,dt):
        # Resolve the two real extreme generators of a finite cylinder against the flat shelf.
        a=self.a;radial=-R*(UP-a*a[2])/max(1e-9,math.sqrt(1-a[2]*a[2]))
        contacts=[]
        for side in (-1,1):
            r=side*HALF*a+radial
            foot=self.p+r
            if not (-.100<=foot[0]<=EDGE and abs(foot[1])<=.175):continue
            gap=foot[2]-TOP
            velocity=self.v+np.cross(self.angular(self.l),r)
            if gap>max(.000001,-velocity[2]*dt):continue
            contacts.append((r,gap))
        if not contacts:return
        if self.touch is None:self.touch=[self.time,self.p.tolist(),self.v.tolist(),self.angular(self.l).tolist()]
        # Zero restitution, non-tensile two-point normal impulses. Nonzero yaw/roll
        # changes the two supports independently. Friction is bounded by each impulse.
        arm=np.array([r for r,g in contacts]);gaps=np.array([g for r,g in contacts])
        angular=self.angular(self.l)
        vel=self.v+np.cross(angular,arm)
        target=np.maximum(-gaps/dt,0)
        rates=vel[:,2]-target; count=len(arm)
        columns=np.cross(arm,UP)
        inv=columns/IP+(columns@a)[:,None]*a*(1/I-1/IP)
        mat=1/M+columns@inv.T
        normal=np.zeros(count)
        if count==2:
            n=np.linalg.solve(mat,-rates)
            if min(n)>=0:normal=n
            else:
                j=int(np.argmin(rates));normal[j]=max(0,-rates[j]/mat[j,j])
        else:normal[0]=max(0,-rates[0]/mat[0,0])
        if not any(normal):return
        self.v[2]+=sum(normal)/M
        self.l+=normal@columns
        ux=(self.v+np.cross(self.angular(self.l),arm))
        for r,n,v in zip(arm,normal,ux):
            speed=math.hypot(v[0],v[1])
            if speed<=1e-12:continue
            tangent=np.array([v[0],v[1],0.])/speed
            lever=np.cross(r,tangent)
            directional=1/M+float(lever@self.angular(lever))
            impulse=-min(cfg.shelf_mu*n,speed/directional)*tangent
            self.v+=impulse/M;self.l+=np.cross(r,impulse)
        self.max_pen=max(self.max_pen,float(-min(gaps)))

    def ports(self):
        # Cylinder at Y=0. The ray returns first opaque material across each aperture.
        hits=[]
        for port,height in ((.100,.004),(.040,.0175)):
            good=False
            # The transverse 12mm bore includes every x extremum. A 1mm y-grid is an
            # explicit discrete sensitivity; all four quarter-millimeter offsets are unknown.
            for dx in np.linspace(-.006,.006,13):
                yspan=math.sqrt(max(0,.006**2-dx*dx))
                for y in (-yspan,0.,yspan):
                    d=np.array([port+dx,y,TOP])-self.p
                    b=float(d[2]-self.a[2]*(d@self.a)); k=1-self.a[2]**2
                    c=float(d@d-(d@self.a)**2-R**2);dis=b*b-k*c
                    if dis<0:continue
                    t=(-b-math.sqrt(dis))/k
                    if -.0003<=t<=height and abs((d+t*UP)@self.a)<=HALF:good=True;break
                if good:break
            hits.append(good)
        return hits


def trial(proc,delta,mode,dt,phase,cache,fault=None,feed_side=0,relief=False,until_ms=None):
    pipe=Pipe(delta,feed_side,relief);epoch=100+phase
    proc.stdin.write("COUNTERFEED\n");proc.stdin.flush()
    trace=[];ticks=[];seed=None
    for tick in range(0,until_ms+21 if until_ms else 2700,20):
        entry,seat=pipe.ports() if tick>=epoch else (False,False)
        if fault=="dark-seat":seat=False
        bits=[tick,tick,1,1,int(fault!="undocked"),1,int(entry),int(seat)]
        proc.stdin.write(",".join(map(str,bits))+"\n");proc.stdin.flush()
        state,active,value,reason=proc.stdout.readline().strip().split(",")
        ticks.append([tick,state,entry,seat,float(value)])
        if state=="IN_BAND" and seed is None:seed=tick
        if state=="FAULT" or (seed is not None and tick>=(until_ms if until_ms else seed+240)) or pipe.hard or pipe.loss:break
        target=float(value)
        if tick<epoch:pipe.belt+=max(-cfg.belt_accel*.020,min(cfg.belt_accel*max(0,min(tick+20,epoch)-tick)/1000,target-pipe.belt))
        end=(tick+20-epoch)/1000
        while end-pipe.time>1e-11 and not pipe.hard and not pipe.loss:
            step=min(dt,end-pipe.time);pipe.moves(target,step)
            trace.append([epoch+pipe.time*1000,*pipe.p*1000,*pipe.a,*pipe.v,*pipe.angular(pipe.l)])
    path=cache/(mode+".npy");np.save(path,np.array(trace))
    return {"name":mode,"dt":dt,"feedSideMm":feed_side,"actualReturn":delta,"hardStop":pipe.hard,"firstBeltSeconds":pipe.first_belts,
            "sourceBeltPeakTravelMm":pipe.peak,"touchdown":pipe.touch,"peakAxisTiltDeg":pipe.peak_tilt,"maxShelfPenetrationMm":1000*pipe.max_pen,
            "minimumUMm":1000*pipe.low,"firstPortBandMs":seed,"lastPortState":state,"reason":reason,"loss":pipe.loss,
            "finalPositionMm":(1000*pipe.p).tolist(),"finalAxis":pipe.a.tolist(),"finalVelocityMps":pipe.v.tolist(),"finalAngularRadps":pipe.angular(pipe.l).tolist(),
            "trajectorySHA256":hashlib.sha256(path.read_bytes()).hexdigest(),"portTicks":ticks}


def run(cache,jdk,relief=False,probe=False,hold=False):
    cache.mkdir(parents=True,exist_ok=True)
    base=ROOT/"observable-acquisition";results=[]
    poses={r["mode"]:r["returnStationRobotDxDyYawMmDeg"] for r in json.loads((ROOT/"spatial-contact/match.json").read_bytes())["cases"] if r["pipe"]=="coral-A"}
    with tempfile.TemporaryDirectory() as tmp:
        tmp=Path(tmp);text=(base/"PortAcquisition.java").read_text().replace("speed = 0.45;","speed = 0.65;")
        if probe:
            anchor="        return new Output(state, state != State.FAULT, speed, reason);"
            if text.count(anchor)!=1:raise ValueError("Port controller changed")
            text=text.replace(anchor,"""        if (counterfeed && state == State.BRAKE && incomingSeatOnly && !incomingClear
                && previous - entered >= 160 && previous - entered < 480) {
            speed = -0.14;
        }
"""+anchor)
        (tmp/"PortAcquisition.java").write_text(text,encoding="utf8",newline="\n")
        subprocess.run([str(jdk/"bin/javac"),"-d",str(tmp),str(tmp/"PortAcquisition.java"),str(base/"PortConsole.java")],check=True)
        with subprocess.Popen([str(jdk/"bin/java"),"-cp",str(tmp),"org.reefscape.controls.PortConsole"],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1) as proc:
            corrected={"bias-"+k:r["stationRobotDxDyYawMmDeg"] for k,r in json.loads((HERE/"return.json").read_bytes())["cases"].items()}
            for name,delta in (corrected if relief else {"center":[0,0,0],**poses,**corrected}).items():
                for dt in ((.00025,) if hold else (.0005,.00025)):
                    label=("hold-" if hold else "probe-" if probe else "relief-" if relief else "")+name+"-"+str(round(dt*1e6))
                    r=trial(proc,delta,label,dt,19,cache,relief=relief,until_ms=5540 if hold else None)
                    results.append(r);print(label,r["lastPortState"],r["firstPortBandMs"],r["hardStop"],r["finalPositionMm"],flush=True)
            for name,fault,side in (() if hold else (("dark-seat","dark-seat",0),("undocked","undocked",0),("shifted-source",None,3.0))):
                results.append(trial(proc,corrected["bias-nominal"] if relief else poses["nominal"],("probe-" if probe else "relief-" if relief else "")+name,.0005,19,cache,fault,side,relief))
            proc.stdin.close();proc.wait()
    (HERE/("hold-feed.json" if hold else "probe-feed.json" if probe else "relief-feed.json" if relief else "feed.json")).write_text(json.dumps(results,indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("cache",type=Path);p.add_argument("jdk",type=Path)
    p.add_argument("--relief",action="store_true");p.add_argument("--probe",action="store_true");p.add_argument("--hold",action="store_true");a=p.parse_args();run(a.cache,a.jdk,a.relief or a.probe or a.hold,a.probe or a.hold,a.hold)
