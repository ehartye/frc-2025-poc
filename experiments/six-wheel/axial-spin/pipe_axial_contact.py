"""Centered transverse tube and finite, opposed end-face belts in the robot forward/up plane."""
import math
from dataclasses import dataclass

R=.05715
RI=.0508
HALF=.1508125
M=.816466266
K=(1+(RI/R)**2)/2
I=K*M*R**2
G=9.80665
BELT_BACK=-.0845
BELT_FRONT=.0955
BELT_HALF_HEIGHT=.0075


def clip(a,b):
    return min(b,max(-b,a))


@dataclass(frozen=True)
class Contact:
    ramp_mu:float=.40
    belt_mu:float=.40
    shelf_mu:float=.10
    preload:float=10.
    slip_scale:float=.02
    drive:str="force-following"
    release_limit:float=1.05
    belt_accel:float=26.25
    slices:int=5
    area_for_preload:float=15e-6


def ramp(source,mu):
    """Normal restitution zero; tangential impact, finite sliding and optional rolling."""
    angle=math.radians(35)
    vdown=source["fall_speed_mps"]
    tangent=vdown*math.sin(angle)
    impulse=min(mu*M*vdown*math.cos(angle),M*tangent*K/(1+K))
    speed=tangent-impulse/M
    roll=impulse*R/I
    accel=G*(math.sin(angle)-mu*math.cos(angle))
    alpha=mu*M*G*math.cos(angle)*R/I
    slip=speed-roll*R
    to_roll=max(0,slip/(alpha*R-accel)) if alpha*R>accel else math.inf
    if abs(slip)<1e-10 and mu+1e-12>=K/(1+K)*math.tan(angle):
        to_roll=0
    dist=source["run_m"]
    sliding=math.inf if math.isinf(to_roll) else speed*to_roll+.5*accel*to_roll**2
    if sliding>=dist:
        t=2*dist/(speed+math.sqrt(speed**2+2*accel*dist))
        return dict(speed=speed+accel*t,spin=roll+alpha*t,time=t,mode="sliding",impact_impulse_Ns=impulse)
    speed+=accel*to_roll
    roll_accel=G*math.sin(angle)/(1+K)
    final=math.sqrt(speed*speed+2*roll_accel*(dist-sliding))
    return dict(speed=final,spin=final/R,time=to_roll+(final-speed)/roll_accel,mode="rolling",impact_impulse_Ns=impulse)


def patches(u,zrel,config):
    """Midpoint quadrature over belt height; exact annular horizontal strips, area-weighted."""
    height=2*BELT_HALF_HEIGHT/config.slices
    result=[]
    for j in range(config.slices):
        z=-BELT_HALF_HEIGHT+(j+.5)*height-zrel
        if abs(z)>=R:
            continue
        outer=math.sqrt(R*R-z*z)
        inner=math.sqrt(max(0,RI*RI-z*z))
        for lo,hi in ((-outer,-inner),(inner,outer)):
            low=max(lo,BELT_BACK-u)
            high=min(hi,BELT_FRONT-u)
            if low<high:
                result.append(((low+high)/2,z,height*(high-low)))
    return result


class AxialContact:
    def __init__(self,config,spin=0):
        self.cfg=config
        self.spin=spin
        self.phase=""
        self.samples=[]
        self.now=0.
        self.phases={}
        self.previous=None
        self.start=None
        self.points=([],[])
        self.belt_work=0.
        self.dissipation=0.
        self.min_shelf_normal=1e6
        self.max_force_fraction=0.
        self.lifted=False
        self.angular_impulse=0.
        self.belt_speed=[0.,0.]
        self.max_belt_speed=0.
        self.max_belt_accel=0.
        self.max_belt_load=0.

    def enter_phase(self,phase,time):
        self.now=time
        if phase!=self.phase:
            self.phases[phase]=[time,self.spin]
            self.phase=phase

    def normals(self,u,v,yaw,phase,time,zrel=0):
        cfg=self.cfg
        gain=HALF*1000*(math.cos(yaw)-1)+R*1000*abs(math.sin(yaw))
        offsets=(gain-1000*v,gain+1000*v)
        rate=(0,0) if self.previous is None else tuple(
            (a-b)/1000/(time-self.previous[0]) for a,b in zip(offsets,self.previous[1]))
        self.previous=time,offsets
        self.points=tuple(patches(u+s*HALF*math.sin(yaw),zrel,cfg) for s in (-1,1))
        n=[max(0,cfg.preload+2*off+37.52*vel)*min(1,sum(p[2] for p in pts)/cfg.area_for_preload)
           for pts,off,vel in zip(self.points,offsets,rate)]
        limit=1.05
        if phase=="place":
            if self.start is None:self.start=time
            limit=self.cfg.release_limit*min(1,(time-self.start)/.04)
        return *n,cfg.belt_mu,limit,cfg.shelf_mu

    def forces(self,speed,vertical,normals,requests,limit,dt,required_normal):
        # Requests set achievable belt surface speed. Both friction components share a circular cap.
        fx=[0.,0.]; vertical_sum=0.; moment=0.
        for side,(pts,n,request) in enumerate(zip(self.points,normals,requests)):
            capacity=self.cfg.belt_mu*n
            area=sum(p[2] for p in pts)
            if not area or not capacity:
                self.belt_speed[side]+=clip(-self.belt_speed[side],self.cfg.belt_accel*dt)
                continue
            if self.cfg.drive=="velocity-command":
                delta=2*request/(M*20.)
            else:
                delta=self.cfg.slip_scale*request/capacity
            desired=clip(speed+delta,limit)
            increment=clip(desired-self.belt_speed[side],self.cfg.belt_accel*dt)
            belt=self.belt_speed[side]+increment
            self.belt_speed[side]=belt
            self.max_belt_speed=max(self.max_belt_speed,abs(belt))
            self.max_belt_accel=max(self.max_belt_accel,abs(increment)/dt)
            for x,z,part_area in pts:
                slip_x=speed-self.spin*z-belt
                slip_z=vertical+self.spin*x
                scale=capacity*(part_area/area)/max(self.cfg.slip_scale,math.hypot(slip_x,slip_z))
                tx,tz=-scale*slip_x,-scale*slip_z
                fx[side]+=tx;vertical_sum+=tz;moment+=x*tz-z*tx
                self.dissipation+=(-tx*slip_x-tz*slip_z)*dt
                self.belt_work+=tx*belt*dt
                self.max_force_fraction=max(self.max_force_fraction,math.hypot(tx,tz)/(capacity*part_area/area))
        self.max_belt_load=max(self.max_belt_load,*map(abs,fx))
        floor=0.
        if required_normal is not None:
            support=required_normal-vertical_sum
            self.min_shelf_normal=min(self.min_shelf_normal,support)
            if support < -1e-6:
                self.lifted=True
            slip=speed+self.spin*R
            floor=-self.cfg.shelf_mu*max(0,support)*clip(slip/self.cfg.slip_scale,1.)
            moment+=R*floor
            self.dissipation-=floor*slip*dt
        self.spin+=moment*dt/I
        if self.phase=="place" and (not self.samples or self.now-self.samples[-1][0]>=.00095):
            self.samples.append([self.now,self.spin,speed])
        self.angular_impulse+=moment*dt
        return fx, floor, vertical_sum

    def supported(self,u,speed,required,n_a,n_b,f_a,f_b,mu,limit,shelf_mu,dt):
        fx, floor, _=self.forces(speed,0,(n_a,n_b),(f_a,f_b),limit,dt,required)
        return fx[0],fx[1],floor
