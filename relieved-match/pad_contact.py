"""Rigid contact with the three as-drawn 0.8 mm sensor laminates on the extended shelf."""
import math
import numpy as np
from feed import Pipe,TOP,HALF,R,M,UP


class SensorShelf(Pipe):
    def __init__(self,delta,relief=True,pad_friction=.15):
        super().__init__(delta,relief=relief)
        self.pad_friction=pad_friction
        self.loads=np.zeros(3)
        self.trace=[]
        self.max_pad_indent=0.
        self.edge_impulse=0.
        self.edge_contacts=[]

    def edge(self,dt):
        if self.p[0]<.163 or self.p[0]>.21:return
        axis=self.a
        for s in (-HALF,0.,HALF):
            c=self.p+s*axis
            if c[0]<=.165:continue
            d=c-np.array([.165,c[1],.500])
            direction=d/np.linalg.norm(d)
            along=axis@direction
            r=s*axis-R*(direction-along*axis)/math.sqrt(1-along**2)
            gap=(self.p+r-np.array([.165,c[1],.5]))@direction
            vn=(self.v+np.cross(self.angular(self.l),r))@direction
            if gap>max(1e-8,-vn*dt):continue
            arm=np.cross(r,direction)
            impulse=max(0.,(-vn+max(0.,-gap/dt))/(1/M+arm@self.angular(arm)))
            if impulse:
                self.v+=impulse*direction/M;self.l+=impulse*arm
                tangent=self.v+np.cross(self.angular(self.l),r)
                tangent-=direction*(tangent@direction)
                rate=np.linalg.norm(tangent)
                if rate>1e-12:
                    tangent/=rate;arm=np.cross(r,tangent)
                    tangential=-min(.10*impulse,rate/(1/M+arm@self.angular(arm)))
                    self.v+=tangential*tangent/M;self.l+=tangential*arm
                self.edge_contacts.append(self.time)
                self.edge_impulse+=impulse

    def floor(self,dt):
        super().floor(dt)
        if getattr(self,"resolve_edge",False):self.edge(dt)
        self.loads.fill(0)
        if self.p[0]<.105 or self.p[0]>.181 or self.p[2]>TOP+R+.003:
            return
        candidates=[]
        a=self.a
        for index,mid in enumerate((-.070,0.,.070)):
            for dy in (-.0025,0.,.0025):
                y=mid+dy
                # Union of a 30 x 5 mm tail and 7.2 mm circular head. Three transverse
                # sections per sensor resolve small body tilts; smaller step and opposite
                # section ordering are separate convergence controls.
                xmax=.155+math.sqrt(.0036**2-dy**2)
                lo,hi=.125,xmax
                station=(y-self.p[1])/a[1]
                if abs(station)>HALF:continue
                for _ in range(2):
                    c=self.p+station*a
                    x=np.clip(c[0],lo,hi)
                    dx,dz=c[0]-x,c[2]-(TOP+.0008)
                    d=math.hypot(dx,dz); n=np.array([dx/d,0.,dz/d])
                    dot=a@n
                    h=math.sqrt(max(1e-12,1-dot*dot))
                    radial=-R*(n-dot*a)/h
                    station=(y-self.p[1]-radial[1])/a[1]
                lever=station*a+radial
                point=self.p+lever
                gap=(point-np.array([x,y,TOP+.0008]))@n
                speed=(self.v+np.cross(self.angular(self.l),lever))@n
                if gap>max(1e-6,-speed*dt):continue
                arm=np.cross(lever,n)
                denom=1/M+arm@self.angular(arm)
                active=(x-.155)**2+dy*dy<=(.0028)**2
                candidates.append((index,lever,n,arm,denom,max(0.,-gap/dt),active))
                self.max_pad_indent=max(self.max_pad_indent,-gap)
        if not candidates:return
        fractions=np.zeros(len(candidates))
        # Separable normal complementarity impulses; reverse each iteration to avoid always
        # putting the first pad's longitudinal edge ahead of its symmetrical partner.
        for passno in range(6):
            order=range(len(candidates)) if passno%2==0 else reversed(range(len(candidates)))
            for j in order:
                k,r,n,arm,denom,target,active=candidates[j]
                vel=self.v+np.cross(self.angular(self.l),r)
                updated=max(0.,fractions[j]+(target-vel@n)/denom)
                inc=updated-fractions[j];fractions[j]=updated
                self.v+=inc*n/M;self.l+=inc*arm
        for j,(k,r,n,arm,denom,target,active) in enumerate(candidates):
            normal=fractions[j]
            if active:self.loads[k]+=normal/dt
            vel=self.v+np.cross(self.angular(self.l),r)
            tangent=vel-(n@vel)*n
            speed=np.linalg.norm(tangent)
            if speed>1e-12 and normal:
                tangent/=speed;arm=np.cross(r,tangent)
                impulse=-min(self.pad_friction*normal,speed/(1/M+arm@self.angular(arm)))
                self.v+=impulse*tangent/M;self.l+=impulse*arm
        self.trace.append([self.time,*self.loads])
