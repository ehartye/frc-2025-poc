"""Finite distributed line contact between the tube's lower generator and the physical shelf."""
from dataclasses import dataclass
from functools import lru_cache
import math
import sys
from pathlib import Path

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"station-guide"))
from guide import GuidedPipe, feed


@dataclass(frozen=True)
class Shelf:
    stiffness_n_m: float = 400_000.
    dissipation_s_m: float = 4.
    friction: float = .10
    slip_speed_m_s: float = .02
    stations: int = 8


@lru_cache(maxsize=8)
def quadrature(count):
    return np.polynomial.legendre.leggauss(count)


def clip(low, high, base, slope, minimum, maximum):
    if abs(slope)<1e-14:
        return (low,high) if minimum<=base<=maximum else (1.,-1.)
    a,b=(minimum-base)/slope,(maximum-base)/slope
    return max(low,min(a,b)), min(high,max(a,b))


class CompliantPipe(GuidedPipe):
    def __init__(self, delta, feed_side, guide, shelf=Shelf()):
        super().__init__(delta,feed_side,guide)
        self.shelf=shelf
        self.peak_shelf_n=0.
        self.peak_shelf_compression=0.
        self.shelf_impulse_ns=0.
        self.last_normal_n=0.
        self.first_impact_speed=None
        self.record_load=False
        self.distributed_history_s=.010
        self.load_samples=[]
        self.distributed_load_samples=[]
        self.normal_dissipation_j=0.
        self.friction_dissipation_j=0.

    def floor(self,dt):
        self.last_normal_n=0.
        axis=self.a
        radial=-feed.R*(feed.UP-axis*axis[2])/max(1.e-12,math.sqrt(1-axis[2]**2))
        center=self.p+radial
        if center[2]-feed.HALF*abs(axis[2])>=feed.TOP:
            return
        low,high=-feed.HALF,feed.HALF
        for i,(minimum,maximum) in enumerate(((-.100,feed.EDGE),(-.175,.175),(-math.inf,feed.TOP))):
            low,high=clip(low,high,center[i],axis[i],minimum,maximum)
        if high<=low:return
        omega=self.angular(self.l)
        velocity=self.v+np.cross(omega,radial)
        slope=np.cross(omega,axis)
        cfg=self.shelf
        # Cut the no-tension boundary analytically even when one end rebounds while the other compresses.
        if cfg.dissipation_s_m:
            low,high=clip(low,high,velocity[2],slope[2],-math.inf,1/cfg.dissipation_s_m)
        if high<=low:return
        ends=np.array([low,high])
        compression=feed.TOP-center[2]-axis[2]*ends
        self.peak_shelf_compression=max(self.peak_shelf_compression,float(max(compression)))
        nodes,weights=quadrature(cfg.stations)
        x=(low+high)/2+(high-low)/2*nodes
        w=(high-low)/(4*feed.HALF)*weights
        arms=radial+x[:,None]*axis
        rates=velocity+x[:,None]*slope
        delta=np.maximum(0.,feed.TOP-center[2]-axis[2]*x)
        normal=w*cfg.stiffness_n_m*delta*np.maximum(0.,1-cfg.dissipation_s_m*rates[:,2])
        vt=rates[:,:2];speed=np.linalg.norm(vt,axis=1)
        tangent=-cfg.friction*normal[:,None]*vt/np.maximum(cfg.slip_speed_m_s,speed)[:,None]
        f=np.column_stack((tangent,normal))
        fn=float(np.sum(normal))
        if not fn:return
        if self.touch is None:
            self.touch=[self.time,self.p.tolist(),self.v.tolist(),omega.tolist()]
            self.first_impact_speed=float(min(rates[:,2]))
        if self.record_load:
            location=self.p+normal@arms/fn
            self.load_samples.append([self.time,*(-f.sum(axis=0)),*(1000*location),1000*(high-low)])
            if self.first_impact_speed is None or self.time-self.touch[0] < self.distributed_history_s:
                self.distributed_load_samples.extend([self.time,i,*xyz,*reaction] for i,(xyz,reaction) in enumerate(zip(1000*(self.p+arms),-f)))
        self.last_normal_n=fn
        self.peak_shelf_n=max(self.peak_shelf_n,fn)
        self.shelf_impulse_ns+=dt*fn
        self.normal_dissipation_j+=float(dt*np.sum(w*cfg.stiffness_n_m*delta*cfg.dissipation_s_m*rates[:,2]**2))
        self.friction_dissipation_j-=float(dt*np.sum(tangent*vt))
        self.max_pen=max(self.max_pen,self.peak_shelf_compression)
        self.v+=dt*f.sum(axis=0)/feed.M
        self.l+=dt*np.cross(arms,f).sum(axis=0)


def analytic_impact(speed, cfg):
    """Flat symmetric gravity-free drop. Return exact peak compression and first separation speed."""
    m,k,a=feed.M,cfg.stiffness_n_m,cfg.dissipation_s_m
    if a==0:
        return speed*math.sqrt(m/k),speed
    def potential(x):
        return x-math.log1p(x)
    target=potential(a*speed)
    lo,hi=-1.,0.
    for _ in range(60):
        mid=(lo+hi)/2
        if mid<=-1. or potential(mid)>target:lo=mid
        else:hi=mid
    return math.sqrt(2*m*target/k)/a, -(lo+hi)/(2*a)
