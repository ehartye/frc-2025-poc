"""Test finite, face-lined station guide surfaces against the actual rotating annular pipe."""
from dataclasses import dataclass
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT/"return-station"))
import feed


@dataclass(frozen=True)
class Guide:
    throat_mm: float = .20
    stiffness_n_mm: float = 5
    damping_n_s_m: float = 15
    friction: float = .1
    sectors: int = 32
    radial_bins: int = 2
    x_width_mm: float = 80
    upper_mm: float = 900
    x_offset_mm: float = 0


def x_center(z):
    # Follow the original ballistic approach in the receiver frame, all dimensions in metres.
    return .145 + .46*(z-.625)


def gap(z, guide):
    return guide.throat_mm/1000 + .038*np.maximum(z-.770, 0.)


def annular_points(axis, sectors, radial_bins=2):
    first = np.array([1., 0., 0.])
    first -= axis*float(first@axis)
    first /= np.linalg.norm(first)
    second = np.cross(axis, first)
    angle = (np.arange(sectors)+.5)*2*np.pi/sectors
    radii = np.sqrt(feed.RI**2+(np.arange(radial_bins)+.5)*(feed.R**2-feed.RI**2)/radial_bins)
    direction = np.cos(angle)[:,None]*first+np.sin(angle)[:,None]*second
    return (radii[:,None,None]*direction).reshape(-1, 3)


class GuidedPipe(feed.Pipe):
    def __init__(self, delta, feed_side, guide):
        super().__init__(delta, feed_side, relief=True)
        self.guide = guide
        self.peak_guiding_n = 0.
        self.max_local_compression_mm = 0.
        self.first_guide = None
        self.last_guide = None

    def moves(self, target, dt):
        g = self.guide
        vertical_radius = feed.HALF*abs(self.a[2])+feed.R*math.sqrt(max(0.,1-self.a[2]**2))
        if g and self.p[2]+vertical_radius >= .625 and self.p[2]-vertical_radius <= g.upper_mm/1000:
            ring = annular_points(self.a, g.sectors, g.radial_bins)
            for side in (-1,1):
                arms = ring+side*feed.HALF*self.a
                points = self.p+arms
                x,y,z = points.T
                taper = np.where(z>.770, .038, 0.)
                norm = np.sqrt(1+taper*taper)
                penetration = (side*y-feed.HALF-gap(z,g))/norm
                inside = ((z>=.625)&(z<=g.upper_mm/1000)&(abs(x-x_center(z)-g.x_offset_mm/1000)<.0005*g.x_width_mm)&(penetration>0))
                if not np.any(inside):
                    continue
                r = arms[inside]
                n = np.column_stack((np.zeros(len(r)), side/norm[inside], -taper[inside]/norm[inside]))
                velocity = self.v + np.cross(self.angular(self.l), r)
                closing = np.sum(velocity*n,axis=1)
                compression = penetration[inside]
                pressure = np.maximum(0, g.stiffness_n_mm*1000*compression + g.damping_n_s_m*closing)/len(ring)
                tangent = velocity-closing[:,None]*n
                f = -pressure[:,None]*(n+g.friction*tangent/np.maximum(.02,np.linalg.norm(tangent,axis=1))[:,None])
                force = np.sum(f,axis=0)
                self.l += np.sum(np.cross(r,f),axis=0)*dt
                self.v += force/feed.M*dt
                if self.first_guide is None:self.first_guide = self.time
                self.last_guide = self.time
                self.peak_guiding_n = max(self.peak_guiding_n,float(sum(pressure)))
                self.max_local_compression_mm = max(self.max_local_compression_mm,1000*float(max(compression)))
        super().moves(target,dt)
