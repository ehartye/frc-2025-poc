"""Screen gravity seating and lateral friction on the two actual registered drive-and-turn routes."""
import argparse
import json
import math
import sys
from pathlib import Path
from model import ROOT, G

sys.path.insert(0,str(ROOT/"full-match"))
from match import corrected


def loads(sector, lean_deg=0., dt=.01):
    route=corrected().RegisteredMission(sector)
    def pose(t):
        _,x,y,h,top,stroke=route.pose(t)
        return (x+(stroke-.03285)*math.cos(h),y+(stroke-.03285)*math.sin(h),top+.063,h)
    peak_side=peak_long=peak_yaw=peak_mu=0.
    min_support=1.
    min_load=1.
    a=math.radians(lean_deg)
    rear,front=-math.pi/2-a,math.atan2(.044,.245)-a
    n1=(-math.sin(rear),math.cos(rear))
    n2=(-math.sin(front),math.cos(front))
    det=n1[0]*n2[1]-n2[0]*n1[1]
    for i in range(1,round((route.terminal_time-1.1)/dt)):
        t=1.1+i*dt
        p,q,r=pose(t-dt),pose(t),pose(t+dt)
        ax,ay,az,alpha=[(l-2*m+n)/dt**2 for l,m,n in zip(p,q,r)]
        co,si=math.cos(q[-1]),math.sin(q[-1])
        along,side=co*ax+si*ay,-si*ax+co*ay
        g=G+az
        support_rear=(along*n2[1]-g*n2[0])/det
        support_front=(n1[0]*g-n1[1]*along)/det
        total=support_rear+support_front
        min_support=min(min_support,support_rear/total,support_front/total)
        # Per kg; longest/widest screened pipe, 10 mm offcenter plus 2 mm mounting shift.
        iz=(.307975**2)/12+(.0579**2+.0524**2)/4
        unequal=(.012+.064*abs(side)/g)/.085
        min_load=min(min_load,(1-unequal)/2)
        need=math.hypot(abs(side),(abs(iz*alpha)+.058*abs(side))/.085)
        demand=need/(max(1e-9,total*(1-unequal)))
        peak_mu=max(peak_mu,demand)
        peak_side=max(peak_side,abs(side));peak_long=max(peak_long,abs(along));peak_yaw=max(peak_yaw,abs(alpha))
    return dict(sectorM=sector,leanDeg=lean_deg,dtS=dt,maxSideMps2=peak_side,maxLongMps2=peak_long,
                maxYawRadS2=peak_yaw,minimumSideRailShare=min_load,minimumFrontRearShare=min_support,
                minimumSufficientFriction=peak_mu)


if __name__=="__main__":
    a=argparse.ArgumentParser(description=__doc__);a.add_argument("--output",type=Path,required=True);p=a.parse_args()
    p.output.write_text(json.dumps([loads(s,h) for s in (-.18,.18) for h in (-1,0,1)],indent=2)+"\n",encoding="utf-8",newline="\n")
