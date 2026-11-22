"""Matched transverse pipe, actual registered station lip and finite concept rails."""
import json
import math
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'experiments/six-wheel/scripts'))
from receiver_contact import departure,flight_center

M,R,I=.816466266,.05715,.816466266*(.05715**2+.0508**2)/2
G=9.80665

def capture(name,case,geom):
    source=json.loads((ROOT/'experiments/six-wheel/sourced-match/source-545.json').read_bytes())['feed']
    adverse=case=='adverse'
    # Station speed comes from the project feed study. Adverse speed reduction
    # and docking error are deliberately applied to every architecture.
    speed=source['lip_mps']*(.85 if adverse else 1.)
    lip=departure(speed);wx,wz=flight_center(lip,0)
    sx=wx-(.444525+(.020 if adverse else 0.));sz=wz
    vx=lip['velocity_x_mps'];vz=lip['velocity_z_mps'];omega=speed/R
    rail=geom['receiveProfileMm'];points=[(x/1000,z/1000) for x,z in rail]
    mu=.15 if adverse else .35
    dt=.0002;contact_started=None;settled_time=None;dwell=0.;peak=0.;max_compression=0.
    carry_since=2.8
    for i in range(int(4.5/dt)):
        t=i*dt;fx=0.;fz=-M*G;torque=0.;engaged=False
        if carry_since<t<carry_since+.7:fx-=M*(1.5 if adverse else .9)
        if carry_since+.9<t<carry_since+1.6:fx+=M*(1.5 if adverse else .9)
        for (x,z),(u,v) in zip(points,points[1:]):
            dx,dz=u-x,v-z
            q=min(1.,max(0.,((sx-x)*dx+(sz-z)*dz)/(dx*dx+dz*dz)))
            cx,cz=x+q*dx,z+q*dz
            dist=math.hypot(sx-cx,sz-cz)
            if dist>=R or dist<1e-9:continue
            nx,nz=(sx-cx)/dist,(sz-cz)/dist;tx,tz=nz,-nx
            vn=vx*nx+vz*nz;vt=vx*tx+vz*tz-omega*R
            fn=max(0.,60000*(R-dist)-1.2*math.sqrt(60000*M)*vn)
            ft=-mu*fn*math.tanh(vt/.005)
            fx+=fn*nx+ft*tx;fz+=fn*nz+ft*tz;torque-=ft*R
            peak=max(peak,fn);max_compression=max(max_compression,R-dist)
            if fn>0:engaged=True
        if engaged and contact_started is None:contact_started=t
        lateral=abs(.025 if adverse else .003)+.1508125*math.cos(.045 if adverse else 0)+R*math.sin(.045 if adverse else 0)
        covered=lateral<=geom['mouthHalfWidthMm']/1000
        dwell=dwell+dt if engaged and covered and math.hypot(vx,vz)<.035 and abs(omega)<.7 else 0.
        if dwell>.12 and settled_time is None:settled_time=t
        vx+=dt*fx/M;vz+=dt*fz/M;omega+=dt*torque/I;sx+=dt*vx;sz+=dt*vz
        if sz<.12 or abs(sx)>1.1:break
    return dict(feedToLipS=source['free_drop_s']+source['ramp_time_s'],firstContactS=contact_started,stationarySeatS=settled_time,heldAfterAcceleration=bool(settled_time and covered and sz>.20 and t>4.49 and engaged),
                lastS=t,finalCenterMm=[sx*1000,sz*1000],peakNormalN=peak,maxCompressionMm=1000*max_compression,
                transverseEnvelopeMarginMm=geom['mouthHalfWidthMm']-1000*lateral,
                scope='Finite longitudinal rail cross-section and transverse mouth test. Assumes matched rail support. Off-axis impulse, actual sensing, actuator coupling, whole-robot transit and independent score still require an integrated test.')
