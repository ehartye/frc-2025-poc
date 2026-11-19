"""Clearance along the reached chassis samples including wheel motion, stock and the other five robots."""
import math

import numpy as np

from drive import TRACK,route


def polygon_separation(x,y,h,poly):
    vx,vy=np.asarray(poly).T
    c,s=np.cos(h),np.sin(h)
    dx=vx[None,:]-x[:,None];dy=vy[None,:]-y[:,None]
    u=dx*c[:,None]+dy*s[:,None]
    v=-dx*s[:,None]+dy*c[:,None]
    gap=np.maximum.reduce([u.min(axis=1)-.425,-u.max(axis=1)-.425,
                           v.min(axis=1)-.425,-v.max(axis=1)-.425])
    for a,b in zip(poly,[*poly[1:],poly[0]]):
        norm=math.dist(a,b)
        if not norm:continue
        nx,ny=(a[1]-b[1])/norm,(b[0]-a[0])/norm
        projection=vx*nx+vy*ny
        center=x*nx+y*ny
        radius=.425*(abs(c*nx+s*ny)+abs(-s*nx+c*ny))
        gap=np.maximum.reduce([gap,projection.min()-center-radius,center-projection.max()-radius])
    return gap


def screen(f,time,pose,stock,team,stop_wall):
    x,y,h=pose.T
    max_z=1066.8
    # Cover the entire interval with a bound on center translation and all body points.
    # The paths are generated from finite wheel velocities with <=1.2 m/s² acceleration.
    dtime=np.diff(time)
    if np.min(dtime)<0 or np.max(dtime)>.011:raise ValueError("Discontinuous route clock")
    arc=np.linalg.norm(np.diff(pose[:,:2],axis=0),axis=1)+np.sqrt(2)*.425*abs(np.diff(np.unwrap(h)))
    pad=np.r_[arc+.0005,0] # one-sided bound to the next state, incl within-step velocity variation
    constraints={}
    for name,poly in f.obstacles():
        constraints[name]=float(np.min(polygon_separation(x,y,h,poly)-pad))*1000
    boundary=[("blue wall",[0,0],0),("red wall",[f.PROFILE["field"]["length"]*f.IN,0],180),
              ("lower rail",[0,0],90),("upper rail",[0,f.PROFILE["field"]["width"]*f.IN],-90)]
    for item in f.PROFILE["rehearsal"]["planes"]:
        spot=np.array(item["at"])*f.IN
        if "tag 2" in item["name"]:spot=f.add(f.STATION,f.rot((f.station["bumperFaceRobotwardM"],0),f.NORMAL))
        boundary.append((item["name"],spot,item["inwardDeg"]))
    for name,spot,angle in boundary:
        a=math.radians(angle)
        rad=.425*(abs(np.cos(h-a))+abs(np.sin(h-a)))
        dist=(x-spot[0])*math.cos(a)+(y-spot[1])*math.sin(a)-rad-pad
        constraints[name]=float(min(dist))*1000
    stock_gaps={}
    for key,obj in stock.items():
        nx,up,nz=obj["native_center_mm"]
        wx,wy=f.field_from_native(nx/25.4,nz/25.4)
        dx=1000*(wx-x);dy=1000*(wy-y)
        sx=np.maximum(0,abs(dx*np.cos(h)+dy*np.sin(h))-425)
        sy=np.maximum(0,abs(-dx*np.sin(h)+dy*np.cos(h))-425)
        horizontal=np.hypot(sx,sy)
        if obj["group"]=="floor-pipe":
            b=obj["native_bounds_mm"];radius=max(b[3]-b[0],b[5]-b[2])/2
            d=horizontal-radius
        else:
            d=np.hypot(horizontal,max(0,up-max_z,-up))-209.55
        stock_gaps[key]=float(min(d-1000*pad))
    traffic=route.route.field.traffic
    peers={}
    for key,program in team.items():
        if key=="red2":continue
        lower=math.inf
        for i,t in enumerate(time):
            p,w,_=program.at(max(0,t-18))
            step=dtime[i] if i<len(dtime) else 0
            extra=step*traffic.motion_bound(w)+4*step**2 if t>=18 else 0.
            lower=min(lower,traffic.separation(pose[i],p)-pad[i]-extra)
        peers[key]=lower*1000
    return {"barriersMm":constraints,"stockMm":stock_gaps,"peerMm":peers,
            "minimumBarrierMm":min(constraints.values()),"minimumStockMm":min(stock_gaps.values()),
            "minimumPeerMm":min(peers.values()),"throughWallS":stop_wall}


def parked_peers(team,pose,from_wall):
    traffic=route.route.field.traffic
    bounds={}
    # Propagate the finite programmed peers through the remaining TELEOP. The
    # focal robot is held stationary after the reached fault stop.
    for key,program in team.items():
        if key=="red2":continue
        lower=math.inf
        for wall in np.arange(max(18,from_wall),153,.02):
            p,w,_=program.at(wall-18)
            lower=min(lower,traffic.separation(pose,p)-.02*traffic.motion_bound(w)-4*.02**2)
        bounds[key]=1000*lower
    return bounds


def join(f,r,t,truth,local):
    # Switch from field units to station-frame mm only inside the visual controller.
    rows=np.array(local)
    a=f.NORMAL
    c,s=math.cos(a),math.sin(a)
    wx=f.DOCK[0]+(c*rows[:,1]-s*rows[:,2])/1000
    wy=f.DOCK[1]+(s*rows[:,1]+c*rows[:,2])/1000
    world=np.c_[wx,wy,a+math.pi+rows[:,3]]
    if math.dist(world[0,:2],truth[-1,:2])>.008:raise ValueError("Chassis discontinuity at handoff")
    times=np.r_[t,rows[1:,0]]
    pose=np.vstack((truth,world[1:]))
    # Continue the stationary forecast only as far as the scheduled feed.
    hold=np.arange(times[-1]+.01,21.10001,.01)
    return np.r_[times,hold],np.vstack((pose,np.tile(pose[-1],(len(hold),1))))
