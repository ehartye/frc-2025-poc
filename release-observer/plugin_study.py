"""Write original project fixtures consumable by the installed Agent FRC sensor:review CLI."""
import argparse
import hashlib
import json
from math import cos,sin,hypot
from pathlib import Path
import sys
from stereo import endpoints,image_pairs,clearance,RADIUS,LENGTH

ROOT=Path(__file__).resolve().parents[1]


def write(folder,name,value):
    path=folder/name
    path.write_bytes((json.dumps(value,indent=2)+"\n").encode())
    return {"file":name,"sha256":hashlib.sha256(path.read_bytes()).hexdigest()}


def create(folder,mission):
    folder.mkdir(parents=True,exist_ok=True)
    ids={"attempt":"synthetic-station-1","piece":mission["piece_id"]}
    tags={"frame":"translating-shelf-R6","clock":"station-mission"}
    phases=("approach","seated","early-exit","cleared")
    expected={p:dict(seat=a,entry=b) for p,(a,b) in zip(phases,((False,True),(True,True),(False,False),(False,False)))}
    model=dict(**tags,units="m",maxAgeS=.02,clearanceM=.001,expected=expected,
               support=dict(axis="x",direction=1,end=.165,material=[
                   dict(kind="box",center=[.0325,0,-.0046],halfSize=[.1325,.210,.0054])]),
               pieceSolids=[dict(kind="tube",center=[0,0,0],axis="y",halfLength=LENGTH/2000,
                                 radius=RADIUS/1000,bore=.049)],
               obstacles=[],sensors=[dict(id=n,mode="reflective",origin=[x,0,-.012],axis="z",direction=1,range=.15)
                                     for n,x in (("seat",.040),("entry",.100))])
    mr=write(folder,"layout.json",model)
    end=mission["trace"][-1]
    ends=endpoints(end); position=[(a+b)/2000 for a,b in zip(*ends)]
    position[0]-=end["slide_m"]; position[2]-=end["shelf_top_m"]
    yaw=end["yaw_rad"]-end["chassis_xy_heading"][2]
    times=[0.,1.,2.,end["time_s"]]
    samples=[]
    for phase,t,pos in zip(phases,times,([.14,0,.05715],[.08,0,.05715],[.16,0,.05715],position)):
        row=dict(**ids,phase=phase,timeS=t,translation=pos,
                 readings={n:dict(**ids,timeS=t,occupied=r) for n,r in expected[phase].items()})
        if phase=="cleared": row["quaternion"]=[cos(yaw/2),0,0,sin(yaw/2)]
        samples.append(row)
    tr=write(folder,"trace.json",dict(**ids,**tags,modelSha256=mr["sha256"],samples=samples))
    gap=clearance(image_pairs(ends),1000*end["shelf_front_robot_u_m"],1000*end["shelf_top_m"]+.8,.10,1,.5)
    dx=max(0,min(p[0] for p in ends)/1000-end["shelf_front_robot_u_m"])
    dz=max(0,min(p[2] for p in ends)/1000-end["shelf_top_m"]-.0008)
    length=hypot(dx,dz)
    witness=dict(**ids,**tags,units="m",timeS=end["time_s"],gapM=gap/1000,
                 method="independent-geometry",source="bounded synthetic stereo",
                 modelSha256=mr["sha256"],traceSha256=tr["sha256"])
    if length: witness["separatingDirection"]=[dx/length,0,dz/length]
    wr=write(folder,"witness.json",witness)
    write(folder,"request.json",dict(model=mr,trace=tr,witness=wr))


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("output",type=Path);p.add_argument("--mission-root",type=Path,default=ROOT)
    a=p.parse_args();sys.path.insert(0,str(a.mission_root.resolve()/"scripts"))
    import station_l1_mission as producer
    for case in ("reference","slow-release"):
        create(a.output/case,producer.run(case,.001,sample_every_step=True))
