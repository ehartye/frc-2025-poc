"""Check connector withdrawal, split-clamp mounting and released-frame lift against the registered robot."""
import argparse
import json
from pathlib import Path
from connector_fit import HERE,ROOT, cad,bounds,conflicts
from support import design
from camera_parts import box, cylinder
from routes import read,values
import numpy as np

def fit_service(boxes_path):
    hosts=read(boxes_path)["relative_sweep_bounds_mm"]
    shop=ROOT/"camera-datum-shop"
    kit=cad.read_named(shop/"camera-shelf.step",[{"id":k} for k in read(shop/"native.json")["parts"]])
    kit={"roof-"+k:v for k,v in kit.items() if k!="receiver-R6-drilled-for-camera"}
    service,_,pads=design()
    shop_hits={};moving=[];lift=[]
    for k,shape in service.items():
        b=list(bounds(shape));up=[*b[:5],b[5]+535]
        for n,r in hosts.items():
            if cad.occupied(b,r):
                if n in kit:
                    v=cad.intersection(shape,kit[n])
                    if v>.002:shop_hits[k+"/"+n]=v
                else:moving.append([k,n])
            if not n.startswith("roof-") and cad.occupied(up,r):lift.append([k,n])
    mouth=read(HERE/"interfaces.json")["disconnects"]
    data=mouth["data-J1"]["mouth_mm"]
    power=mouth["power-J3"]["mouth_mm"]
    # Manufacturer toleranced body gauges – travel includes 14 mm release plus 0.5 mm axial placement allowance.
    # Locking screws remain unused. Their exposed length is a go gauge, absent from the drawing.
    def removal(xyz,width,height,boot):
        px,y,z=xyz
        return box(px-boot-14.8,y-width/2,z-height/2,boot+15.0,width,height)
    gauges={"data-boot":removal(data,7.,20.4,32.25),"power-boot":removal(power,7.6,12.6,25.5)}
    for side in (-1,1):
        gauges[f"data-stud-{side}"]=cylinder(data[0]-14.8,data[1],data[2]+side*7.5,1.25,20.0,(1,0,0))
    contacts=conflicts(gauges,service|kit)
    from support import rod
    rejected=rod([-49.,-45.,835.],[24.,-28.,840.0],2.4)
    prior=conflicts({"data":gauges["data-boot"]},{"first-rest-support":rejected})
    gauge_pair=conflicts({k:v for k,v in gauges.items() if k.startswith("data")},{"power":gauges["power-boot"]})
    access=[]
    for k,gauge in gauges.items():
        for n,b in hosts.items():
            if n not in kit and cad.occupied(bounds(gauge),b):access.append([k,n])
    # Exact source-independent fastener corridors for the two caps: tools are inserted along their shoe normal.
    paths=read(HERE/"route-controls.json")
    driver={}
    for name,data in pads.items():
        center=np.array(data["center_mm"]);axis=np.array(data["axis"]);up=np.array(data["release_cap_direction"])
        lat=np.cross(up,axis)
        d=6.30 if name=="data" else 8.60
        for sign in (-1,1):
            top=center+lat*sign*(d/2+3.25)+up*(d/2+4.5+3+.3)
            tool=cylinder(*top,1.8,34,up)
            driver[f"{name}/{sign}"]=conflicts({"driver":tool},service|kit)
    # The soldered transition stays with the harness. Remove its two screws and unclip the
    # jacket before lifting the board clear of the frame. The lower fixture remains installed.
    others={k:v for k,v in service.items() if k not in ("transition-board","board-M2-0","board-M2-1","insulating-board-clip","board-support")}
    board_sweep=box(-53,-15.55,844.01,18,16,13.59)
    board_conflicts=conflicts({"lift-board-12mm":board_sweep},others|kit)
    for i,y in enumerate((-13.,-2.1)):
        driver["board/"+str(i)]=conflicts({"driver":cylinder(-45.5,y,847.5,1.8,34)},service|kit)
    return {"support_sha256":read(HERE/"support.json")["step_sha256"],"host_sha256":read(boxes_path)["source_sha256"],
            "transition_board_service_lift_mm":12,"transition_board_lift_conflicts":board_conflicts,
            "full_stroke_native_kit_intersections":shop_hits,"full_stroke_unresolved_host_boxes":moving,
            "release_path_mm":14,"both_plug_release_material_conflicts_mm3":contacts,
            "rejected_first_support_mm3":prior,"plug_to_plug_conflicts_mm3":gauge_pair,"plug_to_other_host_unresolved":access,
            "maximum_data_stud_projection_gauge_mm":5.2,"cap_driver_gauge_mm":[3.6,34],"cap_drivers":driver,
            "released_support_535mm_lift_unresolved":lift,"lift_requires_both_leads_unplugged_and_parked":True}

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("boxes",type=Path)
    (HERE/"service.json").write_text(json.dumps(fit_service(p.parse_args().boxes), indent=2)+"\n",encoding="utf8",newline="\n")
