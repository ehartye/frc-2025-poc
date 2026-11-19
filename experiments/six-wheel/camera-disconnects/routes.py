"""Certify the two bounded, plane-independent harness tails from control points in the shelf frame."""
from pathlib import Path
import argparse
import json
import math
import numpy as np

HERE = Path(__file__).resolve().parent

def read(p):
    return json.loads(p.read_bytes())

def values(points, t, derivative=0):
    p = np.array(points, float)
    for _ in range(derivative):
        p = (len(p)-1)*np.diff(p,axis=0)
    t = np.atleast_1d(t)
    n = len(p)-1
    return np.column_stack([math.comb(n,i)*t**i*(1-t)**(n-i) for i in range(n+1)])@p

def derivative_bound(points, order):
    p = np.array(points)
    for _ in range(order):
        p = (len(p)-1)*np.diff(p,axis=0)
    return np.linalg.norm(p,axis=1).max()

def assess(p, radius, boxes):
    # A midpoint's Cartesian deviation is bounded by half-interval times its derivative supremum.
    n=65536; dt=1/(2*n);t=(np.arange(n)+.5)/n
    pos=values(p,t);v=values(p,t,1);a=values(p,t,2)
    speed=np.linalg.norm(v,axis=1);acc=np.linalg.norm(a,axis=1)
    v_error=derivative_bound(p,2)*dt; a_error=derivative_bound(p,3)*dt
    travel=(speed+.5*v_error)*dt
    curv=(np.linalg.norm(np.cross(v,a),axis=1)+v_error*acc+(speed+v_error)*a_error)/(speed-v_error)**3
    lower=math.inf;owner=None
    for key,b in boxes.items():
        b=np.array(b)
        if np.any(pos.max(axis=0)+40<b[:3]) or np.any(pos.min(axis=0)-40>b[3:]):
            continue
        gap=np.linalg.norm(np.maximum(0,np.maximum(pos-b[3:],b[:3]-pos)),axis=1)
        room=(gap-radius-travel).min()
        if room<lower: lower=float(room);owner=key
    return {"length_interval_mm":[float((speed-v_error).mean()),float((speed+v_error).mean())],
            "length_numerical_mm":float(np.linalg.norm(np.diff(values(p,np.linspace(0,1,n+1)),axis=0),axis=1).sum()),
            "min_radius_certified_mm":float(1/curv.max()),"minimum_host_clearance_mm":lower,"closest_native_id":owner}

def audit(boxes_file):
    source=read(boxes_file)
    baseline=read(HERE.parent/"camera-datum-shop/native-place.json")
    if source["source_sha256"]!=baseline["step_sha256"] or source["native_body_count"]!=603:
        raise ValueError("Wrong full-robot boundary source")
    boxes=source["relative_sweep_bounds_mm"]
    paths=read(HERE/"route-controls.json")
    data=assess(paths["data"],3.15,boxes);power=assess(paths["power"],4.30,boxes)
    ts=np.linspace(0,1,2001)
    a,b=(values(paths[k],ts) for k in ("data","power"))
    near=min(np.linalg.norm(a[i:i+40,None,:]-b[None,:,:],axis=2).min() for i in range(0,len(a),40))
    pair=near-(derivative_bound(paths["data"],1)+derivative_bound(paths["power"],1))/4000-3.15-4.30
    # Use 2023 drawing length from A tip to C tip. Preserve the worst surviving source route,
    # its minimum bend radii and the measured as-received length as independent conditions.
    head=read(HERE/"interfaces.json")
    data_tip=head["disconnects"]["data-J1"]["mouth_mm"][0]+6.2
    total_data=[float(1540.659+v+data_tip-a[-1,0]) for v in data["length_interval_mm"]]
    # Power is terminated on a fixed local PCB; a short 26-AWG cord conveys the final current.
    bulk_length=1321.695+power["length_interval_mm"][1]+8.0  # include terminated conductor breakout
    small_length=70.0  # finished one-way 26 AWG allowance including split and dressed ends
    heavy_ohm_per_m=.0393585/1.90
    # Conservative 26-AWG estimate by scaling the source 14-AWG resistance by the gauge area ratio.
    thin_loop_ohm_per_m=2*1.25*.008286*92**(2*(26-14)/39)
    loop=bulk_length/1000*heavy_ohm_per_m+small_length/1000*thin_loop_ohm_per_m+.055
    return {"host_sha256":source["source_sha256"],"data":data,"power":power,"cable_to_cable_clearance_lower_mm":float(pair),
            "data_tip_to_tip_mm":total_data,"data_at_1970mm_fits":total_data[1]<=1970,
            "data_at_2030mm_fits":total_data[1]<=2030,"full_power_2_5mm2_length_mm":bulk_length,
            "power_26AWG_tail_mm":small_length,"estimated_power_terminal_V":5.12-1.2*loop,
            "minimum_tested_converter_low_for_5V":5+1.2*loop+.010,
            "max_interfaces_ohm_at_5V":.12/1.2-(loop-.055),
            "includes_physical_cable_flex_measured":False}

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("boxes",type=Path)
    (HERE/"route-proof.json").write_text(json.dumps(audit(p.parse_args().boxes),indent=2)+"\n",encoding="utf8",newline="\n")
