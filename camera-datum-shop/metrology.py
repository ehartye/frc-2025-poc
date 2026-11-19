"""Fit the two surveyed target planes and evaluate bounded, physical camera registration."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
# Survey these nominal dot centers in shelf coordinates, then feed the measured values to the checker.
DOTS=np.array([[8,-60],[88,-60],[8,60],[88,60],[19,-29]],float)
Z=(525.,685.)
F=432.4551801426587
PUPILS={"left":(48.,37.5,842.),"right":(48.,-37.5,842.)}


def project(h,xy):
    x=np.c_[xy,np.ones(len(xy))]@h.T
    return x[:,:2]/x[:,2,None]


def mapping(pixels,metric):
    # Four corner correspondences fit a planar projective map. The asymmetric fifth dot is independent.
    rows=[];out=[]
    for (x,y),(a,b) in zip(pixels[:4],metric[:4]):
        rows+= [[x,y,1,0,0,0,-a*x,-a*y],[0,0,0,x,y,1,-b*x,-b*y]]
        out+=[a,b]
    h=np.r_[np.linalg.solve(rows,out),1].reshape(3,3)
    residual=float(np.linalg.norm(project(h,pixels[4:])-metric[4:],axis=1).max())
    return h,residual


def eye_result(eye,record,budget):
    axis=np.array(record["principal_px"],float)
    # Ray stays inside the measured four-corner plane for both eyes.
    other=axis+np.array([110. if eye=="left" else -110.,0.])
    result=[]
    for plane in record["planes"]:
        dots=np.array(plane["surveyed_xy_mm"],float)
        observed=np.array(plane["rectified_dots_px"],float)
        if dots.shape!=(5,2) or observed.shape!=(5,2) or not np.isfinite([dots,observed]).all():raise ValueError("five surveyed/identified dots required")
        h,residual=mapping(observed,dots)
        metric=project(h,[axis,other])
        # Require both ray intersections to be interpolated within the surveyed quadrilateral.
        corners=dots[[0,1,3,2]]
        for sample in metric:
            edges=np.roll(corners,-1,axis=0)-corners
            signed=edges[:,0]*(sample-corners)[:,1]-edges[:,1]*(sample-corners)[:,0]
            if not(np.all(signed>=0) or np.all(signed<=0)):raise ValueError("ray outside surveyed target")
        result.append((float(plane["z_mm"]),metric,residual))
    (z0,p0,r0),(z1,p1,r1)=result
    if not all(np.isfinite([z0,z1,*axis,r0,r1])) or z1-z0<150 or max(r0,r1)>budget["ray_xy_mm"]:raise ValueError("uncertain registration target")
    d=z1-z0; err=budget["ray_xy_mm"]; ez=budget["survey_z_mm"]
    widths=[np.linalg.norm(v[1]-v[0]) for v in (p0,p1)]
    dw=widths[0]-widths[1]
    if dw<=4*err:raise ValueError("parallel or reversed rays")
    off=d*widths[1]/dw
    lo=(d-2*ez)*(widths[1]-2*err)/(dw+4*err)
    hi=(d+2*ez)*(widths[1]+2*err)/(dw-4*err)
    zhat=z1+off
    zerr=ez+max(off-lo,hi-off)
    slope=(p1[0]-p0[0])/d
    xy=p1[0]+slope*off
    origin=np.r_[xy,zhat]
    slope_err=(2*err+np.linalg.norm(slope)*2*ez)/(d-2*ez)
    angular=math.degrees(math.atan(np.linalg.norm(slope)))
    tilt=angular+math.degrees(math.atan(slope_err))+budget["survey_tilt_deg"]
    xerr=err+hi*slope_err+np.linalg.norm(slope)*max(off-lo,hi-off)+budget["survey_origin_mm"]
    # The plane determines image clocking at the pupil even if the principal ray is perfectly plumb.
    vec=(p0[1]-p0[0])*(1 if eye=="left" else -1)
    yaw=math.degrees(math.atan2(vec[0],-vec[1]))
    clock=abs(yaw)+math.degrees(math.asin(min(1,2*err/widths[0])))+budget["survey_clock_deg"]
    # Per-eye error includes common fixture registration, not just two-camera baseline consistency.
    dist=np.linalg.norm(np.abs(origin-np.array(PUPILS[eye]))+[xerr,xerr,zerr])
    return {"pupil_mm":origin.tolist(),"pupil_error_bound_mm":float(dist),"xy_uncertainty_mm":float(xerr),
            "z_uncertainty_mm":float(zerr),"tilt_bound_deg":float(tilt),"clock_bound_deg":clock,
            "independent_fifth_mark_max_error_mm":max(r0,r1),"ray_width_mm":[float(x) for x in widths]}


def evaluate(record):
    b=record["uncertainties"]
    keys=("ray_xy_mm","survey_z_mm","survey_origin_mm","survey_tilt_deg","survey_clock_deg")
    if set(b)!=set(keys) or any(not np.isfinite(b[k]) or b[k]<=0 for k in keys):raise ValueError("positive independent uncertainty budgets required")
    if record.get("measurement_kind") not in ("synthetic","measured"):raise ValueError("measurement kind required")
    if record["measurement_kind"]=="measured":
        for key in ("survey_record","image_pair_records","instrument_record","camera_serials","stroke_mm","load_case"):
            if key not in record or record[key] in ("",None,[]):raise ValueError("missing acquisition evidence: "+key)
    if not record.get("calibration_epoch") or set(record.get("eyes",{}))!=set(PUPILS):raise ValueError("both eyes and revision required")
    if record.get("fixture_survey")!="independent" or not record.get("undistorted"):raise ValueError("independent target and rectified frames required")
    views={eye:eye_result(eye,v,b) for eye,v in record["eyes"].items()}
    fits=all(v["pupil_error_bound_mm"]<=1 and v["tilt_bound_deg"]<=.12583 and v["clock_bound_deg"]<=.1 for v in views.values())
    return {"observations":views,"within_project_pose_screen":fits,"measurement_kind":record["measurement_kind"],
            "physical_measurement_eligible":fits and record["measurement_kind"]=="measured","authorizes_motion":False}


def example(offset=0):
    example={"calibration_epoch":"synthetic-bench-fixture","measurement_kind":"synthetic","fixture_survey":"independent","undistorted":True,
       "uncertainties":{"ray_xy_mm":.035,"survey_z_mm":.025,"survey_origin_mm":.08,"survey_tilt_deg":.015,"survey_clock_deg":.015},"eyes":{}}
    for eye,p in PUPILS.items():
        x,y,z=p
        rows=[]
        for level in Z:
            uv=np.c_[320-(DOTS[:,1]-y)*F/(z-level),240-(DOTS[:,0]-(x+offset))*F/(z-level)]
            rows.append({"z_mm":level,"surveyed_xy_mm":DOTS.tolist(),"rectified_dots_px":uv.tolist()})
        example["eyes"][eye]={"principal_px":[320,240],"planes":rows}
    return example


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("measurement",nargs="?",type=Path);a=p.parse_args()
    if a.measurement:
        try:r=evaluate(json.loads(a.measurement.read_bytes()))
        except (ValueError,KeyError,np.linalg.LinAlgError) as exc:r={"within_project_pose_screen":False,"reason":str(exc),"authorizes_motion":False}
        print(json.dumps(r,indent=2,allow_nan=False))
    else:
        ref=example()
        (HERE/"metrology-example.json").write_text(json.dumps(ref,indent=2)+"\n",encoding="utf8",newline="\n")
        loose=example();loose["uncertainties"]["ray_xy_mm"]=.18
        alias=example(2.)
        swapped=example();swapped["eyes"]["left"]["planes"][1]["rectified_dots_px"][4][0]+=3.
        try:out=evaluate(swapped)
        except ValueError as exc:out={"within_project_pose_screen":False,"reason":str(exc)}
        rows={"source_ideal":evaluate(ref),"uncertainty_too_large":evaluate(loose),"housing_relocated_2mm":evaluate(alias),"asymmetric_mark_3px":out}
        (HERE/"metrology.json").write_text(json.dumps(rows,indent=2)+"\n",encoding="utf8",newline="\n")
