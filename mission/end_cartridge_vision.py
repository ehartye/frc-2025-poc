"""Bound the full pipe beyond the robot's forwardmost occupied X plane using synthetic stereo end marks."""
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"release-observer"))
import stereo
from build_mount import hits


def ray_box(origin,target,box):
    lo,hi=0.,1.
    for i in range(3):
        delta=target[i]-origin[i]
        if abs(delta)<1e-9:
            if not box[i]<=origin[i]<=box[i+3]:
                return False
            continue
        a=(box[i]-origin[i])/delta
        b=(box[i+3]-origin[i])/delta
        lo=max(lo,min(a,b))
        hi=min(hi,max(a,b))
    return lo<=hi


def plane_gap(marks,front,pixel_error):
    # Reuse stereo's full endpoint identity, epipolar, FOV and finite-size validation.
    stereo.clearance(marks,front,0,pixel_error,1,.5)
    ends=[stereo.bound_pair(mark,pixel_error) for mark in marks]
    nearest_axis=min(p[0] for box in ends for p in box)
    return nearest_axis-front-stereo.RADIUS-.5-1


def study(robot,up,dep):
    release=next(e["time_s"] for e in up["events"] if e["event"]=="left-powered-shelf-with-same-piece")
    contact=[]
    exposed=[]
    front=robot.hulls["place"][3]+min(2,robot.errors["outer_slide_x_mm"])
    for r in up["trace"]:
        if r["phase"]=="place" and r["state"]=="supported":
            a,b=stereo.endpoints(r)
            yfrac=-a[1]/(b[1]-a[1])
            u=a[0]+(b[0]-a[0])*yfrac-r["slide_m"]*1000
            if 0<=yfrac<=1 and abs(u-155)<=2.8:
                contact.append(r["time_s"])
        if r["time_s"]>=release:
            ends=stereo.endpoints(r)
            exposed.append((r["time_s"],ends,stereo.image_pairs(ends)))
    earliest=contact[-1]-.003 if contact else None
    obstacles={k:s for k,s in robot.shapes.items() if k!="observer-camera-proxy"}
    results=[]
    rays={}
    for px in (.10,.25):
        detections=[]
        timely=0
        for phase in range(17):
            rows=[(t,ends) for t,ends,marks in exposed if round(t*1000)%17==phase and plane_gap(marks,front,px)>=1]
            if rows:
                t,ends=rows[0]
                detections.append(t)
                if earliest is not None and t+.001+.050+.020<=earliest+.150+1e-9:
                    timely+=1
        views=[]
        # Check both extremes in the eligible exposure sequence against native
        # robot geometry. Intermediate rays and actual end-rim visibility stay open.
        if detections:
            for t in sorted({min(detections),max(detections)}):
                if t not in rays:
                    ends=next(ends for time,ends,_ in exposed if time==t)
                    intersect=set()
                    for eye in (-stereo.BASELINE/2,stereo.BASELINE/2):
                        origin=(stereo.ORIGIN[0],eye,stereo.ORIGIN[2])
                        for pt in ends:
                            # Include upper-rim landmark in addition to end center.
                            for dz in (0,stereo.RADIUS):
                                target=(pt[0],pt[1],pt[2]+dz)
                                candidates={k:s for k,s in obstacles.items() if ray_box(origin,target,robot.boxes[k])}
                                intersect.update(hits(candidates,origin,target))
                    rays[t]=sorted(intersect)
                views.append({"after_shelf_ms":round((t-dep["first_shelf_witness_s"])*1000,3),
                              "intersected_bodies":rays[t]})
        results.append({"pixel_error_px":px,"phases":17,"detected_in_free_flight":len(detections),
                        "nominal_clock_bracket_pass_before_visibility":timely,
                        "after_shelf_ms":[round((t-dep["first_shelf_witness_s"])*1000,3) for t in sorted(detections)],
                        "extreme_exposure_native_rays":views})
    return {"front_bound_mm":front,"gap_required_mm":1,"registration_error_mm":1,"pipe_size_error_mm":.5,
            "frame_period_ms":17,"outcomes":results,"qualified_vehicle_observer":False}
