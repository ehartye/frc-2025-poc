"""Check release and neighboring placement against all named, reopened candidate STEP solids."""
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/"design/station-l1"))
import build_assembly as cad
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt
from scripts import registered_transit as field, station_l1_mission as mission, station_l1_settle as settle

RAD=57.15
HALF=150.8125
FACTORS={"base":0,"lift":0,"middle":.5,"slide":1}


def interval_gap(a,b,c,d):
    return max(0,c-b,a-d)


def cylinder_sweep(a,b,box,pad=.02):
    # Both endpoints in the part's own frame. A conservative prism around the
    # center segment bounds a pitched/yawed uncertainty pad before subtracting radius.
    gaps=[interval_gap(min(a[i],b[i]),max(a[i],b[i]),box[i],box[i+3]) for i in range(3)]
    horizontal=max(0,gaps[1]-HALF-pad)
    radial=max(0,math.hypot(gaps[0],gaps[2])-RAD-pad)
    return math.hypot(horizontal,radial)


class CAD:
    def __init__(self):
        directory=ROOT/"design/station-l1"
        review=json.loads((directory/"review.json").read_text())
        self.pose_bounds=review["poses"]["transport"]["boundsMm"]
        path=directory/"place.step"
        self.sha=hashlib.sha256(path.read_bytes()).hexdigest()
        if self.sha!=review["poses"]["place"]["stepSha256"]:
            raise ValueError("Unexpected place assembly bytes")
        self.shapes=cad.read_named(path)
        self.boxes={key:cad.bounds(shape) for key,shape in self.shapes.items()}
        self.frames={part["id"]:part["frame"] for part in cad.PARTS}
        self.transport_sha=review["poses"]["transport"]["stepSha256"]
        if hashlib.sha256((directory/"transport.step").read_bytes()).hexdigest()!=self.transport_sha:
            raise ValueError("Transport STEP differs from the reviewed hull")

    def next_sector(self,pipe_x,pipe_y,pipe_z,next_y):
        # Place-stage CAD uses chassis X = -490 mm relative to the trough lip.
        p=((pipe_x+.49)*1000,(pipe_y-next_y)*1000,pipe_z*1000)
        tool=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(p[0],p[1]-HALF,p[2]),gp_Dir(0,1,0)),RAD,2*HALF).Shape()
        entrants=[]
        native=[]
        min_gap=math.inf
        for key,box in self.boxes.items():
            factor=FACTORS[self.frames[key]]
            # Covers movement from carry height to place and all intermediate extensions.
            cover=(box[0]-370*factor,box[1],box[2],box[3],box[4],
                   box[5]+(110 if self.frames[key]!="base" else 0))
            gap=cylinder_sweep(p,p,cover)
            min_gap=min(min_gap,gap)
            if gap>0:
                continue
            entrants.append(key)
            dist=BRepExtrema_DistShapeShape(tool,self.shapes[key])
            contact=BRepAlgoAPI_Common(tool,self.shapes[key])
            if not dist.IsDone() or not contact.IsDone():
                raise ValueError("Native prior-piece occupancy check failed: "+key)
            native.append({"body":key,"gap_mm":dist.Value(),"volume_mm3":cad.volume(contact.Shape())[0]})
        return {"sector_m":next_y,"min_swept_bound_gap_mm":min_gap,"candidate_bodies":entrants,
                "native_place_witnesses":native,"clear_all_modeled_poses":not entrants}


def departure(geometry,sector,delay,piece_id="station-coral-001"):
    route=field.RegisteredMission(sector)
    up=mission.run(dt=.001,target_y=sector,trajectory=route,sample_every_step=True,piece_id=piece_id)
    post=settle.continue_contact(sector=sector,withdrawal="after-separation",wait_after_true_separation_s=delay,
                                 upstream=up,piece_id=piece_id)
    if post["stop"]!="sustained-paired-floor-and-lip":
        raise ValueError("No finite supported state available to continue")
    witness=post["clear_witness_from_station_s"]
    floor=post["start_from_station_s"]
    # The raw solver's adjacent substeps are closer than the decimated 10 ms post
    # trace. Inflate that later center-box by a bound using sampled speeds plus
    # the maximum per-step displacement recorded by the solver for this run.
    pts=[(r["time_s"],r["target_xyz_m"]) for r in up["trace"]
         if witness<=r["time_s"]<floor]
    pts.extend((floor+t,(x,sector,z)) for t,x,z,vx,vz,omega in post["trace"])
    last_unclear={}
    min_gap=math.inf
    tail_gap=math.inf
    bound_speed=post["peak_planar_speed_mps"]
    for (ta,a),(tb,b) in zip(pts,pts[1:]):
        # The total along-path displacement is bounded by observed step maximum.
        # Endpoints of a continuous path imply all intermediate points lie within
        # vmax*dt/2 of at least one endpoint. Gravity bow sag is covered too.
        pad=.025+(bound_speed*1000*(tb-ta)/2 if tb>floor else 1.3*1000*(tb-ta)/2)
        dlo=max(0,min(370,(ta-witness-delay)*150))
        dhi=max(0,min(370,(tb-witness-delay)*150))
        for key,box in geometry.boxes.items():
            factor=FACTORS[geometry.frames[key]]
            p=((a[0]+.49)*1000+factor*dlo,(a[1]-sector)*1000,a[2]*1000)
            q=((b[0]+.49)*1000+factor*dhi,(b[1]-sector)*1000,b[2]*1000)
            gap=cylinder_sweep(p,q,box,pad)
            if gap<=0:
                last_unclear[key]=tb
            min_gap=min(min_gap,gap)
            if ta>=floor+.25:
                tail_gap=min(tail_gap,gap)
    x,z=post["last_xz_vxvz_spin"][:2]
    return {"piece_id":up["piece_id"],"sector_m":sector,"floor_time_s":floor,
            "first_shelf_witness_s":witness,"body_count":len(geometry.boxes),
            "last_possible_contact_s":max(last_unclear.values(),default=witness),
            "last_possible_by_body":last_unclear,"min_tail_gap_mm":tail_gap,
            "post_observation_s":floor+post["elapsed_s"],"settled_target_xz_m":[x,z],
            "checks_next":[geometry.next_sector(x,sector,z,other) for other in (.18,.22)]}
