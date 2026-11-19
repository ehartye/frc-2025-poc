"""Verify the full motion tube and both scoring assessments from an attained plant trace."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from park_endpoint import R4, SIX, HERE, assembly, field, park, planar_shift, read

# This tube covers the requested south lane through the red-side AUTO stop.
WEST_M, EAST_M = 9.600,10.60
LANE_Y_M = 2.4272875
TRANSLATION_M = .025
YAW_RAD = math.radians(3)
PARK_CASE = "offset-for-cage-swing"


def certify(field_step):
    endpoint = read(HERE/"park-endpoint.json")
    hull = endpoint["hullMm"]
    modules = read(HERE.parent/"packaging/module.json")["modules"]
    wheel = read(HERE.parent/"packaging/envelopes.json")
    radius = max(wheel["movingRadiusMm"].values())
    for module in modules.values():
        for axis, coordinate in enumerate(("xM","yM")):
            center = 1000*module[coordinate]
            if center-radius < hull[axis] or center+radius > hull[axis+3]:
                raise ValueError("Turning a wheel can leave the R4 swept box")
    f = park.corrected()
    route = f.Route((EAST_M, LANE_Y_M, math.pi))
    route.go((WEST_M, LANE_Y_M))
    box = field.native_box(f,route,hull)
    padding = planar_shift(hull)
    for axis in (0,2):
        box[axis] -= padding
        box[axis+3] += padding
    leaves, supports = field.field_tree(field_step, box)
    prism=field.prism(box)
    gaps={key:field.distance(prism,s) for key,(s,b) in leaves.items() if field.box_distance(b,box)<=field.SEARCH_MM}
    nearest=min(gaps,key=gaps.get)
    stock=read(SIX/"full-match/installed-stock.json")
    if stock["field_step_sha256"]!=field.FIRST_SHA or len(stock["stock"])!=24:
        raise ValueError("Stock inventory differs from registered field")
    stock_gaps={key:field.box_distance(box,v["native_bounds_mm"]) for key,v in stock["stock"].items()}
    cage=read(SIX/"leave-park/cage-motion.json")
    r=endpoint["hangingCageRadiusMm"]
    height=endpoint["highestCageEyeMm"]
    def cone_margin(station):
        x,z=station["axis_mm"]
        nearest_axis=math.hypot(max(box[0]-x,x-box[3],0),max(box[2]-z,z-box[5],0))
        tilt=math.radians(6)
        return nearest_axis-r/math.cos(tilt)-height*math.tan(tilt)
    # The real R4 cloth is tested at the registered start, with the tape eroded for
    # the complete initial translation/yaw error. Numeral ink is excluded.
    robot=R4/"full-robot.step"
    if assembly.sha(robot)!=endpoint["pins"]["cad/full-swerve-r4/full-robot.step"]:
        raise ValueError("R4 STEP changed")
    shapes=assembly.named(robot)
    covers={k:s for k,s in shapes.items() if k.startswith("bumper-cover-")}
    start=read(SIX/"leave-park/near-tape.json")["cases"]["near-tape"]["start"]
    radius=math.hypot(max(abs(hull[0]),abs(hull[3])),max(abs(hull[1]),abs(hull[4])))
    inset=(2+2*radius*math.sin(math.radians(.12)/2))/25.4
    initial=park.overlap(f,covers,start,(park.TAPE[0]+inset,inset,park.TAPE[1]-inset,f.PROFILE["field"]["width"]-inset))
    return dict(fieldSha256=field.FIRST_SHA,parkSourceSha256=assembly.sha(HERE/"park-endpoint.json"),
                nativeSweepBoxMm=box,rotatingModuleRadiusMm=radius, assessedLeaves=len(leaves), contactSupports=sorted(supports),
                staticClearanceLowerMm=min(field.SEARCH_MM,gaps[nearest]), nearestHardware=nearest,
                minimumInitialStockGapMm=min(stock_gaps.values()),cageConeMarginMm=min(map(cone_margin,cage["stations"].values())),
                guaranteedInitialClothOnStartingLineMm3=sum(initial.values()),
                assumedPoseAllowanceMmDeg=[25,25,3], poseAxisBoundsM=[WEST_M,EAST_M,LANE_Y_M],
                scope="Whole stowed R4, previously registered module envelope, static hardware, 24 installed stock positions and a conditional six-degree hanging envelope.")


def review(case_file,geometry):
    result=read(case_file)
    path=case_file.with_suffix(".npy")
    if assembly.sha(path)!=result["trajectorySha256"]:
        raise ValueError("Unrecognized trajectory")
    for rel, digest in result["sourceHashes"].items():
        if assembly.sha(HERE.parents[2]/rel) != digest:
            raise ValueError("Mismatched solver or geometry input: " + rel)
    if assembly.sha(HERE/"park-endpoint.json")!=geometry["parkSourceSha256"]:
        raise ValueError("Parking qualification uses different geometry")
    a=np.load(path,allow_pickle=False)
    if a.shape!=(31201,10) or not np.isfinite(a).all() or not np.allclose(a[:,0],np.arange(len(a))*.005,rtol=0,atol=1e-12):
        raise ValueError("Incomplete state or clock")
    dt=.005
    # For the modeled masses, mu=0.8 and the four module arms, the total contact
    # force is <= mu*m*g; moment <= mu*m*g*max arm. This slack also covers the
    # small reference-point shift from the CG and the angular component.
    cfg=read(HERE/"model.json")
    if cfg["contact"]["mu"]!=.8 or result["massKg"] not in (65,72) or abs(a[:,6]).max()>=1:
        raise ValueError("Recompute interval acceleration bounds for changed model")
    linear_pad=dt*np.linalg.norm(a[:-1,4:6],axis=1)+.5*50*dt**2
    yaw_pad=dt*abs(a[:-1,6])+.5*100*dt**2
    h=abs(np.arctan2(np.sin(a[:-1,3]-math.pi),np.cos(a[:-1,3]-math.pi)))
    track = min(np.min(a[:-1,1]-linear_pad-(WEST_M-TRANSLATION_M)),
                np.min(EAST_M+TRANSLATION_M-a[:-1,1]-linear_pad),
                np.min(TRANSLATION_M-np.abs(a[:-1,2]-LANE_Y_M)-linear_pad))
    heading = YAW_RAD-np.max(h+yaw_pad)
    assessment=a[a[:,0]>=153]
    end=read(HERE/"park-endpoint.json")["cases"][PARK_CASE]
    pos_error=abs(assessment[:,1:3]-np.array(end["poseXYMetresYawRadians"][:2]))
    capture=np.max(h[round(153/dt):]+yaw_pad[round(153/dt):])<YAW_RAD
    pause=a[round(15/dt):round(18/dt)+1]
    auto_end=pause[0]
    half=.42505
    east_edge=park.TAPE[1]*.0254
    auto_gap=np.min(pause[:,1] - half*(abs(np.sin(pause[:,3]))+abs(np.cos(pause[:,3]))) - east_edge)
    start=read(SIX/"leave-park/near-tape.json")["cases"]["near-tape"]["start"]
    start_shift=abs(a[0,1:4]-start)
    fits_start=bool(np.max(start_shift[:2])<=.0020000001 and start_shift[2]<=math.radians(.12000001))
    drift=np.linalg.norm(a[round(15/dt):round(18/dt)+1,1:3]-auto_end[1:3],axis=1).max()
    assessed_stationary=np.max(np.linalg.norm(assessment[:,4:6],axis=1))<1e-5 and max(abs(assessment[:,6]))<1e-5
    peers=result["clearance"]
    positive = all(geometry[key]>0 for key in ("staticClearanceLowerMm","minimumInitialStockGapMm","cageConeMarginMm"))
    gates=dict(movingTubeAndCages=bool(positive and track>0 and heading>0),
               peerAndBarrierClear=bool(min(peers["minimumPeerMm"]-.1,peers["minimumBarrierMm"]-.1)>0),
               startsOnLine=bool(fits_start and geometry["guaranteedInitialClothOnStartingLineMm3"]>0),
               disableAndHold=bool(result["disabledMaxAbsLeadVoltageV"]<1e-8 and assessed_stationary),
               leave=bool(auto_gap>0),
               park=bool(capture and np.max(pos_error)+max(linear_pad[round(153/dt):])<TRANSLATION_M
                         and sum(end["guaranteedClothVolumeMm3"].values())>0),
               battery=set(result["battery"])=={"weak","nominal","screened"} and all(pack["supplied"] for pack in result["battery"].values()))
    return dict(case=result["case"],gates=gates,provisionalPoints=5 if all(gates.values()) else None,
                minPositionTubeMarginMm=1000*track,minHeadingTubeMarginDeg=math.degrees(heading),
                autoGapMm=auto_gap*1000, disabledTravelMm=drift*1000,
                rawTraceSha256=result["trajectorySha256"], unreservedStartRejected=result["unassignedSouthStartPeerMarginMm"]<0)


if __name__=="__main__":
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("field",type=Path)
    ap.add_argument("output",type=Path)
    ap.add_argument("cases",nargs="*",type=Path)
    args=ap.parse_args()
    geometry=certify(args.field)
    reviews=[review(case,geometry) for case in args.cases]
    cases = []
    for case in args.cases:
        native = read(case)
        trajectory=np.load(case.with_suffix(".npy"),allow_pickle=False)
        # Display samples retain the original timestamps in both moving periods and
        # across the full hold. The scoring review above uses every plant service.
        indices=sorted(set(range(0,1001,10)) | set(range(3600,4801,10)) |
                       set(range(0,len(trajectory),200)) | {len(trajectory)-1})
        native["displaySecondsXYHeading"] = np.round(trajectory[indices,:4],7).tolist()
        cases.append(native)
    args.output.write_text(json.dumps(dict(corridor=geometry,trials=reviews,cases=cases),indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
    print(geometry["staticClearanceLowerMm"],geometry["cageConeMarginMm"],geometry["guaranteedInitialClothOnStartingLineMm3"],reviews)
