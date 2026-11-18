"""Exercise source-bound station measurements, time gates and error budgets offline."""
import hashlib
import itertools
import json
import math
from pathlib import Path
import numpy as np
from feature import run as feature_patches
from attitude import PRIOR,PIVOT_Z,outer,projected
from pose import EYE,SECOND_EYE,LANDING,ROOT,image,POINTS

HERE=Path(__file__).resolve().parent
EYES=(EYE,SECOND_EYE)
# Coupled survey errors are defined at the landing center, not at the robot center.
# Includes the rigid camera-to-receiver, board-to-chute and receiver deflection errors.
CAL=(.50,.10,.020)
MAX_AGE_MS=20
SPEED_MM_S=2
YAW_DEG_S=.05


def edges(result):
    if result is None: return None
    intervals=np.array(result)
    normal,side,yaw=[max(abs(intervals[i]))+c for i,c in zip((0,1,3),CAL)]
    # The held-state velocity bounds must include motion of the receiver datum itself.
    normal+=SPEED_MM_S*MAX_AGE_MS/1000
    side+=SPEED_MM_S*MAX_AGE_MS/1000
    yaw+=YAW_DEG_S*MAX_AGE_MS/1000
    planes=[]
    dy=intervals[1]+np.array([-1,1])*(CAL[1]+SPEED_MM_S*MAX_AGE_MS/1000)
    dx=intervals[0]+np.array([-1,1])*(CAL[0]+SPEED_MM_S*MAX_AGE_MS/1000)
    ang=intervals[3]+np.array([-1,1])*(CAL[2]+YAW_DEG_S*MAX_AGE_MS/1000)
    for station_x in (-100,100):
        pts=[y+(station_x-x)*math.tan(math.radians(t)) for x,y,t in itertools.product(dx,dy,ang)]
        planes.append([min(pts),max(pts)])
    return {"normalMm":normal,"sideMm":side,"headingDeg":yaw,"sideIntervalsAtTwoPlanesMm":planes,
            "tight":bool(normal<=5 and side<=.35 and yaw<=.15),"wide":bool(normal<=5 and side<=.50 and yaw<=.25)}


def gate(frames,now_ms,*,contact=False,validated=False,epoch=1,last_capture_ms=-1):
    if contact: return "hold-possible-load-or-contact"
    if not validated: return "hold-unverified-survey-or-calibration"
    if not frames or len({f["camera"] for f in frames})!=len(frames):return "hold-camera-identity"
    for f in frames:
        if f["camera"] not in ("left","right") or f["tag"]!=2 or f["count"]!=4 or not f["clear"]:
            return "hold-identity-visibility"
        if f["epoch"]!=epoch or not last_capture_ms < f["captureMs"] <= now_ms or now_ms-f["captureMs"]>MAX_AGE_MS:
            return "hold-time-epoch"
    if max(f["captureMs"] for f in frames)-min(f["captureMs"] for f in frames)>.2:
        return "hold-unsynchronized"
    return "eligible-for-pose-estimation"


def source_sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run():
    native=json.loads((HERE/"sightlines.json").read_bytes())
    static=np.zeros(6)
    controls={}
    cases={}
    thresholds={}
    for name,optics in (("right",(EYE,)),("pair",EYES)):
        m=[projected(static,e) for e in optics]
        rows={}
        for px in (.04,.05,.10,.20,.40,1.0):
            nominal=outer(m,optics,px)
            # All pixel combinations within +/-px of the zero-pose projection may be measured.
            # A different true pose consistent with those readings can be up to 2px from nominal.
            repeat=outer(m,optics,2*px)
            rows[str(px)]={"alignedExample":edges(nominal),"alignedEveryInBudgetReading":edges(repeat),
                           "sixCoordinatesAtNominal":nominal}
        cases[name]=rows
        thresholds[name]={}
        for tier in ("tight","wide"):
            lo,hi=0.,.4
            for _ in range(14):
                mid=(lo+hi)/2
                if edges(outer(m,optics,mid))[tier]:lo=mid
                else:hi=mid
            thresholds[name][tier]={"largestTestedAlignedResidualPx":lo,
                                    "alignedEveryInBudgetReadingPx":lo/2}
    for name,q,error in (("tiltedEligible",[.4,.02,.02,.01,.01,-.01],.04),
                         ("tiltedUnresolved",[2,.20,.5,.09,.10,-.09],.10),
                         ("adverse",[3,1.20,.5,.45,.12,-.08],.10)):
        pose=np.array(q)
        pose[3:]=np.radians(pose[3:])
        observed=[projected(pose,e) for e in EYES]
        bounded=outer(observed,EYES,error)
        truth=np.array(q)
        if not all(a<=v<=b for (a,b),v in zip(bounded,truth)):raise ValueError("Ground truth excluded")
        cases[name]={"truth":q,"errorPx":error,"sixCoordinates":bounded,"entry":edges(bounded)}
    frames=[{"camera":k,"tag":2,"count":4,"clear":True,"epoch":1,"captureMs":100} for k in ("right","left")]
    for label,changes,now in (("nominal",{},108),("stale",{},125),("future",{},99),("wrongTag",{"tag":1},108),
                             ("missingBolt",{"count":3},108),("hidden",{"clear":False},108),("priorEpoch",{"epoch":0},108),
                             ("unpairedTime",{"captureMs":99.7},108)):
        attempt=[frames[0],frames[1]|changes]
        controls[label]=gate(attempt,now,validated=True)
    controls["sameFrameRepeated"]=gate(frames,108,validated=True,last_capture_ms=100)
    controls["physicalDefault"]=gate(frames,108)
    controls["feedInProgress"]=gate(frames,108,validated=True,contact=True)
    m=[projected(static,e) for e in EYES]
    inconsistent=[[list(uv) for uv in eye] for eye in m]
    inconsistent[1][0][1]+=20
    controls["shiftOneLandmark20pxRejected"]=outer(inconsistent,EYES,.1) is None
    # A common board displacement cannot be distinguished from the equal robot translation.
    diff=max(math.dist(image(p,(0,0,0),e),image((p[0],p[1]+.60,p[2]),(0,.60,0),e))
             for e in EYES for p in POINTS)
    controls["unsurveyedBoardShiftMm"]={"side":.60,"maximumPixelDifference":diff,"bothWindowsExceeded":.60>.50}
    frame_sha={p.relative_to(ROOT).as_posix():source_sha(p) for p in (
        ROOT/"station-supply/entry-window.json",ROOT/"station-supply/compensated-dock.json",ROOT/"release-observer/stereo.py",
        ROOT/"design/competition-package/shop-duplex/red/source-register.json",HERE/"sightlines.json",HERE/"rejected-mount.json")}
    front=native["boltFaces"][0]["centerStationMm"][0]
    return {"schema":"station-mounted-survey/1","sourceSha256":frame_sha,"landingStationMm":[LANDING,0,PIVOT_Z],
            "boardFaceToLandingNormalMm":LANDING-front,"coarsePriorMmDeg":[*PRIOR[:3],*np.degrees(PRIOR[3:])],
            "landingBiasMmDeg":CAL,"maxCaptureAgeMs":MAX_AGE_MS,"velocityHeldMmS":SPEED_MM_S,"yawRateHeldDegS":YAW_DEG_S,
            "cases":cases,"thresholds":thresholds,"syntheticPatches":feature_patches(),"controls":controls,"physicalMeasurementVerified":False}


if __name__=="__main__":
    print(json.dumps(run(),indent=2,allow_nan=False))
