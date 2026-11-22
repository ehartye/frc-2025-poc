"""Test both directions of P02 axial migration against the open shaft slot and the stock ring."""
from pathlib import Path
import csv,hashlib,json,sys
import numpy as np
from inset_chassis_shop import HERE


def run(plugin,motor_file,key_file):
    sys.path.insert(0,str(plugin/"scripts"))
    from mechanical_motion import bounds,common
    from component_shape import volume
    from showcase_contacts import load_solids
    from fusion_authority import register_path,verify
    from OCP.gp import gp_Trsf,gp_Ax1,gp_Dir,gp_Pnt
    from OCP.TopLoc import TopLoc_Location
    charge=json.loads((HERE/"inset-oem-steering-key.json").read_bytes());R=HERE.parents[3];reg=register_path(R)
    if verify(R,reg,reg.parent/"issue-005/lineage.json")!=charge["authority"]:raise ValueError("Changed project authority")
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    file=HERE.parent/"private-inputs/matched-sds/corner.step"
    if [sha(p) for p in (file,motor_file,key_file)]!=[charge[k] for k in ("donorStep","motorSHA256","keyKit")]:raise ValueError("Source changed")
    donor,_=load_solids(file);source_corner=next(r for r in json.loads((HERE/"inset-steering-key-installation.json").read_bytes())["rows"] if r["role"]=="steering")
    ring=[(k,v) for k,v in donor.items() if k.startswith("Retaining Ring, 8mm") and abs(bounds(v)[1]-136.2625)<.001]
    if len(ring)!=1:raise ValueError("Wrong installed retaining ring")
    label,ring=ring[0];key=load_solids(key_file)[0][charge["keyArticle"]];motor=next(iter(load_solids(motor_file)[0].values()));shaft=np.array(charge["referenceShaftPoseMm"])
    def move(item,M):
        t=gp_Trsf();t.SetValues(*M[:3].ravel());return item.Moved(TopLoc_Location(t))
    motor=move(motor,shaft);kb=bounds(key)
    def position(y):
        T=np.eye(4);T[:3,3]=[0,2.8-kb[1],y-shaft[1,3]-kb[2]]
        return move(key,shaft@T)
    axial=[]
    for y in (123.9125,124.9125,125.1625,125.4125,125.6625,126.4125,127.7625,128.2625,130.2625,132.7625,137.5):
        key_at=position(y)
        axial.append(dict(keyLowY=y,keyHighY=y+8.5,V11CommonMm3=common(key_at,motor),stockRingCommonMm3=common(key_at,ring),
                         pinionOverlapUpperMm=max(0,min(136.2625,y+8.5)-max(123.5625,y))))
    # Force the actual captured stock ring opening past the key, independent of its parked clock.
    # With both axial ends of the key beyond the ring thickness, this section tests its actual full-width passage.
    stages=[]
    for angle in range(0,360,10):
        tf=gp_Trsf();tf.SetRotation(gp_Ax1(gp_Pnt(*shaft[:3,3]),gp_Dir(0,1,0)),np.deg2rad(angle));r=ring.Moved(TopLoc_Location(tf))
        key_at=position(131.);v=common(key_at,r)
        stages.append(dict(angleDeg=angle,commonMm3=v))
    from mechanical_motion import bearing_area, distance
    def clock(a):
        t=gp_Trsf();t.SetRotation(gp_Ax1(gp_Pnt(*shaft[:3,3]),gp_Dir(0,1,0)),np.deg2rad(a))
        return ring.Moved(TopLoc_Location(t))
    gear=donor[source_corner["gear"]]
    middle=position(131.)
    optimized=[]
    for lo,hi in ((-5.,5.),(39.,51.)):
        for _ in range(26):
            left=(2*lo+hi)/3;right=(lo+2*hi)/3
            if common(middle,clock(left)) <= common(middle,clock(right)):hi=right
            else:lo=left
        angle=(lo+hi)/2
        optimized.append(dict(clockDeg=angle,commonMm3=common(middle,clock(angle)),gapMm=distance(middle,clock(angle))))
    minimum=min(optimized,key=lambda r:r["commonMm3"]);worst=clock(minimum["clockDeg"])
    def tip(y,obj):return common(position(y),obj)
    def limit(lo,hi,sign,obj):
        for _ in range(19):
            mid=(lo+hi)/2
            if (tip(mid,obj)>1e-7)==(sign>0):hi=mid
            else:lo=mid
        return (lo+hi)/2
    lower=limit(123.9125,125.6625,-1,motor)
    high=limit(127.7625,131.,1,worst) if minimum["commonMm3"]>1e-7 else None
    thrust={}
    for label,stop_y,side,obj in (("inboard shaft slot",lower,-1,motor),("outboard push ring",high,1,worst)):
        if stop_y is None:
            thrust[label]=dict(keyCanPassNominalClock=True)
            continue
        at=position(stop_y)
        bearing=bearing_area(at,obj,[0,side,0])
        pressures=[]
        for du in (.005,.01,.05,.10):
            v=tip(stop_y+side*du,obj)
            pressures.append(dict(overtravelMm=du,commonMm3=v,secantDisplacedAreaMm2=v/du))
        thrust[label]=dict(firstContactLowEndYmm=stop_y,travelFromCenteredMm=stop_y-125.6625,
                    staticPlanarBearingAreaMm2=bearing,displacementSensitivity=pressures)
    endpoints=[]
    for label,y in (("shaft-side contact",lower),("nominal",125.6625),("ring-side contact",high)):
        if y is None:continue
        k=position(y)
        endpoints.append(dict(state=label,keyAxialLowYmm=y,
            pinionAxialOverlapMm=max(0,min(136.2625,y+8.5)-max(123.5625,y)),
            shaftTangentialFacesMm2=[bearing_area(k,motor,n*shaft[:3,0]) for n in (-1,1)],
            pinionTangentialFacesMm2=[bearing_area(k,gear,n*shaft[:3,0]) for n in (-1,1)]))
    original_key=donor[source_corner["key"]];as_sold=[]
    for displacement in (0.,.70,2.15):
        S=np.eye(4);S[1,3]=displacement;kk=move(original_key,S)
        as_sold.append(dict(deltaMm=displacement,boundsMm=bounds(kk),motorCommonMm3=common(kk,motor),
            ringAtParkCommonMm3=common(kk,ring),ringAtWeakClockCommonMm3=common(kk,worst),pinionCommonMm3=common(kk,gear)))
    retention=dict(shortKeyEndpoints=endpoints,original12mmKeyAsDrawn=as_sold,minClockControls=optimized,nominalLimits=thrust,
              pinionBearingAgainstRingAreaMm2=bearing_area(gear,ring,[0,1,0]),
              pinionToLowerShimAreaMm2=bearing_area(gear,next(v for k,v in donor.items() if k.startswith("Shim, 8mm") and abs(bounds(v)[4]-123.5625)<.002),[0,-1,0]),
              springRingToV11ShaftRawCommonMm3=common(ring,motor),
              limit="Rotated symmetric source ring using two local minima informed by a 36-angle screen; a positive absent around the whole circle is not certified. Stock floor/end constraint uses actual rounded form. Secant displaced volume per travel is a geometric sensitivity, not allowable area or press-fit strength.")
    bus_csv=HERE.parent/"strategy/timed-bus.csv"
    with bus_csv.open() as f: event=max(csv.DictReader(f),key=lambda v:float(v["steerPhaseA"]))
    area=charge["rows"][1]["pairs"];shaft_area=min(area[1]["parkedKeyFaceAreaMm2"][:2]);pinion_area=min(area[2]["parkedKeyFaceAreaMm2"][:2])
    peak=float(event["steerPhaseA"]);demands=[]
    for name,torque in (("maximum commissioned route phase current, torque upper bound",2.6*peak/105),("full NEO strategy stall-constant upper bound",2.6)):
        demands.append(dict(case=name,torqueNm=torque,tangentialForceAtMinimumKeyRadiusN=1000*torque/2.8,
                            averageProjectedShaftBearingDemandMPa=1000*torque/(2.8*shaft_area),averageProjectedPinionBearingDemandMPa=1000*torque/(2.8*pinion_area)))
    return dict(authority=charge["authority"],sourceIssue=source_corner["motor"],shortKey=charge["keyKit"],ring=label,shaftY=shaft[:3,3].tolist(),axial=axial,clockedStockRing=stages,
        retention=retention,currentTraceSHA256=sha(bus_csv),peakSteeringEvent=event,loadDemands=demands,
        scope="Two real axial constraints, three-dimensional purchased key, genuine stock ring rotated over its legal unconstrained clock. Checking a key cross-section through the entire ring thickness can disprove azimuth-independent retention. Bounds at commissioned max route current and modeled stall use the minimum 2.8 mm contact radius. Local edge stress, material allowables, axial capture and installed housing must all be satisfied.")
if __name__=="__main__":
    d=run(*map(Path,sys.argv[1:]));(HERE/"inset-oem-key-retention.json").write_text(json.dumps(d,indent=1)+"\n")
