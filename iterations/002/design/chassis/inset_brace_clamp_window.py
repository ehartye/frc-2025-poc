"""Check the first contacts, head seat and blind-side grip of the relocated brace screws."""
from pathlib import Path
import hashlib,json,sys,math
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Pnt,gp_Dir,gp_Ax2
from inset_stock_check import HERE,form
from port_anchor_fit import oriented

def run(scripts):
    sys.path.insert(0,str(scripts))
    from showcase_contacts import load_solids
    from mechanical_motion import common,distance,bearing_area
    from fusion_authority import register_path,verify
    reg=register_path(HERE.parents[3]);auth=verify(HERE.parents[3],reg,reg.parent/"issue-005/lineage.json")
    old=json.loads((HERE/"inset-corrected-brace-foot.json").read_bytes());load=json.loads((HERE/"inset-corrected-foot-loads.json").read_bytes())
    if old["authority"]!=auth or load["authority"]!=auth:raise ValueError("Changed registered parent")
    source=json.loads((HERE/"inset-mount-purchases.json").read_bytes())["originals"]
    allparts=load_solids(reg.parent/"issue-005/assembly.step")[0]
    need=max(r["combined"]["upperTensionOneBoltN"] for r in load["stations"].values())
    gauges=[];seats=[]
    for sign in [-1,1]:
        name=f"Truss landing 250 {sign*145}";stock=old["stockChanges"][name]
        for radius in [3.3,3.575]:
            revised=json.loads(json.dumps(stock))
            for hole in revised["steps"][-1]["profiles"]:hole["radius"]=radius
            foot=form(revised)
            for x,y in [(234,132*sign),(266,158*sign)]:
                for sku in ["91251A539","91251A542"]:
                    specification=source[sku];raw=allparts[specification["source"]]
                    screw=oriented(raw,specification["underHeadCenterMm"],specification["intoThreadUnit"],[x,y,193.675],[0,0,-1])
                    seats.append(dict(sku=sku,axis=[x,y],radiusMm=radius,bearingAreaMm2=bearing_area(screw,foot,[0,0,1]),oppositeAreaMm2=bearing_area(screw,foot,[0,0,-1])))
                if (sign,x)!=(-1,234):continue
                # A local annulus cut from the commissioned underside of the roof reproduces the through hole.
                p=gp_Ax2(gp_Pnt(x,y,184.15),gp_Dir(0,0,1))
                host=BRepAlgoAPI_Cut(BRepPrimAPI_MakeCylinder(p,7,3.175).Shape(),BRepPrimAPI_MakeCylinder(p,radius,3.175).Shape()).Shape()
                specification=source["91251A539"]
                for dx,dy in [(0,0),(.1,0),(.1,.1),(.15,.15),(.15+.075/2**.5,.15+.075/2**.5)]:
                    screw=oriented(allparts[specification["source"]],specification["underHeadCenterMm"],specification["intoThreadUnit"],[x+dx,y+dy,193.675],[0,0,-1])
                    gauges.append(dict(radiusMm=radius,shiftMm=[dx,dy],gapMm=distance(screw,host),commonMm3=[common(screw,host),common(host,screw)]))
    head=4.40
    min_clearance=3.55-3.175-math.sqrt(2)*.15-.075
    guaranteed_area=math.pi*(head**2-3.60**2)
    cold_outer={radius:math.sqrt(min(row["oppositeAreaMm2"] for row in seats if row["radiusMm"]==radius)/math.pi + radius**2) for radius in [3.3,3.575]}
    if any(radius<head for radius in cold_outer.values()):raise ValueError("Effective head land outside the stated stock receiving range")
    rows=[]
    for sku,length in [("91251A539",15.875),("91251A542",25.4)]:
        # Commissioned fabrication dimensional ranges, no unsupported supplier tolerance is asserted.
        grip=(6.20+3.00,6.50+3.30)
        engagement=max(0,min(9.85,length-.15-grip[1])-2*1.27)
        rows.append(dict(sku=sku,lengthMm=length,gripIntervalMm=grip,minUsableFemaleLengthAfterTwoPitchesMm=engagement,
                         boundingMinorCylinderQuarterThreadAreaMm2=math.pi*5.1054*engagement*.25,
                         outsideFaceProjectionNominalMm=length-9.525-10,
                         gapToInsideBeamFloorAtLongShortStackMm=193.675-.15-length-.15-(165.10+.15)))
    return dict(authority=auth,source=source["91251A539"]["source"],gauges=gauges,headSeats=seats,
                measuredUpperScrewTensionN=need,nominalEffectiveCircularHeadRadiusMm=cold_outer,requiredShopLimits={"maxBossTakeupPerAxisMm":.15,"maxHoleToBossFunctionalCenterOffsetMm":.075,
                "throughRadiiMm":[3.55,3.60],"requiredOuterHeadBearingRadiusMinMm":head,"requiredInnerHeadBearingRadiusMaxMm":3.20},
                minShankGapAfterAdverseDiagonalMm=min_clearance,minRemainingHeadRadialLandMm=head-3.60-math.sqrt(2)*.15-.075,
                conditionalMinimumBearingAreaMm2=guaranteed_area,oneBoltExternalStressBeforePreloadMPa=need/guaranteed_area,
                minFootHoleToEdgeLigamentMm=8.-.15-.075-3.60,minNutbarTapMajorToEdgeLigamentMm=5.8-.15-.075-3.175,
                minBeamHoleToBossApertureLigamentMm=math.hypot(9,4)-2.1-3.60-.075-.15,hardwareLengthBounds=rows,
                limits="Drawing/traveler receiving envelopes; the two clearance-hole dimensions distinguish positioning and clamps. Geometry alone does not assign friction, achieved preload, bolt proof strength or aluminum female-thread stripping allowable. 6.2 mm foot / 3.0 mm thin beam roof and adverse stack considered. Head bearing area assumes measured continuous underhead ring spanning the commissioned radii. Grip and bolt length ranges require procurement verification.")
if __name__=="__main__":
    d=run(Path(sys.argv[1]));(HERE/"inset-brace-clamp-window.json").write_text(json.dumps(d,indent=1)+"\n")
