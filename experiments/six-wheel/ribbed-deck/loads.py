"""Variable-section rib and inter-rib skin screen for the load-bearing deck."""
import json
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
E=68300.
RAIL=232.

def section(flange,skin,web,depth):
    af=flange*skin
    h=depth-skin
    aw=web*h
    z=(af*skin/2+aw*(skin+h/2))/(af+aw)
    inertia=flange*skin**3/12+af*(skin/2-z)**2+web*h**3/12+aw*(skin+h/2-z)**2
    return inertia,np.maximum(z,depth-z)

def response(force,y,flange,skin,local_width,local_skin,profile,web,local_span=50.,step=.25):
    # The full load goes to one transverse web. Side bearing is in compression.
    # Unit-load work gives the displacement at the application point.
    span=2*RAIL
    s=np.arange(step/2,span,step)
    arm=RAIL+y
    left=(span-arm)/span
    unit=left*s-np.maximum(0.,s-arm)
    line=np.abs(s-RAIL)
    depth=np.interp(line,profile[:,0],profile[:,1])
    active=line<226.
    thickness=np.where(line>=212,4.,skin)
    inertia,c=section(flange,thickness,web,depth)
    inertia=np.where(active,inertia,flange*4**3/12)
    c=np.where(active,c,2.)
    stress=np.abs(force*unit*c/inertia)
    global_travel=force*np.sum(unit**2/inertia)*step/E
    # Mid-bay transverse load is first delivered through a simply supported
    # strip of skin in the orthogonal direction. This uses the full patch
    # load as a point and ignores two-way spreading into the continuous plate.
    local_travel=force*local_span**3/(4*E*local_width*local_skin**3)
    local_stress=1.5*force*local_span/(local_width*local_skin**2)
    return {"ribTravelMm":float(global_travel),"interRibSkinTravelMm":float(local_travel),
            "seriesTravelMm":float(global_travel+local_travel),"nominalRibPeakMPa":float(stress.max()),
            "peakRibYMm":float(s[int(stress.argmax())]-RAIL),"interRibSkinPeakMPa":local_stress,
            "leftBearingN":force*left,"rightBearingN":force*(1-left)}

def run():
    dimensions=json.loads((HERE/"geometry.json").read_bytes())
    profile=np.array(dimensions["sectionDepthFromTopMm"])
    skin=dimensions["skinMm"]
    mechanics=json.loads((ROOT/"fixed-lift-omission/mechanics.json").read_bytes())
    # Width across the rib for effective flange, along the rib for local skin;
    # neither is evidence of actual load spreading through the built panel.
    mast=mechanics["oneSideLock"]["3"]["forceN"]
    cases=(("battery",6.17*9.80665*3,-14.5,50,30,4,50),
           ("service",100,25,30,30,skin,50),
           ("platformPost",2*9.80665*3,205,8,8,4,50),
           ("mastSingleSide",mast,222.5,50,25,4,50))
    result={}
    for key,force,y,w,lw,lh,pitch in cases:
        check=response(force,y,w,skin,lw,lh,profile,dimensions["webMm"],pitch)
        fine=response(force,y,w,skin,lw,lh,profile,dimensions["webMm"],pitch,.125)
        control=mechanics["stripAt3g"]["servicePoint" if key=="service" else key]["original"]
        result[key]={"forceN":force,"yMm":y,"effectiveFlangeMm":w,"localStripWidthMm":lw,"localThicknessMm":lh,
                     "sourcePlateTravelMm":control["deflectionMm"],"sourcePlateStressMPa":control["stressMPa"],
                     "nominal":check,"travelRefinementMm":abs(fine["seriesTravelMm"]-check["seriesTravelMm"])}
    controls={}
    for label,load,y,w,thick,pitch in (("lostWebUnderService",100.,25.,30.,skin,100.),
              ("narrowBatteryLoadPath",6.17*9.80665*3,-14.5,8.,4.,50.),
              ("touchdownOneSideSensitivity",1900.,222.5,25.,4.,50.)):
        controls[label]=response(load,y,w,skin,w,thick,profile,dimensions["webMm"],pitch)
    result={"deckSha256":dimensions["stepSha256"],"cases":result,"controls":controls,
        "modulusMPa":E,"physicalDeflectionMm":None,"acceptedLoadN":None}
    (HERE/"loads.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf8",newline="\n")
    for k,v in result["cases"].items():
        print(k,v["sourcePlateTravelMm"],round(v["nominal"]["seriesTravelMm"],3),
              round(v["nominal"]["nominalRibPeakMPa"],1),round(v["nominal"]["interRibSkinPeakMPa"],1))
    for k,v in controls.items():print(k,round(v["seriesTravelMm"],3),round(v["interRibSkinPeakMPa"],1))

if __name__=="__main__":
    run()
