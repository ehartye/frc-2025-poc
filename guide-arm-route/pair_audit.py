"""Hold the chassis pose and prediction fixed while substituting one pupil at a time."""
import argparse
from pathlib import Path
import sys
import json
import numpy as np
from field_view import HERE,ROOT,read,digest

sys.path.insert(0,str(ROOT/"mounted-guide-vision"))
import sector
import vision


def run(assets):
    old=read(ROOT/"mounted-guide-vision/ablation-sources.json")["omit-lower-left-arm"]["register"]
    new=read(assets/"guide-arm-route/shallow-corridor/view-source.json")
    refs=(assets/"mounted-guide-vision/omit-lower-left-arm-fixed.npz",old["fixedBackgroundSha256"])
    changed=(assets/"guide-arm-route/shallow-corridor/fixed-stereo.npz",new["imageSha256"])
    before,after=[sector.nav.checked_scene(assets,use_guide=False,registered_background=v) for v in (refs,changed)]
    scenes={"reference":before,"plate":after}
    for eye in ("left","right"):
        scenes[eye+"Only"]={**before,eye:after[eye],eye+"_z":after[eye+"_z"]}
    feature=sector.VisibleFasteners(before)
    original=feature.identify
    samples={}
    observations=[]
    def capture(pose,ctx,tag,duplicate,hide):
        result=sector.coarse_observation(pose,ctx,tag,duplicate,hide)
        samples[id(result[1])]=pose.copy()
        return result
    def identify(detections,prediction,ms):
        pose=samples.pop(id(detections))
        reply=original(detections,prediction,ms)
        if ms not in (1785,1820,1855,1890,1925,1960,1995,2030,2065,2100,2135,2170):return reply
        variants={}
        for name,ctx in scenes.items():
            other=sector.VisibleFasteners(ctx)
            _,heads,_,_=sector.coarse_observation(pose,ctx,10,False,False)
            rays=other.identify(heads,prediction,ms)
            detail=other.observations[-1]
            neighbors={}
            _,without_heads,_,_=sector.coarse_observation(pose,ctx,10,False,True)
            for eye_index,eye in enumerate(vision.EYES):
                centers,_=vision.project(ctx["bolts"],prediction,eye)
                for target,center in enumerate(centers):
                    def within(points):
                        return [np.asarray(px).round(3).tolist() for px in points if np.linalg.norm(px-center)<13]
                    neighbors[f"{eye_index}/{target}"]={"predicted":center.round(3).tolist(),
                        "observed":within(heads[eye_index]),"withFastenersHidden":within(without_heads[eye_index])}
            variants[name]={"neighbors":neighbors,"acceptedRays":None if rays is None else [[e,n] for e,n,_ in rays],
                "ambiguous":detail["ambiguous"],"features":detail["features"],"viable":detail["viable"]}
        observations.append({"ms":ms,"physicalPose":pose.tolist(),"prediction":prediction.tolist(),"variants":variants})
        return reply
    feature.identify=identify
    route=read(ROOT/"station-landmark/routes.json")["cases"]["leftSmall"]
    start=read(ROOT/"conditioned-match/slow-navigation.json")["leftSmall"]
    out=assets/"guide-arm-route/pair-audit";out.mkdir(exist_ok=True)
    row=sector.nav.sector.tracecase("coral-A",start["initial"],"nominal",before,out,
        wheel_scale=route["wheelScale"],physical_track_mm=route["effectiveTrackMm"],observer=capture,features=feature)
    prior=next(r for r in read(ROOT/"mounted-guide-vision/ablation-sector.json")
        if r["installation"]=="omit-lower-left-arm" and r["wheelCase"]=="leftSmall")
    if row["motionSHA256"]!=prior["motionSHA256"]:raise ValueError("Reference controller changed")
    return {"referenceRasterSha256":refs[1],"plateRasterSha256":changed[1],"referenceMotionSha256":row["motionSHA256"],
        "observations":observations}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets",type=Path)
    result=run(p.parse_args().assets.resolve())
    (HERE/"pair-audit.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf8",newline="\n")
