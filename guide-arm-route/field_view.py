"""Re-run the surveyed fastener approach with the six exchanged guide parts in place."""
import argparse
from pathlib import Path
import sys
import hashlib
import json
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_bytes())


def assembled_mesh(station,folder,installation,setting):
    sensor=station.sensor
    kit,gsha,ssha=station.mesh(installation,setting)
    source=read(folder/"revision.json")
    step=folder/"route.step"
    if (source["guideSha256"]!=gsha or source["settingSha256"]!=ssha
            or source["stepSha256"]!=digest(step)):
        raise ValueError("Guide revision provenance differs")
    solids=sensor.mount.cad.read_named(step,[{"id":k} for k in source["volumesMm3"]])
    parts=[]
    seen=set()
    for row in kit["parts"]:
        k=row[0]
        if k in solids:
            solid=solids[k]
            if installation=="outward":
                solid=sensor.mount.cad.translated(solid,(0,.315,0))
            vertices,triangles=sensor.mount.original.triangulate(solid)
            row=[*row[:3],vertices,triangles]
            seen.add(k)
        parts.append(row)
    if seen!=set(solids) or len(parts)!=56:
        raise ValueError("Incomplete replacement in camera model")
    return {"parts":parts},gsha,ssha


def render(vision, assets, layouts, installation):
    suffix="" if installation=="nominal" else "-outward"
    sys.path.insert(0,str(vision.resolve()))
    import station
    base=assets/"observed-reef-a/machined-fixed.npz"
    bridge=read(ROOT/"observed-reef-a/station-bridge.json")
    if digest(base)!=bridge["leftSmall"]["machinedFieldBackgroundSha256"]:
        raise ValueError("The fixed stereo views belong to a different robot")
    sensor=station.sensor
    sensor.scenes.camera=sensor.camera
    sensor.scenes.fixed_pose(station.O)
    robot=sensor.background(base)
    for layout in layouts:
        folder=assets/"guide-arm-route"/layout
        revised,gsha,ssha = assembled_mesh(station,folder,installation,ROOT/"guide-setting")
        parts=revised["parts"]
        step=folder/"route.step"
        layer=sensor.camera.images_for_pose({"parts":parts})
        both=sensor.composite(robot,layer)
        image=folder/("fixed-stereo"+suffix+".npz")
        np.savez_compressed(image,left=both[0][0],left_z=both[0][1],right=both[1][0],right_z=both[1][1])
        imgsha=digest(image)
        receipt={"installation":installation,"stepSha256":digest(step),"baseRobotImageSha256":digest(base),"imageSha256":imgsha,
            "appliedGuideArticles":len(parts),"stationSourceSha256":digest(vision/"station.py")}
        (folder/("view-source"+suffix+".json")).write_text(json.dumps(receipt,indent=2)+"\n",encoding="utf8",newline="\n")


def trace(vision, assets, layouts, modes, installation):
    suffix="" if installation=="nominal" else "-outward"
    sys.path.insert(0,str(vision.resolve()))
    import sector
    routes=read(ROOT/"station-landmark/routes.json")["cases"]
    starts=read(ROOT/"conditioned-match/slow-navigation.json")
    for layout in layouts:
        folder=assets/"guide-arm-route"/layout
        receipt=read(folder/("view-source"+suffix+".json"))
        image=folder/("fixed-stereo"+suffix+".npz")
        if digest(folder/"route.step")!=receipt["stepSha256"]:
            raise ValueError("Geometry changed since camera export")
        ctx=sector.nav.checked_scene(assets,use_guide=False,registered_background=(image,receipt["imageSha256"]))
        trials=[]
        for mode in modes:
            for wheel,entry in starts.items():
                dest=folder/(wheel+suffix)
                dest.mkdir(exist_ok=True)
                calibration=routes[wheel]
                feature=sector.VisibleFasteners(ctx)
                result=sector.nav.sector.tracecase("coral-A",entry["initial"],mode,ctx,dest,
                    wheel_scale=calibration["wheelScale"],physical_track_mm=calibration["effectiveTrackMm"],
                    observer=sector.coarse_observation,features=feature)
                result.pop("path")
                result.update(wheel=wheel,mode=mode)
                trials.append(result)
                print(layout,wheel,mode,result["accepted"],result["reason"],flush=True)
        receipt.update(sectorSourceSha256=digest(vision/"sector.py"),runs=trials)
        (folder/("field"+suffix+".json")).write_text(json.dumps(receipt,indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("stage",choices=("render","trace"))
    p.add_argument("assets",type=Path)
    p.add_argument("layouts",nargs="+")
    p.add_argument("--installation",choices=("nominal","outward"),default="nominal")
    p.add_argument("--vision",type=Path,default=ROOT/"mounted-guide-vision")
    p.add_argument("--modes",nargs="+",default=["nominal"])
    args=p.parse_args()
    if args.stage=="render":
        render(args.vision,args.assets.resolve(),args.layouts,args.installation)
    else:
        trace(args.vision,args.assets.resolve(),args.layouts,args.modes,args.installation)
