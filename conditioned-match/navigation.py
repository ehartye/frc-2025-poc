"""Register the attained first pipe and chassis to the loaded wheel path and both reef cameras."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "selected-drive"))
import route
import dock as sector
from navigation import corrected, placed_route
from scene import context
from vision import C, S, F, EYES


def read(path):
    return json.loads(Path(path).read_bytes())


def sha(path):
    with Path(path).open("rb") as file:
        return hashlib.file_digest(file, "sha256").hexdigest()


def checked_scene(assets, use_guide=True):
    plan = read(ROOT / "station-guide/optics-narrow.json")
    raw = assets / "station-guide/selected-background/stowed-robot-fixed.npz"
    if sha(raw) != plan["registeredReceiverDepthSha256"]:
        raise ValueError("Shop camera does not match the station trial")
    sys.path.insert(0, str(ROOT / "station-guide"))
    from optics import depth
    ctx = context(assets / "sector-alignment")
    data = np.load(raw)
    bounds = read(ROOT / "sector-alignment/scene.json")["carryBoxMm"]
    v, u = np.indices(data["left"].shape)
    up = (240 - v) / F
    direction = (C + up*S, (320-u)/F, -S + up*C)
    for eye, key in zip(EYES, ("left", "right")):
        z = data[key + "_z"]
        guide = depth(eye, 0, 3.8, 40, 20)
        if not use_guide:
            guide = np.full(z.shape, np.inf)
        image = np.where(guide < z, 160, data[key])
        z = np.minimum(guide, z)
        near = np.full(z.shape, -np.inf)
        far = np.full(z.shape, np.inf)
        with np.errstate(divide="ignore", invalid="ignore"):
            for i, (origin, ray) in enumerate(zip((-315, eye, 970), direction)):
                a, b = (bounds[i]-origin)/ray, (bounds[i+3]-origin)/ray
                near = np.maximum(near, np.minimum(a,b))
                far = np.minimum(far, np.maximum(a,b))
        occludes = (near > 0) & (far >= near) & (near < z)
        ctx[key] = np.where(occludes, 170, image).astype(np.uint8)
        ctx[key + "_z"] = np.where(occludes, near, z)
    return ctx


def simulate_transit(before, scale, width):
    t = np.linspace(0, before.duration, math.ceil(before.duration/.005)+1)
    nominal = np.array([before.at(x)[0] for x in t])
    pose = route.integrate(nominal[0], route.wheel_steps(nominal), scale, width/1000)
    return t, pose


def outbound(piece, source, wheel_case, cache, ctx, use_gyro=False, pace=1.):
    target, before, _ = placed_route(piece, source)
    marker = target.segments[-1]["start"]
    cfg = read(ROOT / "station-landmark/routes.json")["cases"][wheel_case]
    if use_gyro:
        from heading import drive
        _, planned, _ = placed_route(piece, [0., 0., 0.])
        times, poses, wheels = drive(planned, before.start, np.array(cfg["wheelScale"]), cfg["effectiveTrackMm"],
                                     cfg["gyroBiasDegS"], cfg["gyroScaleError"], pace)
    else:
        times, poses = simulate_transit(before, cfg["wheelScale"], cfg["effectiveTrackMm"])
        wheels = route.wheel_steps(poses)*1000/(times[1]-times[0])
    endpoint = poses[-1]
    observed = [1000*(endpoint[0]-marker[0]), 1000*(endpoint[1]-marker[1]),
                math.degrees(corrected().short(endpoint[2]-marker[2]))]
    cache.mkdir(exist_ok=True, parents=True)
    np.savez_compressed(cache/"transit.npz", times=times, poses=poses, wheels_mm_s=wheels)
    result = sector.tracecase(piece, observed, "nominal", ctx, cache, wheel_scale=cfg["wheelScale"],
                              physical_track_mm=cfg["effectiveTrackMm"])
    result.pop("path")
    result.update(stationPoseMmDeg=source, wheelScale=cfg["wheelScale"], trackMm=cfg["effectiveTrackMm"],
                  transitDurationS=float(times[-1]), gyroFeedback=use_gyro, pace=pace, transitSha256=sha(cache/"transit.npz"))
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets", type=Path)
    args = p.parse_args()
    ctx = checked_scene(args.assets)
    source = read(ROOT / "shelf-contact/fine-ports.json")["results"]
    rows = {}
    for case in ("leftSmall", "rightSmall"):
        r = next(x for x in source if x["case"] == case and x["dtS"] == .000015625)
        rows[case] = {}
        for variant in ("shaft", "gyro", "monocular-seed"):
            row = outbound("coral-A", r["sourceRobotMmDeg"], case, args.assets/"conditioned-match"/case/variant/"A",
                           dict(ctx,allowSingleEyeId=(variant == "monocular-seed")), use_gyro=(variant != "shaft"))
            rows[case][variant] = row
            print(case, variant, row["accepted"], row["initial"], row["reason"], flush=True)
    for side,row in read(HERE/"opening.json").items():
        for variant,single_eye in (("corrected",False),("corrected-mono",True)):
            rows[side][variant] = outbound("coral-A",row["sourceRobotMmDeg"],side,
                args.assets/"conditioned-match"/side/variant/"A",dict(ctx,allowSingleEyeId=single_eye),use_gyro=True)
    (HERE/"navigation.json").write_text(json.dumps(rows, indent=2)+"\n", encoding="utf8", newline="\n")
