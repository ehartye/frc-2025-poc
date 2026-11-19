"""Challenge station landmark association against the complete native station at two reached poses."""

import argparse, math, json, sys
from pathlib import Path
import numpy as np
from plant import HERE, ROOT, read, digest
from attained_views import views, sensor, O, scene, field_mesh

sys.path.insert(0, str(ROOT / "station-landmark"))
import panel
from station_landmarks import Landing


def evaluate(assets, fieldstep):
    if digest(fieldstep) != read(ROOT / "station-landmark/native.json")["fieldSha256"]:
        raise ValueError("Field source changed")
    field = field_mesh(fieldstep)
    fixed, _, _ = views(assets, "nominal", ROOT / "guide-setting", write_labels=False)
    sensor.scenes.fixed_pose(O)
    bias = panel.referenced(assets / "selected-optics/issued/marked")[2]
    rows = []
    for side, start in read(HERE / "revised-opening.json").items():
        path = assets / "straight-guide-match" / (side + "-revised-opening.npz")
        if digest(path) != start["pathSha256"]:
            raise ValueError("Wrong signed wheel poses")
        with np.load(path) as d:
            t, p = d["t"], d["pose"]
        for stage, wall in (
            ("first exposure", start["poseFits"][0][0] - 0.03),
            ("stationary", start["stationWallS"]),
        ):
            where = int(np.argmin(abs(t - wall)))
            x, y, angle = p[where]
            terminal = np.array(start["reachedMmDeg"])
            terminal[2] = math.radians(terminal[2])
            normal = p[-1, 2] - terminal[2] - math.pi
            c, s = math.cos(normal), math.sin(normal)
            dx, dy = [x, y] - p[-1, :2]
            q = terminal + np.array(
                [
                    1000 * (c * dx + s * dy),
                    1000 * (-s * dx + c * dy),
                    math.remainder(angle - p[-1, 2], 2 * math.pi),
                ]
            )
            observation = scene(
                field,
                [-q[0], -q[1], math.degrees(q[2])],
                fixed,
            )
            target = assets / "straight-guide-match/native-panels"
            target.mkdir(parents=True, exist_ok=True)
            packet = target / (side + "-" + stage.replace(" ", "-") + ".npz")
            np.savez_compressed(
                packet,
                left=observation[0][0],
                left_z=observation[0][1],
                right=observation[1][0],
                right_z=observation[1][1],
            )
            imgs = [g for g, _ in observation]
            oracle = panel.pixels(q)[:, :2]
            distances = []
            detected = []
            for eye, img in enumerate(imgs):
                points = np.asarray(panel.components(img)).reshape(-1, 2)
                distances.append(
                    [
                        min(np.linalg.norm(points - v, axis=1), default=float("inf"))
                        for v in oracle[eye]
                    ]
                )
                detected.append(len(points))
            actual = Landing().observe(imgs, None, bias)
            tracked = Landing().observe(imgs, q, bias)
            rows.append(
                dict(
                    side=side,
                    stage=stage,
                    wallS=float(t[where]),
                    poseMmDeg=[q[0], q[1], math.degrees(q[2])],
                    componentCounts=detected,
                    packetSha256=digest(packet),
                    nearestUpperCenterPx=distances,
                    initialMatch=actual is not None,
                    localWindowMatch=tracked is not None,
                    imageSha256=[
                        __import__("hashlib").sha256(v.tobytes()).hexdigest()
                        for v in imgs
                    ],
                )
            )
    return rows


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("assets", type=Path)
    p.add_argument("field", type=Path)
    a = p.parse_args()
    (HERE / "native-panel.json").write_text(
        json.dumps(evaluate(a.assets, a.field), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
