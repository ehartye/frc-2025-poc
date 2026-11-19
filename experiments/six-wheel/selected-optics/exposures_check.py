"""Re-render station and placement exposures from selected native backgrounds and measured trajectory files."""

import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
MATCH = ROOT / "relieved-match"
sys.path.insert(0, str(MATCH))
import exposures
from sight import camera, optics, bridge_fit, background, silhouette
import observe
from scenes import fixed_pose


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def frames(piece, cache):
    label = "first" if piece == "coral-A" else "return"
    source = read(MATCH / (label + "-feed.json"))["cases"][0]
    cargo = read(MATCH / (piece + "-pads-edge-250.json"))
    trace = cache / (piece + "-pads-edge-250.npy")
    known = cache / (label + "-nominal.npy")
    if sha(trace) != cargo["rawSha256"] or sha(known) != source["trajectorySHA256"]:
        raise ValueError("Unregistered pipe motion")
    path, pickup = np.load(trace), np.load(known)
    with (cache / "landing-field/trace.csv").open() as f:
        rows = [r for r in csv.DictReader(f) if r["case"] == piece + "/250/extended"]
    target = np.array(
        [
            [
                float(r[k])
                for k in ("t_s", "x_m", "y_m", "z_m", "axis_x", "axis_y", "axis_z")
            ]
            for r in rows
        ]
    )
    return label, source, cargo, path, pickup, target


def fit_row(img, ref, moving, stroke=370):
    if moving:
        k = bridge_fit.fit(img, ref)
        fields = ("x_mm", "y_mm", "bottom_mm")
    else:
        k = observe.read_pair(img, [g for g, _ in ref], 560, stroke)
        fields = ("u_interval_mm", "y_interval_mm", "bottom_relative_to_shelf_mm")
    return [k["status"], *[v for field in fields for v in k.get(field, [])]]


def run(args):
    piece, head = args.piece, args.head
    label, source, cargo, path, pickup, target = frames(piece, args.cache)
    reg = read(args.scenes / "scenes.json")
    position = read(HERE / "mount.json")["registrations"]

    def scene(name):
        f = args.scenes / (name + ".npz")
        if sha(f) != reg["backgroundSha256"][name]:
            raise ValueError("Selected optical background changed")
        return background(f)

    moving = head == "moving"
    release = read(MATCH / (piece + "-fixed.json"))
    stamp = release["releaseCameraMs"]
    end = release["touchCameraMs"]
    station = read(MATCH / "station-images.json")["cases"][label + "-nominal"]["frames"]
    times = (
        [t for t, *_ in station]
        if head == "seat"
        else list(
            range(
                17
                * math.ceil((source["firstPortBandMs"] - 51 if moving else stamp) / 17),
                int(end + 35) + 1,
                17,
            )
        )
    )
    if args.critical:
        times = [
            t
            for t in times
            if (
                source["firstPortBandMs"] - 51 <= t <= source["firstPortBandMs"] + 170
                or abs(t - source["scheduledDriveCameraMs"]) < 51
                or stamp - 70 <= t
                or (moving and t % 850 == 0)
                or (head == "seat" and t in (station[0][0], station[-1][0]))
            )
        ]
    if moving:
        dz = {"barrel": 0.0, "rear": 0.75, "front": -0.75}[args.origin]
        physical = np.array(position["roof"]["assumedRectifiedOriginMm"]) + [0, 0, dz]
        optics.ORIGIN = physical.copy()
        optics.configure()
        prior = scene(args.origin + "-moving-reference")
        bgs = {s: scene(args.origin + "-moving-" + str(s)) for s in (0, 370)}
        # Compare with each stationary pose's true empty intensity. Only slide-fixed robot material
        # behind that pixel can support a moving contour. Endpoint images remain a finite challenge.
        refs = {
            s: [(g, b) for (g, _), (_, b) in zip(bg, prior)] for s, bg in bgs.items()
        }
        if args.swept:
            sweep = read(args.scenes / "sweep.json")
            guard_path = args.scenes / "barrel-moving-swept.npz"
            if sha(guard_path) != sweep["backgroundSha256"]:
                raise ValueError("Unregistered swept view")
            bgs[185] = refs[185] = background(guard_path)
            if args.occlusion:
                near_file = args.scenes / "barrel-moving-nearest.npz"
                if sha(near_file) != sweep["nearestPossibleSha256"]:
                    raise ValueError("Changed swept foreground")
                # The unclipped native reference carries only the surveyed shelf contrast edges.
                # evidence.py independently checks their coordinates against the guarded reference.
                worst = background(near_file)
                best = prior
                bgs[185] = [(g, z) for (g, _), (_, z) in zip(prior, worst)]
                refs[185] = best
    else:
        import scenes

        scenes.camera = camera
        physical = np.array(
            position["fixed-datum-up"]["assumedRectifiedOriginMm"]
        ) + reg.get("fixedDepthOffsetMm", 0.0) * np.array(
            position["fixed-datum-up"]["outwardAxis"]
        )
        fixed_pose(physical)
        observe.ORIGIN = physical.copy()
        bgs = (
            {0: scene(label + "-fixed")}
            if head == "seat"
            else {370: scene(piece + "-fixed")}
        )
        refs = bgs
    rows = []
    detail = {}
    memo = {}
    ambiguous = {}
    for tick in times:
        point, axis, stroke = exposures.state_at(
            piece, tick, path, source, cargo, pickup, target
        )
        station_pose = head == "seat" or tick < source["scheduledDriveCameraMs"]
        if head == "seat" and tick >= source["scheduledDriveCameraMs"]:
            point = np.array(
                [np.interp(tick, pickup[:, 0], col) for col in pickup[:, 1:4].T]
            )
            axis = np.array(
                [np.interp(tick, pickup[:, 0], col) for col in pickup[:, 4:7].T]
            )
            axis /= np.linalg.norm(axis)
            stroke = 0.0
        samples = [0 if station_pose else 370]
        if moving and 0.001 < stroke < 369.999:
            samples = [185] if args.swept else [0, 370]
        if moving and stroke <= 0.001:
            samples = [0]
        if not moving:
            samples = list(bgs)
            point[0] += stroke
        truth = [point[0] - (0 if moving else stroke), point[1], point[2] - 557.15]
        values = []
        for sample in samples:
            observer_stroke = 0 if head == "seat" else 370
            key = (sample, tuple(np.round(point, 6)), tuple(np.round(axis, 9)))
            if key not in memo:
                bg = bgs[sample]
                ref = refs[sample]
                exposure = silhouette(bg, point, axis)
                if args.occlusion and sample == 185:
                    clear = silhouette(ref, point, axis)
                    changed = [
                        int(np.count_nonzero((a != b) & (d < 3490)))
                        for a, b, (_, d) in zip(clear, exposure, ref)
                    ]
                    ambiguous[tick] = changed
                if args.control == "stale-roof":
                    # Generate pixels from the installed head, then use the old survey for inversion.
                    optics.ORIGIN = np.array([48.0, 0.0, 842.0])
                if args.control == "swapped-eyes":
                    exposure = exposure[::-1]
                if args.control == "lost-eye":
                    exposure[1] = bg[1][0].copy()
                memo[key] = fit_row(exposure, ref, moving, observer_stroke)
                if args.occlusion and sample == 185 and any(changed):
                    coords = [
                        (eye, int(y), int(x))
                        for eye, (a, b, (_, d)) in enumerate(zip(clear, exposure, ref))
                        for y, x in np.argwhere((a != b) & (d < 3490))
                    ]
                    if len(coords) > 12:
                        memo[key] = ["unbounded-occlusion"]
                    else:
                        lohi = memo[key].copy()
                        for variant in range(1, 1 << len(coords)):
                            trial = [image.copy() for image in exposure]
                            for i, (eye, y, x) in enumerate(coords):
                                if variant & (1 << i):
                                    trial[eye][y, x] = clear[eye][y, x]
                            outcome = fit_row(trial, ref, True)
                            if (
                                len(outcome) != 7
                                or len(lohi) != 7
                                or outcome[0] != lohi[0]
                            ):
                                lohi = ["unbounded-occlusion"]
                                break
                            for j in range(1, 7):
                                lohi[j] = (min if j % 2 else max)(lohi[j], outcome[j])
                        memo[key] = lohi
                if moving:
                    optics.ORIGIN = physical.copy()
            values.append(memo[key])
        r = values[0]
        for sample_value in values[1:]:
            if r[0] != sample_value[0] or len(r) != len(sample_value):
                r = ["endpoint-disagreement"]
                break
            if len(r) == 7:
                r = [
                    r[0],
                    *[
                        min(r[i], sample_value[i])
                        if i % 2
                        else max(r[i], sample_value[i])
                        for i in range(1, 7)
                    ],
                ]
        error = None
        if len(r) == 7:
            error = max(
                0.0,
                *(max(r[i + 1] - q, q - r[i + 2]) for i, q in zip((0, 2, 4), truth)),
            )
        rows.append([piece, head, tick, *r])
        if (
            stamp - 70 <= tick
            or source["firstPortBandMs"] <= tick <= source["firstPortBandMs"] + 136
        ):
            detail[tick] = {"bounds": r, "truthMm": truth, "outsideMm": error}
    args.out.mkdir(exist_ok=True, parents=True)
    tag = "-".join(
        (piece, head, args.origin, args.control, "critical" if args.critical else "all")
    ) + ("-bounded" if args.occlusion else "-guarded" if args.swept else "")
    csv_path = args.out / (tag + ".csv")
    with csv_path.open("w", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(
            [
                "piece",
                "head",
                "capture_ms",
                "status",
                "x_lo",
                "x_hi",
                "y_lo",
                "y_hi",
                "bottom_lo",
                "bottom_hi",
            ]
        )
        writer.writerows(rows)
    result = {
        "pipe": piece,
        "head": head,
        "origin": args.origin,
        "fixedDepthOffsetMm": reg.get("fixedDepthOffsetMm", 0.0),
        "control": args.control,
        "frames": len(rows),
        "uncertainBrightPixelCounts": ambiguous,
        "statuses": {
            k: sum(r[3] == k for r in rows) for k in sorted({r[3] for r in rows})
        },
        "frameCsvSha256": sha(csv_path),
        "localClockEpochWallUs": read(MATCH / (label + "-feed.json"))["epochWallUs"],
        "sourceTrajectorySha256": cargo["rawSha256"],
        "sourceFeedSha256": source["trajectorySHA256"],
        "selectedFrames": detail,
    }
    (args.out / (tag + ".json")).write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    print(tag, result["frames"], result["statuses"], flush=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for arg in ("scenes", "cache", "out"):
        p.add_argument(arg, type=Path)
    p.add_argument("piece", choices=["coral-A", "coral-B"])
    p.add_argument("head", choices=["seat", "fixed", "moving"])
    p.add_argument("--origin", choices=["barrel", "front", "rear"], default="barrel")
    p.add_argument(
        "--control",
        choices=["nominal", "stale-roof", "lost-eye", "swapped-eyes"],
        default="nominal",
    )
    p.add_argument("--critical", action="store_true")
    p.add_argument("--swept", action="store_true")
    p.add_argument("--occlusion", action="store_true")
    a = p.parse_args()
    if a.control == "stale-roof" and a.head != "moving":
        p.error("Stale roof requires moving head")
    if a.occlusion:
        a.swept = True
    if a.swept and (a.head != "moving" or a.origin != "barrel"):
        p.error("Swept verification requires the center roof datum")
    run(a)
