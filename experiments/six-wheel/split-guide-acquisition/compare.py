"""Join the signed histories to actual Java samples and show the interval where a jam looks identical."""

import argparse, json, hashlib
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def read(p):
    return json.loads(p.read_bytes())


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def sample(trace, t):
    i = int(np.searchsorted(trace[:, 0], t - 1e-6))
    if i >= len(trace) or abs(trace[i, 0] - t) > 1e-6:
        raise ValueError("Missing requested sensor time")
    return trace[i]


def table(old_raw, new_raw):
    original = read(ROOT / "compliant-carriage/series.json")["cases"]
    out = {}
    plot = {}

    def old_ticks(r):
        return [
            [t, *next(a for a in reversed(r["portTransitions"]) if a[0] <= t)[1:]]
            for t in range(0, r["lastJavaMs"] + 1, 20)
        ]

    runs = [
        (
            "issued",
            r,
            old_raw
            / "powered-dock/two-stages"
            / r["case"]
            / f"us-{r['stepS'] * 1e6:g}",
            old_ticks(r),
        )
        for r in original
    ]
    for dataset in ("candidate", "locked", "traction"):
        for r in read(HERE / (dataset + ".json"))["cases"]:
            runs.append(
                (
                    dataset,
                    r,
                    new_raw / r["id"] / r["side"] / f"us-{r['stepS'] * 1e6:g}",
                    r["portTicks"],
                )
            )
    for group, r, folder, ticks in runs:
        (path,) = folder.glob("*.npy")
        if digest(path) != r["trajectorySha256"]:
            raise ValueError("Wrong registered route")
        data = np.load(path, mmap_mode="r")
        samples = []
        for t, state, entry, seat, command in ticks:
            if t < 120:
                continue
            if data[-1, 0] < t - 1e-5:
                break
            a = sample(data, t)
            samples.append(
                {
                    "ms": t,
                    "entry": entry,
                    "seat": seat,
                    "controller": state,
                    "beltDemandMps": command,
                    "xMm": float(a[1]),
                    "vxMps": float(a[7]),
                }
            )
        side = r.get("side", r.get("case"))
        key = f"{group}/{side}/{r['stepS'] * 1e6:g}"
        result = {
            "trajectory": digest(path),
            "signedCase": side,
            "controller": ticks[-1][1],
            "firstBandJavaMs": r.get("firstBandJavaMs", r.get("firstPortBandMs")),
            "lastTick": ticks[-1][0],
            "lastPipeCenterMm": data[-1, 1:4].tolist(),
            "hardStop": r.get("terminal", {}).get("hard", r.get("hardwareStop")),
        }
        if group != "issued":
            prior = next(
                x
                for x in runs
                if x[0] == "issued"
                and x[1]["case"] == side
                and x[1]["stepS"] == r["stepS"]
            )
            prefix = np.load(next(prior[2].glob("*.npy")), mmap_mode="r")
            n = min(len(data), len(prefix))
            diff = np.flatnonzero(np.any(data[:n] != prefix[:n], axis=1))
            result["firstChangedPhysicalSampleMs"] = (
                float(data[diff[0], 0]) if len(diff) else None
            )
            result["identicalPriorThroughMs"] = float(
                data[diff[0] - 1 if len(diff) else n - 1, 0]
            )
        out[key] = result
        plot[key] = samples
    for trial in ("locked", "traction"):
        for step in ("31.25", "15.625"):
            a = plot["candidate/leftSmall/" + step]
            b = plot[trial + "/leftSmall/" + step]
            first = next(
                (
                    (x, y)
                    for x, y in zip(a, b)
                    if x["entry"] != y["entry"] or x["seat"] != y["seat"]
                ),
                None,
            )
            row = out[trial + "/leftSmall/" + step]
            row["firstDistinctObservedPortMs"] = first[0]["ms"] if first else None
    return out, plot


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("old_raw", type=Path)
    p.add_argument("new_raw", type=Path)
    args = p.parse_args()
    result, plot = table(args.old_raw, args.new_raw)
    (HERE / "comparison.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf8", newline="\n"
    )
    sizes = {}
    for k, rows in plot.items():
        sizes[k] = [
            [
                r["ms"],
                r["entry"],
                r["seat"],
                r["controller"],
                r["beltDemandMps"],
                round(r["xMm"], 3),
                round(r["vxMps"], 4),
            ]
            for r in rows
        ]
    (HERE / "timing.html").write_text(
        (HERE / "timeline.template.html")
        .read_text(encoding="utf8")
        .replace("/*DATA*/", json.dumps(sizes, separators=(",", ":"))),
        encoding="utf8",
        newline="\n",
    )
