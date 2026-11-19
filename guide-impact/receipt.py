"""Reopen issued trajectories and guard the identity of a proposed controller handoff."""

import argparse
import json
from pathlib import Path
import numpy as np
from beam import ROOT, digest


def read(p):
    return json.loads(p.read_bytes())


def collect(group, raw):
    rows = []
    register = read(group)
    for r in register["cases"]:
        folder = raw / r["id"]
        history = folder / "guide.npy"
        body = folder / (r["id"] + ".npy")
        state = folder / "state.npz"
        if [digest(p) for p in (history, body, state)] != [
            r[k] for k in ("guideHistorySha256", "trajectorySha256", "stateSha256")
        ]:
            raise ValueError("Unregistered guide, pipe or departure data")
        trace = np.load(body, mmap_mode="r")
        with np.load(state, allow_pickle=False) as d:
            for k, sl, scale in (
                ("p", slice(1, 4), 1000.0),
                ("a", slice(4, 7), 1.0),
                ("v", slice(7, 10), 1.0),
            ):
                np.testing.assert_array_equal(d[k] * scale, trace[-1, sl])
            if (
                r["lastState"] == "IN_BAND"
                and abs(100 + 1000 * float(d["time"]) - r["lastJavaMs"]) > 1e-7
            ):
                raise ValueError("Port tick and final pipe state have different clocks")
            modes = len(d["z"])
        obs = {
            key: r[key]
            for key in (
                "id",
                "case",
                "poseMmDeg",
                "layout",
                "stepUs",
                "deadlineMs",
                "lastJavaMs",
                "firstBandMs",
                "lastState",
                "reason",
                "loss",
                "hardStop",
                "portTransitions",
                "peakShelfNormalN",
                "damperWorkJ",
                "firstShelfContact",
                "finalPositionMm",
                "finalAxis",
                "beltMps",
                "guide",
                "trajectorySha256",
                "guideHistorySha256",
                "stateSha256",
            )
        }
        obs["programSha256"] = register["controller"]
        if (
            r.get(
                "programSha256",
                read(ROOT / "station-landmark/feed.json")["programSha256"],
            )
            != register["controller"]
        ):
            raise ValueError("Individual result has different program")
        obs["outwardPerSideMm"] = r.get("outwardPerSideMm", 0.0)
        obs["carriageSupport"] = r.get("carriageSupport", {})
        if obs["carriageSupport"]:
            obs["modalTables"] = {
                k: r[k]
                for k in (
                    "shelfDisplacementSlopeTable",
                    "guideDisplacementTable",
                    "guideSlopeTable",
                )
            }
        if r["lastJavaMs"] >= 1500:
            index = int(np.argmin(abs(trace[:, 0] - 1500)))
            if abs(trace[index, 0] - 1500) > 1e-6:
                raise ValueError("Missing common acquisition sample")
            obs["pipeAtJava1500Mm"] = trace[index, 1:4].tolist()
        obs["shelfModeCount"] = modes
        obs["guideModel"] = [
            r["binding"]["layout"],
            r["binding"]["cutoffHz"],
            r["binding"]["movingArmMassFraction"],
        ]
        if obs["layout"] != "rigid":
            a = np.load(history, mmap_mode="r")
            if a.ndim != 2 or a.shape[1] != 15 or not np.all(np.diff(a[:, 0]) > 0):
                raise ValueError("Guide force and work record is incomplete")
            contact = (a[:, 1:3] > 0).any(axis=1)
            obs["contactSamples"] = int(contact.sum())
            obs["guideSamples"] = len(a)
            obs["peakForces"] = []
            for j in range(2):
                ix = int(np.argmax(a[:, 1 + j]))
                obs["peakForces"].append(
                    {
                        "timeAfterLipMs": float(a[ix, 0] * 1000),
                        "forceN": float(a[ix, 1 + j]),
                        "heightMm": float(a[ix, 3 + j] * 1000),
                    }
                )
            obs["positiveSupportInputJ"] = (
                np.maximum(
                    np.diff(np.vstack([np.zeros((1, 2)), a[:, -2:]]), axis=0), 0.0
                )
                .sum(axis=0)
                .tolist()
            )
            obs["negativeSupportInputJ"] = (
                np.minimum(
                    np.diff(np.vstack([np.zeros((1, 2)), a[:, -2:]]), axis=0), 0.0
                )
                .sum(axis=0)
                .tolist()
            )
            obs["peakStoredGuideJ"] = a[:, 11:13].max(axis=0).tolist()
            obs["maxFaceLowerUpperMm"] = (
                1000 * np.max(abs(a[:, 5:9]), axis=0)
            ).tolist()
        # Java custody is required for the optical handoff. Actual images must be taken separately.
        obs["eligibleForStationImageReplay"] = (
            obs["lastJavaMs"] == 5460
            and obs["lastState"] == "IN_BAND"
            and obs["beltMps"] == 0.0
            and not obs["loss"]
            and not obs["hardStop"]
        )
        rows.append(obs)
    return rows


def run(groups, raw):
    allrows = []
    models = {}
    source = {}
    for file in groups:
        run = read(file)
        for k in ("stationSource", "shelfBasis", "upstreamTrajectories"):
            if k in source and source[k] != run[k]:
                raise ValueError("Mixed station sources")
            source[k] = run[k]
        allrows.extend(collect(file, raw))
        for r in run["cases"]:
            b = r["binding"]
            key = f"{b['layout']}/{b['cutoffHz']:g}/{b['movingArmMassFraction']:g}"
            if key in models and models[key] != b:
                raise ValueError("Beam source changed")
            models[key] = b
    if len({r["id"] for r in allrows}) != len(allrows):
        raise ValueError("Repeated controller run")
    return {
        **source,
        "hardware": read(Path(__file__).with_name("section.json")),
        "guideModels": models,
        "cases": allrows,
        "measuredGuideCompliance": None,
        "measuredAcquisition": None,
        "cameraCustody": None,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("raw", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("groups", nargs="+", type=Path)
    a = p.parse_args()
    a.out.write_text(
        json.dumps(run(a.groups, a.raw), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
