"""Bind the three independently initialized scenes in the published receiver film to their source observations."""

import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def read(name):
    return json.loads((HERE / name).read_bytes())


def ref(name):
    return {
        "file": (HERE / name).relative_to(ROOT).as_posix(),
        "sha256": hashlib.sha256((HERE / name).read_bytes()).hexdigest(),
    }


def save(name, value):
    (HERE / name).write_text(
        json.dumps(value, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    return ref(name)


def build():
    m = read("motion-track.json")
    attempt = read("candidate.json")["feeds"]["cam-station-plus-31.25us-0.1mm"]
    landing = read("landing.json")
    contact = next(r for r in landing["cases"] if r["case"] == m["case"])
    if (
        attempt["pipeTrajectorySha256"] not in m["sources"].values()
        or contact["outcome"] != "supported"
    ):
        raise ValueError("Motion and observations must refer to the filmed cases")
    projects = json.loads((ROOT / "highlights.json").read_bytes())
    pins = [ref(k) for k in ("motion-track.json", "candidate.json", "landing.json")]
    starts = []
    events = []
    chapters = []
    wall = 0.0
    for index, (key, label, speed) in enumerate(
        [
            ("station", "Station pickup", 2),
            ("extend", "Extend at the supplied dock", 1),
            ("release", "Release into the trough", 1),
        ]
    ):
        a, b = m[key][0], m[key][-1]
        identity = dict(
            revision=m["geometrySha256"],
            attempt=key,
            piece=m["piece"],
            clock="chapter seconds",
            frame="robot",
            unit="mm",
        )
        start = len(starts)
        starts.extend([{**identity, "timeS": v[0], "position": v[2:5]} for v in (a, b)])
        duration = (b[0] - a[0]) / speed
        chapter = dict(
            id=key,
            label=label,
            start=f"/rows/{start}",
            end=f"/rows/{start + 1}",
            sync=[dict(clipS=wall, runS=a[0]), dict(clipS=wall + duration, runS=b[0])],
            events=[],
        )
        if index:
            chapter["join"] = {"kind": "fixture"}

        def event(kind, event_type, when, qualified, target, criterion, rule):
            k = len(events)
            events.append(
                {
                    **identity,
                    "timeS": when,
                    "type": event_type,
                    "kind": kind,
                    "qualified": qualified,
                    "target": target,
                    "criterion": criterion,
                    "rule": rule,
                }
            )
            chapter["events"].append(
                dict(
                    id=f"{key}-{k}",
                    type=event_type,
                    pointer=f"/events/{k}",
                    timeS=when,
                    outcome="unresolved"
                    if qualified is None
                    else "reached"
                    if qualified
                    else "missed",
                )
            )

        if key == "station":
            event(
                "target",
                "seat",
                attempt["javaFirstBandMs"] / 1000,
                True,
                "Pipe seated",
                "The issued receiver program first entered its seated band.",
                "Station fixture with recovered pipe identity",
            )
        if key == "release":
            event(
                "target",
                "support",
                contact["elapsed_s"] - contact["final_dwell_s"] + 2.0,
                True,
                "Two seconds of field support",
                "The finite-pipe trial recorded two seconds of paired floor and lip support.",
                "Independent dock release fixture",
            )
            event(
                "score",
                "tally",
                b[0],
                None,
                "Match delivery",
                "The dock fixture starts independently of the station pickup.",
                "One traced match delivery is required",
            )
        chapters.append(chapter)
        wall += round((duration + 0.65) * 20) / 20
    trace = save("receiver-chapter-states.json", {"sources": pins, "rows": starts})
    observed = save("receiver-chapter-events.json", {"sources": pins, "events": events})
    return save(
        "receiver-chapters.json",
        dict(
            schema="agent-frc/replay-chapters@1",
            revision=projects["revision"],
            media=ref("receiver-motion.webm"),
            trace=trace,
            observations=observed,
            chapters=chapters,
        ),
    )


if __name__ == "__main__":
    build()
