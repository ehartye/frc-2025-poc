"""Induce the contact graph on precisely the public orbit and compare the scored fixture."""

import csv
import hashlib
import json
from collections import deque
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
ENG = ROOT / "cad/assembly-engineering"


def read(p):
    return json.loads(p.read_bytes())


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def groups(adjacency):
    remaining = set(adjacency)
    islands = []
    while remaining:
        first = min(remaining)
        queue = deque([first])
        remaining.remove(first)
        node = []
        while queue:
            key = queue.popleft()
            node.append(key)
            extras = adjacency[key] & remaining
            remaining.difference_update(extras)
            queue.extend(sorted(extras))
        islands.append(sorted(node))
    return sorted(islands, key=lambda g: (-len(g), g[0]))


def run():
    config = read(ROOT / "cad-showcase.json")
    receipt = read(ROOT / "docs/robot/cad/receipt.json")
    registration = read(ROOT / config["appearances"])
    step = sha(ROOT / config["source"])
    if any(
        v != step
        for v in (
            registration["sourceSha256"],
            receipt["sourceSha256"],
            read(ENG / "audit.json")["baseStepSha256"],
        )
    ):
        raise ValueError("Showcase STEP does not match the pairwise native-body audit")
    nodes = {
        r["id"]
        for r in registration["solids"]
        if r["motionGroup"] != "Demonstration CORAL"
    }
    with (ENG / "installed-bodies.csv").open(encoding="utf8") as f:
        all_ids = {r["id"] for r in csv.DictReader(f)}
    if not nodes <= all_ids or len(nodes) != len(registration["solids"]) - 1:
        raise ValueError("Different physical export")
    adjacency = {k: set() for k in nodes}
    with (ENG / "neighbors.csv").open(encoding="utf8") as f:
        for r in csv.DictReader(f):
            a, b = r["first"], r["second"]
            if a in nodes and b in nodes and float(r["gapMm"]) <= 0.020:
                adjacency[a].add(b)
                adjacency[b].add(a)
    components = groups(adjacency)
    anchor = "host-drilled-swerve-deck"
    anchored = next(c for c in components if anchor in c)
    ids = [
        "grip-" + side + "-front-" + s
        for side in ("R", "L")
        for s in ("support-foot", "riser")
    ]
    final = read(ENG / "receiver-service/issue/connections.json")
    installed = read(ENG / "receiver-service/issue/engineering-reopen.json")
    service = {i for c in final["separateComponents"] for i in c}
    actual = {b["identity"] for b in installed["bodies"] if b["instance"] is not None}
    for i in ids:
        if (
            hashlib.sha256(("team-1780-engineering-v1/" + i).encode()).hexdigest()
            not in actual
        ):
            raise ValueError("Missing actual front support " + i)
    return dict(
        showcaseSource=config["source"],
        showcaseStep=step,
        showcaseMesh=receipt["meshSha256"],
        nodes=len(nodes),
        originalCheckedBodies=len(all_ids),
        neighborPairsFile=sha(ENG / "neighbors.csv"),
        unionThresholdMm=0.020,
        base=anchor,
        baseComponentSize=len(anchored),
        components=len(components),
        outsideBase=[c for c in components if anchor not in c],
        publicRisersAndFeet={
            i: dict(withBase=i in anchored, neighbors=sorted(adjacency[i])) for i in ids
        },
        engineeringSource=final["issue"],
        engineeringArchive=installed["archiveSha256"],
        engineeringBodies=final["installed"],
        engineeringMainComponent=final["supportedComponentCount"],
        engineeringOutsideComponents=final["separateCount"],
        engineeringRisersAndFeetInMain={i: i not in service for i in ids},
        scope="Actual undirected CAD proximity at the parked pose, including display/fastener geometry. Reuses all cold pair measurements on the identical public STEP and discards added jack bodies before traversing. Bearing clearances and unmodeled cable connections can form islands. Geometric proximity alone cannot certify a structural attachment, strength or joint freedom.",
    )


if __name__ == "__main__":
    (HERE / "export-graph.json").write_bytes(
        (json.dumps(run(), indent=2) + "\n").encode()
    )
