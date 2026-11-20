"""Reopen the installed shop issue and recalculate every edge touching a changed solid."""

import argparse
from pathlib import Path
from shop_sources import issue_shapes
from attachments import dist
from installed_audit import HERE, sha, bounds, box_gap, reopen
from issue import rows, document
import json


def check(folder=HERE / "issue"):
    reg = json.loads((folder / "interfaces.json").read_bytes())
    changed = issue_shapes(folder, reg)
    robot = {k: s for k, *_, s in reopen()} | changed
    for k in reg["fixedHeight"]["removed"]:
        robot.pop(k, None)
    inherited = rows(HERE / "neighbors.csv")
    adjacency = {k: set() for k in robot}

    def add(a, b):
        adjacency[a].add(b)
        adjacency[b].add(a)

    for r in inherited:
        a, b = r["first"], r["second"]
        if (
            a not in changed
            and b not in changed
            and a in robot
            and b in robot
            and float(r["gapMm"]) <= 0.020
        ):
            add(a, b)
    bb = {k: bounds(s) for k, s in robot.items()}
    comparisons = 0
    for a, s in changed.items():
        for b, t in robot.items():
            if a == b or (b in changed and b < a) or box_gap(bb[a], bb[b]) > 0.0200001:
                continue
            comparisons += 1
            if dist(s, t) <= 0.0200001:
                add(a, b)
    todo = set(robot)
    components = []
    while todo:
        a = min(todo)
        todo.remove(a)
        pending = [a]
        seen = {a}
        while pending:
            a = pending.pop()
            for b in adjacency[a] - seen:
                seen.add(b)
                todo.remove(b)
                pending.append(b)
        components.append(sorted(seen))
    components.sort(key=lambda row: (-len(row), row))
    document(
        folder / "connections.json",
        dict(
            issue=reg["outputSha256"],
            source=sha(HERE / "neighbors.csv"),
            installed=len(robot),
            broadPhaseChangedPairs=comparisons,
            proximityMm=0.020,
            supportedComponentCount=len(components[0]),
            separateCount=len(components) - 1,
            separateComponents=components[1:],
            scope="Parked native contact candidates only. Separated bearings and noncontact sensors can be intentional; case contact and common solids do not establish positive rated retention. The named isolated groups require explicit physical-owner drawings or a registration change.",
        ),
    )


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("folder", type=Path, nargs="?", default=HERE / "issue")
    check(ap.parse_args().folder)
