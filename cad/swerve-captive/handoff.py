"""Build the public Fusion inventory from the manufacturing exchange, then verify the independent reopen."""

from pathlib import Path
from argparse import ArgumentParser
import hashlib
import json

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
REVISION = "R4 · fixed-height swerve · red alliance"


def read(path):
    return json.loads(path.read_bytes())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def emit(path, data):
    path.write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n",
        encoding="utf8",
        newline="\n",
    )


def build(exchange_path):
    source = read(exchange_path)
    historical = {
        r["id"]: r
        for r in read(HERE.parent / "full-swerve-r4/appearances.json")["solids"]
    }
    ro = HERE / "student-reopen.json"
    obs = read(ro)
    archive = ROOT / obs["source"]
    if digest(archive) != obs["sha256"] or not obs["opened"]:
        raise ValueError("Download differs from independently reopened bytes")
    claims = []
    for group, names in sorted(source["groups"].items()):
        parent = group + ":1"
        for name in sorted(names):
            key = parent + "+" + name + ":1"
            original = historical.get(name, {})
            rep = original.get("representation", "project-cad")
            if "-display-" in name:
                role = "drive"
            elif group in (
                "Outer slide",
                "Middle slide",
                "Left capture yoke",
                "Right capture yoke",
            ):
                role = "mechanism"
            elif original.get("role") in (
                "frame",
                "structure",
                "chassis",
            ) or name.startswith("frame-"):
                role = "frame"
            else:
                role = "other"
            provenance = original.get(
                "source", "Project capture tooling and nominal game piece drawing"
            )
            if name in ("r6-polymer-facing", "r6-receiver-insert"):
                provenance = "Project machined receiver and underside cam drawing"
            physical = obs["components"].get(key)
            if not physical or physical["parent"] != parent or physical["bodies"] != 1:
                raise ValueError(f"Missing native instance {key}")
            claims.append(
                dict(
                    id=key,
                    parent=parent,
                    role=role,
                    representation="project-proxy"
                    if rep
                    in (
                        "registered-envelope",
                        "public-module-representation",
                        "public-supplier-envelope",
                    )
                    or group == "Demonstration CORAL"
                    else "project-original",
                    provenance=provenance,
                )
            )
    if sum(v["bodies"] for v in obs["components"].values()) != len(claims):
        raise ValueError("Unregistered solid")
    directions = {}
    joints = []

    def joint(name, mode, one, two, axis=None):
        claim = dict(
            id=name, type=mode, occurrenceOne=one + ":1", occurrenceTwo=two + ":1"
        )
        if axis:
            directions[name] = dict(frame=two + ":1", axis=axis, sign=1)
        joints.append(claim)

    for corner in ("FL", "FR", "RL", "RR"):
        mount = corner + " module base"
        fork = corner + " steering fork"
        wheel = corner + " tread"
        joint(corner + " module mounting", "rigid", mount, "Fixed chassis")
        joint(corner + " steer", "revolute", fork, mount, [0, 0, 1])
        joint(corner + " roll", "revolute", wheel, fork, [0, 1, 0])
    for name, one, two, axis in [
        ("Receiver middle", "Middle slide", "Fixed chassis", [1, 0, 0]),
        ("Receiver outer", "Outer slide", "Fixed chassis", [1, 0, 0]),
        ("Left yoke close", "Left capture yoke", "Outer slide", [0, 1, 0]),
        ("Right yoke close", "Right capture yoke", "Outer slide", [0, 1, 0]),
        ("CORAL feed and exit", "Demonstration CORAL", "Outer slide", [1, 0, 0]),
    ]:
        joint(name, "slider", one, two, axis)
    dr = HERE / "native-directions.json"
    emit(dr, directions)
    for j in joints:
        if j["id"] in directions:
            j["direction"] = dict(
                file=dr.relative_to(ROOT).as_posix(), sha256=digest(dr)
            )
    manifest = dict(
        schema="agent-frc/public-fusion-handoff@1",
        revision=REVISION,
        configuration=obs["configuration"],
        shareable=True,
        archive=dict(file=obs["source"], sha256=digest(archive)),
        reopen=dict(file=ro.relative_to(ROOT).as_posix(), sha256=digest(ro)),
        components=claims,
        joints=joints,
    )
    emit(HERE / "public-handoff.json", manifest)


if __name__ == "__main__":
    p = ArgumentParser(description=__doc__)
    p.add_argument("exchange", type=Path)
    a = p.parse_args()
    build(a.exchange)
