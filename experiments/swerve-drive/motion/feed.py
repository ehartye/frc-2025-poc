"""Replay the existing stationary receiver at a source-verified, actually reached swerve dock."""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "guide-impact"))
spec = importlib.util.spec_from_file_location(
    "swerve_station_feed", ROOT / "guide-impact/replay.py"
)
engine = importlib.util.module_from_spec(spec)
spec.loader.exec_module(engine)
sys.path.insert(0, str(ROOT / "split-guide-acquisition"))
from split_controller import candidate


def digest(path):
    with path.open("rb") as source:
        return hashlib.file_digest(source, "sha256").hexdigest()


def run(jdk, clockdir, cache, case, step, collect):
    record = json.loads((HERE / "opening.json").read_bytes())["cases"][case]
    if record["stationLockWallS"] is None or record["stationLockWallS"] >= 21.10:
        raise ValueError("No observed dock by this feed slot")
    if digest(clockdir / (case + ".csv")) != record["csvSha256"]:
        raise ValueError("Different attained four-module clock")
    x, y, yaw = record["actualTerminalMmMmDeg"]
    # Positive simulator X/Y move the delivered pipe relative to a stationary receiver.
    # Positive measured station-frame X/Y move the chassis relative to that fixed delivery.
    piece_pose = [-x, -y, yaw]
    side = "leftSmall" if case == "plus" else "rightSmall"
    dst = cache / case
    dst.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        cls = Path(tmp)
        java = cls / "PortAcquisition.java"
        java.write_text(candidate(engine.feed), encoding="utf8", newline="\n")
        program = json.loads(
            (ROOT / "split-guide-acquisition/signals.json").read_bytes()
        )["programSha256"]
        if digest(java) != program:
            raise ValueError("Receipt and two-port Java controller differ")
        if not collect:
            subprocess.run(
                [
                    str(jdk / "bin/javac"),
                    "-d",
                    str(cls),
                    str(java),
                    str(ROOT / "observable-acquisition/PortConsole.java"),
                ],
                check=True,
            )
        trial = engine.trial(
            (
                side,
                piece_pose,
                "folded",
                step,
                ROOT / "guide-mounts",
                jdk,
                cls,
                dst,
                5460,
                1800.0,
                0.236,
                ROOT / "compliant-carriage",
                0.0,
                collect,
                True,
                program,
            )
        )
    human_feed = 21.10
    tube = engine.feed.study.route.DATA["feed"]
    lip = (
        human_feed
        + tube["free_drop_s"]
        + engine.feed.ramp(tube, engine.feed.cfg.ramp_mu)["time"]
    )
    java_epoch = lip - 0.100
    return dict(
        case=case,
        firstFeedWallS=human_feed,
        lipWallS=lip,
        javaEpochWallS=java_epoch,
        firstBandWallS=None
        if trial["firstBandMs"] is None
        else java_epoch + trial["firstBandMs"] / 1000,
        endWallS=java_epoch + trial["lastJavaMs"] / 1000,
        physicalPieceId="coral-A",
        driveTraceSha256=record["csvSha256"],
        dockGateWallS=record["stationLockWallS"],
        trueChassisAtFeedMmMmDeg=[x, y, yaw],
        javaFirstBandMs=trial["firstBandMs"],
        javaLastMs=trial["lastJavaMs"],
        state=trial["lastState"],
        programSha256=program,
        plantStepUs=step,
        pipeStateSha256=trial["stateSha256"],
        guideHistorySha256=trial["guideHistorySha256"],
        pipeTrajectorySha256=trial["trajectorySha256"],
        receiverBoundaries=trial["carriageSupport"],
        guide=trial["binding"]["source"],
        independentCustody=False,
        assumptions="Same release model, fixed station chassis and retained guide/carriage attachment boundary. No independent pipe image or later carry is assessed.",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("jdk", type=Path)
    p.add_argument("clockdir", type=Path)
    p.add_argument("raw", type=Path)
    p.add_argument("case", choices=("plus", "minus"))
    p.add_argument("step", type=float, choices=(15.625, 31.25))
    p.add_argument("--collect", action="store_true")
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args()
    result = run(a.jdk, a.clockdir, a.raw, a.case, a.step, a.collect)
    a.out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf8", newline="\n")
