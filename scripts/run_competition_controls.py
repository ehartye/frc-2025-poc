"""Run the project-owned WPILib receiver tests offline and retain a source-bound receipt."""
from registered_withdrawals import matrix
from tempfile import TemporaryDirectory

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "shared/examples/java-2026-sim"
SOURCE = ROOT / "competition-controls"
CASES = {
    "acquireConfirmCarryReleaseAndWithdraw", "invalidStuckSensorRequiresEmptyStoppedExplicitRecovery",
    "unchangedSeatTimestampCannotCompleteDwell", "lostCustodyOrTargetPermissionInhibitsActuation",
    "lostSeatBlocksDriveWithinOneControlTick", "jamAndStalePoseAreLatched",
    "stuckAxisTimesOutEvenWithFreshEncoderFrames", "latchedEntryDuringReleaseCannotStartWithdrawal",
    "cancellationImmediatelyDropsAllDemandsAndBlocksSilentRestart", "exitFromAnotherAttemptCannotAuthorizeWithdrawal",
    "deployedCoordinatesMatchTheProjectMission", "stuckWithdrawalIsDetectedBeforeFullRetractionTimeout",
    "legacyShelfSignalKeepsBothBeltsAndSlideStationary", "completeFrameCannotEnableOrdinaryCommand",
    "wholeFrameRequiresMatchingScopeIdentityGeometryAndExposure", "changingTrackEpochLatchesFault",
    "withdrawalRevocationStopsMotion", "disablingDuringHoldClearsAllDemands",
    "everyRegisteredBumperExposureCanWithdrawOnItsSelectedTrack",
    "uncalibratedStereoCannotMove", "wrongPieceCannotMove", "tooWideAnIntervalCannotAuthorize",
    "frameFromFutureCannotAuthorize", "schedulingDelayCannotStartLateRetreat",
    "driverStationDisableCancelsAndEnableAloneStaysNeutral", "sensorPortGeometryRejectsTheOldSeatLocation",
}


def digest(file):
    return hashlib.sha256(file.read_bytes()).hexdigest()


def pins():
    inputs = list((SOURCE / "src").rglob("*.java"))
    inputs.extend((ROOT / "full-match/red-kit").glob("*"))
    inputs.extend((ROOT / "design/competition-package/poses").glob("*.step"))
    inputs.extend(ROOT / "full-match" / name for name in ("materialize_red.py", "field-trial.json", "red-assembly.json",
                                                           "red-withdrawal-clock.json", "observer-extent.json"))
    inputs.extend([ROOT / "scripts/withdrawal_trial.py", ROOT / "scripts/registered_withdrawals.py", SOURCE / "init.gradle", SOURCE / "io-contract.json", ROOT / "mission/candidate.json",
                   Path(__file__).resolve(), PROJECT / "build.gradle", PROJECT / "gradle.lockfile",
                   PROJECT / "gradle/wrapper/gradle-wrapper.properties"])
    return {p.relative_to(ROOT).as_posix(): digest(p) for p in sorted(inputs)}


def run(jdk, cache, output):
    with TemporaryDirectory() as temp:
        red_dir = Path(temp) / "red-401"
        subprocess.run([sys.executable, "-B", str(ROOT / "full-match/materialize_red.py"),
                        "--output", str(red_dir)], check=True)
        trials, evidence = matrix(ROOT, red_dir)
    if not trials:
        raise ValueError("no supported registered red exposure")
    source_pins = pins()
    io = json.loads((SOURCE/"io-contract.json").read_bytes())
    mission = json.loads((ROOT/"mission/candidate.json").read_bytes())
    poses = mission["poses"]
    paired = [(name, "transport" if name == "carry" else name) for name in ("stowed","receive","carry","place")]
    for name, motion_name in paired:
        pose = poses[motion_name]
        if io["posesMm"][name] != [pose["height"],pose["stroke"]]:
            raise ValueError("controller contract and mission geometry differ: "+name)
    release = mission["motion"]["releaseSpeed_mps"]
    if release != io["controlParameters"]["releaseSurfaceSpeedMps"]:
        raise ValueError("controller contract and mission release surface speed differ")
    if not (jdk / "bin" / ("javac.exe" if os.name == "nt" else "javac")).is_file():
        raise ValueError("select the project's existing JDK 17")
    if not (cache / "caches/modules-2").is_dir():
        raise ValueError("select the existing populated Gradle cache")
    wrapper_config = (PROJECT/"gradle/wrapper/gradle-wrapper.properties").read_text().splitlines()
    properties = dict(line.split("=",1) for line in wrapper_config if "=" in line)
    distributions = cache / properties["distributionPath"]
    if not list(distributions.glob("gradle-8.11-*/*/gradle-8.11/bin/gradle*")):
        raise ValueError("the required Gradle 8.11 wrapper distribution must already be cached")
    run_id = str(uuid.uuid4())
    output.mkdir(parents=True, exist_ok=False)
    build = output / "build"
    env = dict(os.environ, JAVA_HOME=str(jdk), GRADLE_USER_HOME=str(cache),
               REEF_CONTROLS_SOURCE=str(SOURCE), REEF_CONTROLS_BUILD=str(build), REEF_CONTROLS_RUN=run_id)
    env["REEF_CONTROLS_POSES"] = ",".join(str(value) for name,_ in paired for value in io["posesMm"][name])
    reviewed_rows = output / "reviewed-exposures.csv"
    reviewed_rows.write_text("".join(",".join(map(str,row))+"\n" for row in trials), encoding="ascii", newline="\n")
    env["REEF_WITHDRAWAL_ROWS"] = str(reviewed_rows)
    env["REEF_CONTROLS_RELEASE"] = str(release)
    env["PATH"] = str(jdk/"bin")+os.pathsep+env.get("PATH","")
    wrapper = PROJECT / ("gradlew.bat" if os.name == "nt" else "gradlew")
    args = [str(wrapper), "--offline", "--no-daemon", "--rerun-tasks", "--project-cache-dir",
            str(output/"gradle-project-cache"), "-I", str(SOURCE/"init.gradle"), "test",
            "--tests", "org.reefscape.controls.StationCycleTest"]
    with (output/"gradle.log").open("w") as log:
        subprocess.run(args, cwd=PROJECT, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    xml = build/"test-results/test/TEST-org.reefscape.controls.StationCycleTest.xml"
    result = ET.parse(xml).getroot()
    names = {c.get("name","").removesuffix("()") for c in result.findall("testcase")}
    tokens = (result.findtext("system-out") or "").splitlines()
    if (names != CASES or int(result.get("tests","0")) != len(CASES)
        or any(result.get(k) != "0" for k in ("failures","errors","skipped"))
        or tokens.count("CONTROLS_RUN_ID="+run_id) != 1 or pins() != source_pins):
        raise ValueError("incomplete test inventory, run mismatch or changed source")
    receipt = {"schema":"reefscape-offline-controller-run/1","runId":run_id,"tests":sorted(names),
               "sourceSha256":source_pins,"junitXmlSha256":digest(xml),
               "driverStationMode":"WPILib desktop HAL simulation", "inputSource":"scripted FakeIO",
               "wholeRobotDesktopTrial":evidence, "ordinaryWithdrawalEnabled":False,
               "qualifiedExposureCount":len(trials), "qualifiedRowsSha256":digest(reviewed_rows),
               "electricalHardwareObserved":False,"physicalCalibration":None,"officialScore":None}
    (output/"receipt.json").write_text(json.dumps(receipt,indent=2)+"\n",encoding="utf-8")
    print(f"{len(names)} desktop WPILib checks passed. Receipt: {output/'receipt.json'}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--java-home",type=Path,required=True)
    parser.add_argument("--gradle-home",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True,help="new evidence directory")
    args = parser.parse_args()
    run(args.java_home.resolve(),args.gradle_home.resolve(),args.output.resolve())
