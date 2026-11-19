"""Stop the mission at the first missing physical, observation or field prerequisite."""

import argparse
import json
from pathlib import Path
import numpy as np
from plant import HERE, ROOT, read, Selection, digest
from shared_clock import run_case
from field_screen import DRIVE, brake, nav


def event(t, description):
    return [round(float(t), 6), description]


def match(assets, side, step, mode):
    station_fault = mode in ("late", "interrupt", "noContrast")
    opening_name = "revised-opening" + ("-" + mode if station_fault else "")
    opening = read(HERE / (opening_name + ".json"))[side]
    path = assets / "straight-guide-match" / (side + "-" + opening_name + ".npz")
    if digest(path) != opening["pathSha256"]:
        raise ValueError("Opening hash")
    with np.load(path) as start:
        ts = start["t"].copy()
        poses = start["pose"].copy()
    evidence = read(HERE / "issued-opening.json")[side + "-" + opening_name]
    if evidence["sourceSha256"] != opening["pathSha256"]:
        raise ValueError("Wrong signed opening")
    tape = read(ROOT / "station-landmark/native.json")
    setup_error_mm = 1000 * np.linalg.norm(
        poses[0, :2] - 0.0254 * np.array(tape["startXYin"])
    )
    yaw_error = abs(np.degrees(poses[0, 2]) - tape["startHeadingDeg"])
    if (
        setup_error_mm > 5 + 1e-6
        or yaw_error > 0.2 + 1e-6
        or min(tape["startTapeCommonMm3"].values()) <= 0
    ):
        raise ValueError("Starting pose outside eroded native tape overlap")
    leave = (
        evidence["leaveTapeMm"] > 0
        and evidence["disabledTranslationMm"] == 0
        and evidence["disabledYawDeg"] == 0
    )
    if (
        min(
            evidence[k]
            for k in ("minBarrierMm", "minPeerMm", "minStockMm", "stationHoldPeerMm")
        )
        <= 0
    ):
        raise ValueError("Opening or static hold obstructed")
    if not leave:
        raise ValueError("Opening does not support the modeled leave")
    events = [
        event(0, "Start on the tape with an empty receiver"),
        event(15, "AUTO ends; the robot has cleared the tape"),
        event(18, "TELEOP begins"),
    ]
    reached = opening["status"] == "observed stationary"
    events.append(
        event(
            opening["stationWallS"],
            "Station pose observed"
            if reached
            else "Brake and hold: " + opening["status"],
        )
    )
    outcome = "Station approach inhibited"
    admission = False
    cargo = None
    field = None
    if reached:
        feed = read(HERE / f"feed-{side}-{step:g}.json")
        station = read(HERE / f"station-{side}-{step:g}.json")
        if feed["stateSha256"] != station["stateSha256"]:
            raise ValueError("Station handoff mismatch")
        events.append(event(21.10, "Human releases coral-A"))
        if feed["firstBandMs"] is not None:
            events.append(
                event(
                    21.571672635314284 + feed["firstBandMs"] / 1000,
                    "Both port signals indicate a seat",
                )
            )
        admission = station["guardedStationHandoff"]
        if admission:
            timely = [
                f["serviceMs"]
                for f in station["frames"]
                if f["serviceMs"] <= 5460 and f["fixed"] and f["roof"]
            ]
            if len(timely) < 5:
                raise ValueError("Missing timely independent observations")
            events.append(
                event(
                    21.571672635314284 + max(timely) / 1000,
                    "Five timely fixed and roof captures corroborate coral-A",
                )
            )
            cargo = read(
                HERE
                / (
                    f"jam-{side}.json"
                    if mode == "idler-jam"
                    else f"cargo-{side}-{step:g}.json"
                )
            )
            if mode == "idler-jam" and (step != 31.25 or not cargo["surfaceStuck"]):
                raise ValueError("Wrong jam trial")
            if cargo["sourceFeedSha256"] != feed["trajectorySha256"]:
                raise ValueError("Cargo identity mismatch")
            wheel, body, _ = Selection(step, True).path(assets, side, "nominal")
            events.append(event(DRIVE, "Wheels begin the loaded trip"))
            inhibit = None if cargo["plannedEndpoint"] else cargo["finalWallS"]
            if inhibit is not None:
                events.append(
                    event(
                        inhibit,
                        "Inhibit on "
                        + str(
                            cargo["firstFault"]
                            or cargo["physicalStop"]
                            or cargo["lostPipe"]
                        ),
                    )
                )
                wheel, body, _ = brake(
                    wheel,
                    body,
                    inhibit - DRIVE,
                    read(ROOT / "station-landmark/routes.json")["cases"][side],
                )
                outcome = "Carry inhibited; stationary hold"
            else:
                outcome = "Stopped at sector A; extension held"
            field = run_case(assets, side, step, stop=inhibit)
            if not cargo["plannedEndpoint"] and field["braking"] is None:
                raise ValueError("Missing finite brake")
            if (
                min(
                    field["clearance"][k]
                    for k in ("minimumBarrierMm", "minimumStockMm", "minimumPeerMm")
                )
                <= 0
                or min(field["stationaryPeerMm"].values()) <= 0
            ):
                raise ValueError("Occupied route or hold")
            if field["power"]["screened"]["firstReferenceInhibitWallS"] is not None:
                raise ValueError("Unsupported demand")
            if (
                cargo["peakPlateDeflectionBoundMm"] > 0.05
                or cargo["firstStageAboveGuardS"] is not None
            ):
                raise ValueError("Roof background envelope exceeded")
            if np.max(abs(body[0] - poses[-1])) > 1e-6:
                raise ValueError("Wheel discontinuity")
            ts = np.r_[ts, DRIVE + wheel]
            poses = np.vstack([poses, body])
            events.append(event(field["finalWallS"], outcome))
        else:
            outcome = "Hold at the station; custody unresolved"
            events.append(event(27.03167263531428, outcome))
    events.extend(
        [
            event(153, "Match ends; coral-B remained in station stock"),
            event(156, "Assessment: LEAVE only"),
        ]
    )
    # A separated 850 mm enclosing square excludes PARK even for relieved corners.
    from field import polygon_separation

    w = nav.corrected().PROFILE["field"]["length"]
    zone = [
        ((w / 2 - 23) * 0.0254, 0),
        ((w / 2 + 23) * 0.0254, 0),
        ((w / 2 + 23) * 0.0254, 146.5 * 0.0254),
        ((w / 2 - 23) * 0.0254, 146.5 * 0.0254),
    ]
    park_gap = 1000 * float(polygon_separation(*poses[-1:, :].T, zone)[0])
    if park_gap <= 0:
        raise ValueError(
            "Final robot touches park zone; native material check required"
        )
    # Emit a one-second field overview with exact event poses retained for scrubbing.
    plot_t = np.unique(np.r_[np.arange(157.0), [v[0] for v in events]])
    angles = np.unwrap(poses[:, 2])
    sampled = np.column_stack(
        [
            plot_t,
            *[
                np.interp(plot_t, ts, coords)
                for coords in (poses[:, 0], poses[:, 1], angles)
            ],
        ]
    )
    return dict(
        side=side,
        stepUs=step,
        condition=mode,
        outcome=outcome,
        stationHandoff=admission,
        events=sorted(events),
        field=field,
        cargoTrace=cargo["trajectorySha256"] if cargo else None,
        lastPipeObservationWallS=21.571672635314284 + cargo["lastObservedJavaMs"] / 1000
        if cargo
        else None,
        postHoldPipeState="No later registered pipe observation"
        if reached
        else "No human release",
        finalScoringBasis="Verified starting overlap and whole-bumper LEAVE. No attained L1 support or end-zone body overlap.",
        cargoTiming=list(cargo["lastPipe"][:13]) if cargo else None,
        startXYErrorMm=setup_error_mm,
        startYawErrorDeg=yaw_error,
        erodedTapeBumperMm3=tape["startTapeCommonMm3"],
        bargeZoneSeparationMm=park_gap,
        scoredPieceIds=[],
        remainingStock=["coral-B"] if reached else ["coral-A", "coral-B"],
        points={"leave": 3, "l1": 0, "park": 0},
        pose=sampled.round(6).tolist(),
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("assets", type=Path)
    p.add_argument("side")
    p.add_argument("step", type=float)
    p.add_argument(
        "--mode",
        default="nominal",
        choices=["nominal", "late", "interrupt", "noContrast", "idler-jam"],
    )
    a = p.parse_args()
    r = match(a.assets, a.side, a.step, a.mode)
    (HERE / f"match-{a.side}-{a.step:g}-{a.mode}.json").write_text(
        json.dumps(r, indent=2) + "\n", encoding="utf8", newline="\n"
    )
