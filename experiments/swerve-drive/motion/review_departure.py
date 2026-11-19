"""Collect the independent departure stages and check the causal stop on each signed clock."""

import argparse
from pathlib import Path
from loaded_departure import CASES, STEPS, HERE, read, sha, write


def run(raw, feeds, witness):
    records = []
    fset = read(feeds)
    optics = read(witness)
    if optics["firstFeedSha256"] != sha(feeds) or fset["openingSha256"] != sha(
        HERE / "energy.json"
    ):
        raise ValueError("Feed, optics and wheel receipts disagree")
    for side in CASES:
        for step in STEPS:
            folder = raw / f"{side}-{step:g}us"
            feed = next(
                f
                for f in fset["attempts"]
                if (f["case"], f["plantStepUs"]) == (side, step)
            )
            matches = [
                r for r in optics["cases"] if (r["case"], r["stepUs"]) == (side, step)
            ]
            row = {
                "case": side,
                "stepUs": step,
                "pipe": feed["physicalPieceId"],
                "sourceDrive": feed["driveTraceSha256"],
                "state": feed["state"],
                "dockWallS": feed["dockGateWallS"],
                "signedPoseMmMmDeg": feed["trueChassisAtFeedMmMmDeg"],
                "guardedPermit": False,
                "qualifiedDeliveries": 0,
            }
            records.append(row)
            if not matches:
                continue
            initial = next(r for r in matches if r["finish"] == "guarded")
            row.update(
                guardedPermit=initial["gate"]["accepted"],
                latePermit=initial["late"]["accepted"],
                matchedFinishPermit=next(
                    r["gate"]["accepted"] for r in matches if r["finish"] == "matched"
                ),
            )
            if not row["guardedPermit"]:
                continue
            service = read(folder / "service.json")
            if (
                service["outgoing"] != sha(folder / f"{side}-{step:g}us-service.npz")
                or service["incoming"] != feed["pipeStateSha256"]
            ):
                raise ValueError("Changed first wheel-service state")
            images = read(folder / "swept-images-600.json")
            mask_register = sha(raw / "wheel-visibility.json")
            if (
                images.get("wheelMaskRegisterSha256") != mask_register
                or optics.get("wheelMaskRegisterSha256") != mask_register
            ):
                raise ValueError("Moving and stationary camera masks differ")
            no_move = read(folder / "drive-0.json")
            if (
                no_move["distanceMm"] != 0.0
                or no_move["serviceState"] != service["outgoing"]
            ):
                raise ValueError("Denied-custody branch moved")
            row["lateOrMatchedChassis"] = no_move
            row["illustrativeWheelsControl"] = {
                k: v
                for k, v in read(folder / "images-600.json").items()
                if k in ("stopWallS", "acceptedFrames", "chassis", "contact")
            }

            ms = lambda t: round((t - service["wallS"]) * 1000)
            options = {
                "forwardTrial": 600,
                "observed": ms(images["stopWallS"]),
                "firstMovingFixedEyeDrop": ms(images["dropout"]["stopWallS"]),
            }
            if not 0 <= min(options.values()) <= max(options.values()) <= 600:
                raise ValueError("Invalid lease")
            row.update(
                service=service,
                options=options,
                routes={},
                forwardIllustrationFeedSha256=sha(folder / "feed.json"),
                forwardIllustrationStationWitnessSha256=sha(folder / "station.json"),
            )
            for duration in dict.fromkeys(options.values()):
                wheels = read(folder / f"drive-{duration}.json")
                cargo = read(folder / f"contact-{duration}.json")
                if (
                    wheels["trajectory"] != sha(folder / f"{side}-{duration}.npz")
                    or wheels["wheelStateAndEncoderTrace"]
                    != sha(folder / f"{side}-{duration}-wheels.npz")
                    or wheels["trajectory"] != cargo["chassis"]
                    or wheels["serviceState"] != service["outgoing"]
                    or cargo["serviceState"] != service["outgoing"]
                    or cargo["sha256"] != sha(folder / cargo["file"])
                    or cargo["terminalStateSha256"]
                    != sha((folder / cargo["file"]).with_suffix(".npz"))
                ):
                    raise ValueError("Changed chassis, pipe clock or terminal state")
                compact = {"wheel": wheels, "pipe": cargo}
                if duration == options["observed"] or duration == 600:
                    obs = read(folder / f"swept-images-{duration}.json")
                    if (
                        obs["chassis"] != wheels["trajectory"]
                        or obs["contact"] != cargo["sha256"]
                    ):
                        raise ValueError("Moving eyes refer to a different plant")
                    compact["vision"] = {k: v for k, v in obs.items() if k != "frames"}
                    compact["vision"]["frames"] = [
                        {
                            k: f[k]
                            for k in (
                                "captureWallS",
                                "fixed",
                                "roof",
                                "accepted",
                                "fourFramesSha256",
                            )
                        }
                        for f in obs["frames"]
                    ]
                row["routes"][str(duration)] = compact
    return dict(
        schema="reefscape-loaded-swerve-departure/1",
        registeredFeedSha256=sha(feeds),
        stationWitnessSha256=sha(witness),
        scope="Prebuild first loaded departure at the red station. Guarded shelf finish, registered two-head images, stowed slide and first source-fed pipe. No placement or second trip.",
        cases=records,
        wheelVisibilityRegisterSha256=sha(raw / "wheel-visibility.json"),
        wholeScanlineControlSha256=sha(raw / "whole-row-control.json"),
        wholeScanlineControls=[
            dict(
                case=r["case"],
                stepUs=r["stepUs"],
                granted=r["gate"]["accepted"],
                fixed=[f["fixed"] for f in r["frames"]],
                roof=[f["roof"] for f in r["frames"]],
            )
            for r in read(raw / "whole-row-control.json")["cases"]
            if r["finish"] == "guarded"
        ],
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("raw", "feeds", "witness", "output"):
        p.add_argument(name, type=Path)
    a = p.parse_args()
    write(a.output, run(a.raw, a.feeds, a.witness))
