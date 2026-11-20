"""Check limit edges, seated fasteners and staged access on the actual machined parts."""

from OCP.BRepAlgoAPI import BRepAlgoAPI_Common

from attachments import base, bounds, common, cyl, dist
from battery_fit import collisions
from clearance import move
from endpoint_mounts import CAM_BOTTOM, CAM_Y, HERE_OUT, make
from installed_audit import sha
from issue import document
from receiver_motion import assembly


def run():
    _, selected, _, _ = assembly()
    # Reconstruct *before* applying this operation, so bores and half-depth
    # pockets have exactly the same original host as the issued STEP.
    from installed_audit import HERE
    from installed_audit import reopen

    before = {k: s for k, *_, s in reopen()}
    before.update(base.named(HERE / "issue/station-and-jack-interfaces.step"))
    _, tools, stations = make(before)
    boxes = {k: bounds(s) for k, s in selected.items()}
    cam = selected["slide-middle-stop-dog"]
    stations_out = {}
    for label, station in stations.items():
        k = "slide-" + label + "-switch"
        for i in (0, 1):
            screw = k + f"-base-M2-{i}"
            assert (
                dist(selected[screw + "-washer"], selected["support-machined-bridge"])
                < 1e-4
            )
        # Cast a 0.002 mm-wide feeler normal to the cam at the grounded tip
        # datum. A rising flank must pass the measured operating position
        # before it reaches the constant-depth plateau.
        target = station["centerXmm"]
        samples = {}
        for from_end in (0, 1, 4, 8, 10, 12, 14, 18, 24, 32, 38, 40):
            stroke = from_end if label == "retracted" else 370 - from_end
            moved = move(cam, (stroke / 2, 0, 0))
            col = cyl(target, CAM_Y, 450, 0.001, 15)
            fragment = BRepAlgoAPI_Common(moved, col).Shape()
            pp = list(base.subsolids(fragment))
            low = min((bounds(s)[2] for s in pp), default=None)
            samples[str(from_end)] = round(low, 5) if low is not None else None
        tools[k + "-base-M2-0"]
        bases = {}
        for i in (0, 1):
            stud = k + f"-base-M2-{i}"
            plate = selected[k + "-bracket"]
            washer = selected[stud + "-washer"]
            base_seat = common(move(selected[stud], (0, 0, -0.01)), plate) / 0.01
            washer_seat = (
                common(move(washer, (0, 0, 0.01)), selected["support-machined-bridge"])
                / 0.01
            )
            assert min(base_seat, washer_seat) > 1
            bases[stud] = {
                "headAnnulusMm2": base_seat,
                "underBridgeWasherSeatMm2": washer_seat,
            }
        # Both switches can be assembled and calibrated on the removed
        # bracket. Refit the assembly with its switch installed from above.
        shafts = {
            name: collisions(tool, selected, boxes, {name})
            for name, tool in tools.items()
            if name.startswith(k + "-base-")
        }
        assert all(not hits for hits in shafts.values()), shafts
        pair = {n: s for n, s in selected.items() if n.startswith(k)}
        free_bench = {
            name: collisions(tool, pair, boxes, {name})
            for name, tool in tools.items()
            if name.startswith(k + "-driver-")
        }
        assert all(not h for h in free_bench.values()), free_bench
        assert (
            abs(samples["0"] - CAM_BOTTOM) < 0.001
            and abs(samples["8"] - CAM_BOTTOM) < 0.001
        )
        assert samples["40"] is None
        stations_out[label] = dict(
            surfaceByRackDistanceFromEndMm=samples,
            baseSeats=bases,
            bracketDriverAndSocketHits=shafts,
            detachedSwitchDriverHits=free_bench,
        )
    # The middle-stage cam is installed before the inner rail's moving pan.
    bare = {
        k: s
        for k, s in selected.items()
        if k.startswith("slide-middle-dog-")
        or k in ("slide-middle-stop-dog", "slide-R-middle")
    }
    for k in ("slide-middle-dog-M2-0", "slide-middle-dog-M2-1"):
        assert not collisions(tools[k], bare, boxes, {k})
    result = dict(
        drawing=sha(HERE_OUT / "switch-mounts.step"),
        stations=stations_out,
        outerToMiddleTravelRatio=2,
        camRisePerMm=0.3,
        flatHalfLengthMm=4,
        calibration=dict(
            setOvertravelMm=0.35,
            setupErrorMm=0.1,
            guaranteedRemainingOvertravelMm=0.10,
            assertedByRackDistanceFromEndMm=9.666667,
            earliestAssertionRackDistanceFromEndMm=11,
            releasedBeyondRackDistanceFromEndMm=14.333334,
            catalogOPRangeMm=[5.3, 8.3],
            unadjustedLowOPMissesByMm=1.15,
            unadjustedHighOPOverdrivesToMm=1.85,
        ),
        tripScope="One adjusting jig per switch sets free-to-operated travel. At -/+0.10 mm residual error, operating overtravel is 0.25..0.45 mm on either plateau. Middle displacement is half the outer rack movement. A 0.5 mm maximum return differential extends the departing switch's asserted region. Neither end can qualify the other. This is a geometric switch profile; wiring, contact bounce, actual switching force, and the driven stopping distance require separate commissioning.",
    )
    document(HERE_OUT / "switch-controls.json", result)


if __name__ == "__main__":
    run()
