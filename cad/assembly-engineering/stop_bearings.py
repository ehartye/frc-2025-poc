"""Give each hard stop a locating tongue and an underside fastening screw."""

from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from attachments import base, bounds, common, cut, cyl, fuse, hexagon, write_step
from battery_fit import collisions
from clearance import move
from installed_audit import sha
from issue import document
from receiver_motion import assembly, OUT


def run():
    _, selected, _, _ = assembly()
    bridge = selected["support-machined-bridge"]
    changed = {}
    tools = {}
    seats = {}
    for name in ("slide-retracted-stop", "slide-extended-stop"):
        stop = selected[name]
        b = bounds(stop)
        cx = (b[0] + b[3]) / 2
        y = -138.8
        # The integral lower tongue takes lateral load on the walls of a
        # matched pocket. The projecting dog contacts this removable block.
        # A single M3 clamp screw goes through the bridge from below.
        key = base.block([cx - 1.9, -145.5, 436.8, cx + 1.9, -141.5, 438.1])
        pocket = base.block([cx - 2.0, -145.6, 436.7, cx + 2.0, -141.4, 438.1])
        stop = fuse(stop, key)
        bridge = cut(bridge, pocket)
        stop = cut(stop, cyl(cx, y, 437.9, 1.25, 8.5))
        gauge = BRepAlgoAPI_Common(cyl(cx, y, 432, 0.15, 6.1), bridge).Shape()
        z = round(bounds(gauge)[2], 3)
        if z > 433.1:
            bridge = cut(bridge, cyl(cx, y, 432, 3.2, z - 432))
        bridge = cut(bridge, cyl(cx, y, z - 0.1, 1.65, 438.2 - z))
        # Choose standard under-head length with >=4 mm usable engagement.
        length = 10 if z < 434 else 8
        screw = fuse(cyl(cx, y, z, 1.5, length), cyl(cx, y, z - 3, 2.75, 3))
        screw = cut(screw, hexagon(cx, y, z - 3.1, 2.5, 1.6))
        n = name + "-M3"
        changed[n] = screw
        changed[name] = stop
        tools[n] = cyl(cx, y, z - 31, 1.6, 28)
        seats[name] = dict(
            x=cx,
            screwY=y,
            screwLengthMm=length,
            minThreadEngagementMm=z + length - 438,
            undersideZmm=z,
            headToHostPositiveAreaMm2=common(move(screw, (0, 0, 0.01)), bridge) / 0.01,
            threadMajorTappedMinorMm3=common(screw, stop),
            tongueXsideClearanceMm=0.1,
            tongueEndClearanceMm=0.1,
            pocketBottomClearanceMm=0.1,
            compressionAtPoint15MmXshiftMm3=common(move(stop, (0.15, 0, 0)), bridge),
        )
    changed["support-machined-bridge"] = bridge
    installed = selected | changed
    boxes = {k: bounds(s) for k, s in installed.items()}
    hits = {}
    for k, s in changed.items():
        if k == "support-machined-bridge":
            continue
        for other, v in collisions(s, installed, boxes, {k}).items():
            hits[" / ".join(sorted((k, other)))] = v
    gauges = {k: collisions(t, installed, boxes, set()) for k, t in tools.items()}
    step = OUT / "stop-bearings.step"
    write_step(changed, step)
    document(
        OUT / "stop-bearings.json",
        dict(
            source=sha(OUT / "switch-mounts.step"),
            drawing=sha(step),
            seats=seats,
            allIntersectionsMm3=hits,
            undersideDrivers=gauges,
            scope="Tongue side bearing takes reaction after the locating clearance closes. Bolt shear, contact impact at achieved speed, thread stripping, bridge flexibility and fatigue require a separate load trial.",
        ),
    )
    print("Stopped", hits, gauges, flush=True)


if __name__ == "__main__":
    run()
