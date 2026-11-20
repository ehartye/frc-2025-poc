"""Check the purchased slotted M2 driver and the removable end stops in assembly order."""

from attachments import bounds, cyl
from battery_fit import collisions
from installed_audit import sha
from issue import document
from receiver_motion import OUT, assembly
from service_variant import roof_changes


def run():
    _, parts, _, _ = assembly()
    from attachments import base

    for name in ("stop-bearings.step", "battery-disconnect.step"):
        parts.update(base.named(OUT / name))
    parts.update(roof_changes()[0])
    for retired in ("roof-guide-tower", "roof-guide-rib-2"):
        parts.pop(retired)
    boxes = {k: bounds(s) for k, s in parts.items()}
    result = {}
    for name, s in parts.items():
        if "-base-M2-" not in name and not name.startswith("slide-middle-dog-M2-"):
            continue
        if name.endswith(("-washer", "-locknut")):
            continue
        b = boxes[name]
        x, y = (b[0] + b[3]) / 2, (b[1] + b[4]) / 2
        # Circumscribe the complete 0.5 x 3 mm reversed-blade gauge and
        # allow a 3.10 mm received shaft through 24 mm of unobstructed reach.
        shaft = cyl(x, y, b[5] - .1, 1.55, 24.1)
        stowed = collisions(shaft, parts, boxes, {name})
        if name.startswith("slide-middle-dog"):
            stage = {
                j: t
                for j, t in parts.items()
                if j.startswith("slide-middle-dog") or j == "slide-R-middle"
            }
        else:
            stage = {
                j: t
                for j, t in parts.items()
                if not (
                    j.startswith("slide-retracted-switch-shoulder")
                    or j.startswith("slide-retracted-switch-locknut")
                    or j
                    in (
                        "slide-retracted-switch",
                        "slide-retracted-switch-lever",
                        "slide-extended-stop",
                        "slide-retracted-stop",
                    )
                )
            }
        staged = collisions(shaft, stage, boxes, {name})
        result[name] = dict(shaftTopZmm=b[5], allInstalled=stowed, staged=staged)
    document(
        OUT / "slotted-drivers.json",
        dict(
            switchSource=sha(OUT / "switch-mounts.step"),
            diameterMm=3.10,
            bladeMm=[0.5, 3.0],
            reachMm=24,
            rows=result,
            source="https://www.westfieldfasteners.co.uk/Datasheets/ScrewBolt_SlotChs_M.pdf",
        ),
    )
    print(result, flush=True)


if __name__ == "__main__":
    run()
