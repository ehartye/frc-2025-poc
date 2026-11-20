"""Place the REV PDH on the published four-hole mounting pattern using actual tab depth."""

from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from attachments import base, bounds, cone, cut, cyl, dist, fuse, hexagon, tube, volume
from installed_audit import HERE, sha

SOURCE = "https://www.revrobotics.com/content/cad/REV-11-1850.STEP"
SKU_SHA = "bb14d12b33e8b21bb49f85166f9fb55e358d3044f85580321a291e3a7b9a1fa9"


def apply(proposed, robot):
    key = "distribution-bridge"
    bridge = robot[key]
    deck = proposed["host-drilled-swerve-deck"]
    back = robot["edge-pocket-electrical-backplate"]
    case = robot["rev-pdh"]
    items = {}
    for i, x in enumerate((-320.2375, -104.3375)):
        for j, y in enumerate((104.7625, 206.3625)):
            k = f"pdh-{i}-{j}"
            # The 6.35 mm mounting tabs were measured in the supplier STEP.
            case = cut(case, cyl(x, y, 160, 2.50, 9))
            case = cut(case, cyl(x, y, 167.35, 7.0, 35))
            bridge = cut(bridge, cyl(x, y, 157, 2.25, 5))
            items[k + "-M4x16-low-screw"] = cut(
                fuse(cyl(x, y, 155.6, 3.5, 2.4), cyl(x, y, 158, 2.0, 16)),
                hexagon(x, y, 155.5, 2.5, 1.55),
            )
            items[k + "-M4-top-washer"] = tube(x, y, 167.35, 9, 4.3, 0.8)
            items[k + "-M4-locknut"] = cut(
                hexagon(x, y, 168.15, 7.0, 4.9), cyl(x, y, 168, 2.0, 6)
            )
    through = []
    for i, x in enumerate((-275.0, -129.0)):
        for j, y in enumerate((116.0, 199.0)):
            k = f"pdh-base-{i}-{j}"
            gauge = cyl(x, y, 128, 0.20, 38)
            match = BRepAlgoAPI_Common(deck, gauge).Shape()
            if volume(match) < 0.01:
                raise ValueError(("Missing host", x, y))
            lower = round(bounds(match)[2], 5)
            for s in (back, bridge):
                if volume(BRepAlgoAPI_Common(s, gauge).Shape()) < 0.10:
                    raise ValueError(("Missing mount land", k))
            hole = cyl(x, y, lower - 1, 2.25, 162 - lower)
            bridge = cut(cut(bridge, hole), cone(x, y, 159.1, 2.25, 4.15, 1.9))
            back = cut(back, hole)
            deck = cut(deck, hole)
            # Full size threaded-stock class 10.9 screw, receive length at
            # least stack + prevailing nut + two 0.7 mm pitches; trim to fit.
            items[k + "-M4-flush-screw"] = fuse(
                cyl(x, y, lower - 7.2, 2.0, 159.1 - lower + 7.2),
                cone(x, y, 159.1, 2.0, 3.9, 1.9),
            )
            items[k + "-M4-flush-screw"] = cut(
                items[k + "-M4-flush-screw"], hexagon(x, y, 159.4, 2.5, 2)
            )
            items[k + "-M4-bottom-washer"] = tube(x, y, lower - 0.8, 9, 4.3, 0.8)
            items[k + "-M4-locknut"] = cut(
                hexagon(x, y, lower - 5.7, 7.0, 4.9), cyl(x, y, lower - 6, 2.0, 6)
            )
            through.append(
                dict(
                    x=x,
                    y=y,
                    deckLowerMm=lower,
                    cutLengthMm=round(161.0 - lower + 7.2, 4),
                )
            )
    proposed.update(
        {
            "rev-pdh": case,
            key: bridge,
            "edge-pocket-electrical-backplate": back,
            "host-drilled-swerve-deck": deck,
        }
    )
    return items, through
