"""Place the two Omron hinge-lever limit switches on the same cam and middle slide."""

import json
import math
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism, BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Vec
from attachments import base, bounds, cut, cyl, fuse, hexagon, tube, write_step
from battery_fit import collisions
from clearance import move
from installed_audit import HERE, reopen, sha
from issue import document
from receiver_mechanics import rotate

HERE_OUT = HERE / "receiver-service"
CAM_CENTER = -97.5
CAM_Y = -150.5
CAM_BOTTOM = 456.15
ACTUATION_Z = CAM_BOTTOM + 0.35
SWITCH_AXIS_Z = ACTUATION_Z - 6.8


def slab(xz, y0, y1):
    poly = BRepBuilderAPI_MakePolygon()
    for x, z in xz:
        poly.Add(gp_Pnt(x, y0, z))
    poly.Close()
    return BRepPrimAPI_MakePrism(
        BRepBuilderAPI_MakeFace(poly.Wire()).Face(), gp_Vec(0, y1 - y0, 0)
    ).Shape()


def bore_y(x, y, z, r, length):
    return BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(x, y, z), gp_Dir(0, 1, 0)), r, length
    ).Shape()


def profile(center):
    # Moving cam. Lower surface is flat for +/-4 mm; each lead rises 4.5
    # mm in 15 mm. The dog is cut from a single 6061-T6 blank with the leg.
    coords = [
        (center - 19, CAM_BOTTOM + 4.5),
        (center - 4, CAM_BOTTOM),
        (center + 4, CAM_BOTTOM),
        (center + 19, CAM_BOTTOM + 4.5),
        (center + 19, 463),
        (center - 19, 463),
    ]
    return slab(coords, CAM_Y - 3.4, CAM_Y + 3.4)


def make(parts):
    made = {}
    tools = {}
    carrier = "slide-middle-stop-dog"
    contact = profile(CAM_CENTER)
    web = base.block([-100, CAM_Y + 3.25, 460.9, -95, -144, 463])
    dog = fuse(fuse(parts[carrier], parts[carrier + "-leg"]), fuse(contact, web))
    # Two vertical taps stay in the 6.65 mm top ligament; the inner rail
    # slides on its outboard side. The dog, tongue and transfer foot are
    # manufactured as one piece and bolt to the middle rail.
    middle = parts["slide-R-middle"]
    dog = fuse(dog, base.block([-107.2, -169, 476, -87.8, -165.7, 478.5]))
    for i, x in enumerate((-104, -91)):
        yy = -167.50
        middle = cut(middle, cyl(x, yy, 471.2, 0.8, 5.1))
        dog = cut(dog, cyl(x, yy, 475.9, 1.10, 2.7))
        key = f"slide-middle-dog-M2-{i}"
        made[key] = fuse(cyl(x, yy, 472.5, 1, 6), cyl(x, yy, 478.5, 1.9, 1.4))
        tools[key] = cyl(x, yy, 479.9, 1.2, 20)
    made[carrier] = dog
    made["slide-R-middle"] = middle
    bridge = parts["support-machined-bridge"]
    stations = {}
    for label, center, sign in (
        ("retracted", CAM_CENTER, 1),
        ("extended", CAM_CENTER + 185, -1),
    ):
        name = "slide-" + label + "-switch"

        # Hinge axes and target along X. Housing extends away from interior
        # travel. Y is normal to the face-mounted 5.8 mm wide switch.
        def X(u):
            return center + sign * (u - 14.0)

        def block(u0, y0, z0, u1, y1, z1):
            xx = sorted((X(u0), X(u1)))
            return base.block([xx[0], y0, z0, xx[1], y1, z1])

        a = SWITCH_AXIS_Z
        housing = block(0, CAM_Y - 2.9, a - 1.5, 12.8, CAM_Y + 2.9, a + 5)
        # Omron frame uses one fully round bore and one relieved hole.
        for u in (3.15, 9.65):
            housing = cut(housing, bore_y(X(u), CAM_Y - 3.1, a, 1.0, 6.2))
        made[name] = housing
        bracket = block(-3, CAM_Y - 5.5, 438, 16, CAM_Y - 2.9, a + 3.0)
        foot = block(-7, CAM_Y - 10.7, 438, 20, CAM_Y - 2.2, 440.2)
        bracket = fuse(bracket, foot)
        # Raise/lower the two shoulder dowels together in these closed slots.
        for i, u in enumerate((3.15, 9.65)):
            x = X(u)
            slot = fuse(
                bore_y(x, CAM_Y - 5.6, a - 1.65, 1.05, 2.8),
                fuse(
                    bore_y(x, CAM_Y - 5.6, a + 1.65, 1.05, 2.8),
                    base.block(
                        [
                            x - 1.05,
                            CAM_Y - 5.6,
                            a - 1.65,
                            x + 1.05,
                            CAM_Y - 2.8,
                            a + 1.65,
                        ]
                    ),
                ),
            )
            bracket = cut(bracket, slot)
            # M2 shoulder screw has a smooth 1.90 mm diameter through case
            # and adjustment slot. Its #M2 locking nut clamps against backing.
            axis_y = CAM_Y - 5.5
            pin = fuse(
                bore_y(x, axis_y - 3.4, a, 1.0, 3.4), bore_y(x, axis_y, a, 0.95, 8.4)
            )
            pin = fuse(pin, bore_y(x, CAM_Y + 2.9, a, 1.75, 1.4))
            made[f"{name}-shoulder-screw-{i}"] = pin
            nut = move(
                rotate(hexagon(0, 0, 0, 4.0, 2.4), (1, 0, 0), -math.pi / 2),
                (x, axis_y - 2.4, a),
            )
            made[f"{name}-locknut-{i}"] = cut(
                nut, bore_y(x, axis_y - 2.5, a, 1.0, 2.65)
            )
            tools[f"{name}-driver-{i}"] = bore_y(x, CAM_Y + 4.3, a, 1.2, 15)
        # Base screws pass into the five-mm 6061 bridge. Countersinks are
        # short enough to leave a full annular seat above the pocket underside.
        for i, u in enumerate((-4.5, 6.4) if label == "retracted" else (-4.5, 17.5)):
            x, y = X(u), CAM_Y - (8.65 if label == "retracted" and i == 1 else 8.0)
            bracket = cut(bracket, cyl(x, y, 437.9, 1.25, 2.4))
            key = f"{name}-base-M2-{i}"
            stock = BRepAlgoAPI_Common(cyl(x, y, 432, 0.2, 7), bridge).Shape()
            below = round(bounds(stock)[2], 3)
            # Relieve the pocket edge across the complete washer and socket.
            # The pilot's local thickness alone can miss a neighboring rib.
            if below > 433.1:
                bridge = cut(bridge, cyl(x, y, 432, 3.35, below - 432))
            bridge = cut(bridge, cyl(x, y, below - 0.1, 1.10, 438.2 - below))
            bracket = cut(bracket, cyl(x, y, 440.2, 2.15, 1.60))
            length = 12 if below < 434 else 8
            made[key] = fuse(
                cyl(x, y, 440.2 - length, 1, length), cyl(x, y, 440.2, 1.9, 1.4)
            )
            made[key + "-washer"] = tube(x, y, below - 0.4, 5, 2.2, 0.4)
            made[key + "-locknut"] = cut(
                hexagon(x, y, below - 2.8, 4, 2.4), cyl(x, y, below - 3, 1, 3)
            )
            tools[key + "-below"] = tube(x, y, below - 22, 6.5, 4.9, 21.60)
            tools[key] = cyl(x, y, 441.6, 1.15, 24)
        made[name + "-bracket"] = bracket
        # Stylized 0.30 mm spring blade at the actual assembly state. The
        # contact endpoint and the free-state data are kept separately.
        tip = a + (6.45 if label == "retracted" else 10)
        blade = slab(
            [(X(0.4), a + 5), (X(14), tip - 0.3), (X(14), tip), (X(0.4), a + 5.3)],
            CAM_Y - 1.5,
            CAM_Y + 1.5,
        )
        made[name + "-lever"] = blade
        stations[label] = dict(
            centerXmm=center,
            axisZmm=a,
            freeTipUpperZmm=a + 10,
            operatingTipZmm=a + 6.8,
            atEndpointCamZmm=CAM_BOTTOM,
            overtravelNominalMm=0.35,
            slotHalfTravelMm=1.65,
        )
    made["support-machined-bridge"] = bridge
    return made, tools, stations


def check():
    issued = HERE / "issue"
    allparts = {k: s for k, *_, s in reopen()} | base.named(
        issued / "station-and-jack-interfaces.step"
    )
    for k in json.loads((issued / "interfaces.json").read_bytes())["fixedHeight"][
        "removed"
    ]:
        allparts.pop(k)
    allparts.update(base.named(HERE / "battery-fit/battery-retention.step"))
    allparts.update(base.named(HERE_OUT / "grip-carriers.step"))
    for k in json.loads((HERE_OUT / "grip-fit.json").read_bytes())["removed"]:
        allparts.pop(k)
    made, tools, stations = make(allparts)
    selected = allparts | made
    selected.pop("slide-middle-stop-dog-leg")
    boxes = {k: bounds(s) for k, s in selected.items()}
    hits = {}
    for k, s in made.items():
        if k == "support-machined-bridge":
            continue
        for j, v in collisions(s, selected, boxes, {k}).items():
            hits[" / ".join(sorted([k, j]))] = v
    toolhits = {k: collisions(s, selected, boxes, {k}) for k, s in tools.items()}
    step = HERE_OUT / "switch-mounts.step"
    write_step(made, step)
    document(
        HERE_OUT / "switch-fit.json",
        dict(
            source=sha(issued / "station-and-jack-interfaces.step"),
            drawing=sha(step),
            stations=stations,
            revisedBodies=len(made),
            installedIntersectionsMm3=hits,
            installedTools=toolhits,
            limitations="The catalog permits +/-1.5 mm operating position. Each actual contact needs a measured setup in the supplied slots. A flat cam at the endpoint holds after operation. The published actuator minimum overtravel is 0.55 mm. Lever-force, terminal dressing and worn operation need separate acceptance.",
        ),
    )
    print(hits, toolhits, flush=True)


if __name__ == "__main__":
    check()
