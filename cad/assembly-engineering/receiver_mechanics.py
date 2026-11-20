"""Connect both cartridge plates to their moving cheeks and capture all four guide rods."""

import argparse
import json
import math
from pathlib import Path
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Trsf, gp_Ax1, gp_Ax2, gp_Dir, gp_Pnt
from OCP.TopLoc import TopLoc_Location
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from attachments import base, bounds, common, cut, cyl, dist, fuse, hexagon, write_step
from battery_fit import collisions
from clearance import move
from installed_audit import HERE, reopen, sha
from issue import document

FLANGE = "cd737b615787ed7b8d3c20631712aabdccb619c9eedf9409444fec4243b35068"
OUTPUT = HERE / "receiver-service"
Y0 = 183.5625
X0 = -84.5


def rotate(shape, direction, angle, center=(0, 0, 0)):
    t = gp_Trsf()
    t.SetRotation(gp_Ax1(gp_Pnt(*center), gp_Dir(*direction)), angle)
    return shape.Moved(TopLoc_Location(t))


def add_parts(parts):
    made = {}
    tools = {}
    rooms = {}
    for side, sign in (("L", 1), ("R", -1)):
        prefix = "grip-" + side + "-"
        y = sign * Y0

        def rect(xlo, ylo, zlo, xhi, yhi, zhi):
            yr = sorted((sign * ylo, sign * yhi))
            return base.block([xlo, yr[0], zlo, xhi, yr[1], zhi])

        def cy(x, ypos, z, r, h):
            return BRepPrimAPI_MakeCylinder(
                gp_Ax2(gp_Pnt(x, sign * ypos, z), gp_Dir(0, sign, 0)), r, h
            ).Shape()

        def stackhex(x, ypos, z, af, h):
            return move(
                rotate(hexagon(0, 0, 0, af, h), (1, 0, 0), -sign * math.pi / 2),
                (x, sign * ypos, z),
            )

        # Lift the true output face from 581 to 584 so the projecting
        # rotating boss clears the 572..577 spur. Long shaft keeps its axis.
        plate = rect(-103.5, 164.5625, 581, -65.5, 202.5625, 584)
        plate = fuse(plate, rect(-76, 156, 581, -44, 174.5, 584))
        plate = cut(plate, cyl(X0, y, 580, 11.35, 5))
        cheek = parts[prefix + "upper-cheek"]
        centers = [(-61.5, 158.5), (-61.5, 165.5), (-49.5, 158.5), (-49.5, 165.5)]
        for x, yy in centers:
            # Machined lugs integral with the original upper cheek.
            boss = cyl(x, sign * yy, 570.95, 3.4, 10.05)
            cheek = fuse(cheek, boss)
            cheek = cut(cheek, cyl(x, sign * yy, 570.5, 1.25, 11.0))
            plate = cut(plate, cyl(x, sign * yy, 580.9, 1.70, 3.3))
            stud = fuse(
                cyl(x, sign * yy, 574, 1.5, 10), cyl(x, sign * yy, 584, 2.75, 3)
            )
            label = prefix + f"cheek-tie-{x}-{yy}"
            made[label] = cut(stud, hexagon(x, sign * yy, 585, 2.5, 3))
            tools[label] = cyl(x, sign * yy, 587, 1.65, 30)
        holes = []
        for dx in (-math.sqrt(192), math.sqrt(192)):
            for dy in (-8, 8):
                x, yy = X0 + dx, sign * (Y0 + dy)
                holes.append((x, yy))
                plate = cut(plate, cyl(x, yy, 580.9, 1.65, 3.3))
                screw = fuse(cyl(x, yy, 581, 1.5, 8), cyl(x, yy, 578, 2.75, 3))
                label = (
                    prefix
                    + f"output-tie-{'east' if dx > 0 else 'west'}-{'outer' if dy > 0 else 'inner'}"
                )
                made[label] = cut(screw, hexagon(x, yy, 577.9, 2.5, 1.7))
                tools[label] = cyl(x, yy, 553, 1.60, 25)
        casing = move(parts[prefix + "UP5"], (0, 0, 3))
        for x, yy in holes:
            casing = cut(casing, cyl(x, yy, 583.9, 1.25, 7.1))
        made[prefix + "UP5"] = casing
        made[prefix + "NEO550"] = move(parts[prefix + "NEO550"], (0, 0, 3))
        made[prefix + "cartridge-mount"] = plate
        made[prefix + "upper-cheek"] = cheek
        rooms[side] = holes
        for front, x in (("rear", -73), ("front", 82)):
            name = prefix + front
            tower = parts[name + "-tower"]
            # Shoulder seats inside its counterbore at 186.5625. Flush on the
            # spring side so the original 14 mm spring working space remains.
            tower = cut(tower, cy(x, 185.50, 543.5, 2.50, 1.0625))
            made[name + "-tower"] = tower
            pin = cy(x, 158.5625, 543.5, 1.50, 27)
            pin = fuse(pin, cy(x, 185.5625, 543.5, 2.40, 1.00))
            pin = fuse(pin, cy(x, 186.5625, 543.5, 1.5, 7.00))
            pin = fuse(pin, cy(x, 193.5625, 543.5, 1.25, 6.00))
            made[name + "-slide-guide"] = pin
            made[name + "-guide-washer"] = cut(
                cy(x, 193.5625, 543.5, 3.00, 0.5), cy(x, 193.50, 543.5, 1.35, 0.7)
            )
            made[name + "-guide-locknut"] = cut(
                stackhex(x, 194.0625, 543.5, 5, 3.5), cy(x, 194.0, 543.5, 1.25, 3.7)
            )
    return made, tools, rooms


def actual_output(source):
    if sha(source) != FLANGE:
        raise ValueError(
            "The independently checked REV-41-1604 output revision is required"
        )
    read = STEPControl_Reader()
    if read.ReadFile(str(source)) != IFSelect_RetDone:
        raise ValueError("Unopened gear")
    read.TransferRoots()
    return list(base.subsolids(read.OneShape()))


def check(source):
    issued = HERE / "issue"
    old = {k: s for k, *_, s in reopen()} | base.named(
        issued / "station-and-jack-interfaces.step"
    )
    for k in json.loads((issued / "interfaces.json").read_bytes())["fixedHeight"][
        "removed"
    ]:
        old.pop(k)
    old.update(base.named(HERE / "battery-fit" / "battery-retention.step"))
    made, tools, holes = add_parts(old)
    gone = [
        "observer-rear-cable-raceway",
        "grip-L-motor-backstay",
        "grip-R-motor-backstay",
    ]
    selected = {k: s for k, s in (old | made).items() if k not in gone}
    bbox = {k: bounds(s) for k, s in selected.items()}
    hits = {}
    for k, s in made.items():
        for other, depth in collisions(s, selected, bbox, {k}).items():
            hits[" / ".join(sorted([k, other]))] = depth
    field = {}
    geometry = actual_output(source)
    case = geometry[1]
    rotor = geometry[4]
    for side, sign in (("L", 1), ("R", -1)):
        loc = (X0, sign * Y0, 584)
        prefix = "grip-" + side + "-"
        shell = move(case, loc)
        body = move(rotor, loc)
        gauge = {}
        for x, y in holes[side]:
            gauge[str([round(x, 4), round(y, 4)])] = {
                "shankMm3": common(cyl(x, y, 584.01, 1.2, 5), shell),
                "majorEngagementMm3": common(cyl(x, y, 584.01, 1.5, 5), shell),
            }
        plate = made[prefix + "cartridge-mount"]
        field[side] = dict(
            suppliedHoleGauges=gauge,
            gearboxToPlateGapMm=dist(shell, plate),
            gearboxToPlateMm3=common(shell, plate),
            housingPressBearingMm2=common(move(shell, (0, 0, -0.01)), plate) / 0.01,
            trueRotorToPlateMm3=common(body, plate),
            trueRotorToGearGapMm=dist(body, old[prefix + "opposed-22T"]),
            inheritedGearInterferenceMm3=common(
                move(body, (0, 0, -3)), old[prefix + "opposed-22T"]
            ),
            inheritedPlateInterferenceMm3=common(
                move(body, (0, 0, -3)), old[prefix + "cartridge-mount"]
            ),
        )
    toolhits = {k: collisions(s, selected, bbox, {k}) for k, s in tools.items()}
    OUTPUT.mkdir(exist_ok=True)
    step = OUTPUT / "grip-carriers.step"
    write_step(made, step)
    document(
        OUTPUT / "grip-fit.json",
        dict(
            issuedSha256=sha(issued / "station-and-jack-interfaces.step"),
            batterySha256=sha(HERE / "battery-fit" / "battery-retention.step"),
            sourceSha256=FLANGE,
            drawingSha256=sha(step),
            removed=gone,
            installedCount=len(selected),
            newOrChanged=len(made),
            observedIntersectionsMm3=hits,
            installedToolHits=toolhits,
            supplier=field,
        ),
    )
    print("Intersections", hits, "tools", toolhits, "supplier", field, flush=True)


if __name__ == "__main__":
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("source", type=Path)
    check(cli.parse_args().source.resolve())
