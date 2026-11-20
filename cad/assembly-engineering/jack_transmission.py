"""Dimension the two top-mounted shallow-jack screw drives on the selected chassis.

All public motor/gear shapes are dimensioned installation gauges. Supplier tooth
and winding geometry is not redistributed. The rotor does not carry axial lift
load. The shaft, thrust pack, welded bearing plate and guide column do.
"""

import csv
import json
import math
from pathlib import Path

from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Pnt, gp_Vec

from attachments import (
    base,
    bounds,
    box_gap,
    common,
    cut,
    cyl,
    dist,
    fuse,
    hexagon,
    tube,
    volume,
    write_step,
)
from installed_audit import HERE, reopen, sha

ROTON = "https://www.roton.com/family/acme-right-screws-and-nuts-7059285/"
SKF = "https://www.skf.com/group/products/rolling-bearings/roller-bearings/needle-roller-bearings/needle-roller-thrust-bearings/productid-AXK%201024"
SOURCES = {
    "REV-21-1651": "4fdcdd973fecf09fc8d1851ebcc51220c34c7def9f30e82c21a57c5b72a7625c",
    "REV-41-1603": "1c350b3e3988759cdc5658ed434325c67863e70102ee36d9f8dde7ef0648e893",
}


def block(*args):
    return base.block(args)


def polygon(points, z, h):
    wire = BRepBuilderAPI_MakePolygon()
    for x, y in points:
        wire.Add(gp_Pnt(x, y, z))
    wire.Close()
    return BRepPrimAPI_MakePrism(
        BRepBuilderAPI_MakeFace(wire.Wire()).Face(), gp_Vec(0, 0, h)
    ).Shape()


def ratchet(x, y, z):
    # 24 perpendicular stop flanks, ramped backs. Shaft is Ø10 across the journal;
    # this section has a 8 mm across-flats integral drive, not a smooth free bore.
    border = []
    for tooth in range(24):
        a = tooth * math.tau / 24
        for radius, offset in ((16, 0), (19, 0), (19, 0.025)):
            border.append(
                (x + radius * math.cos(a + offset), y + radius * math.sin(a + offset))
            )
    return cut(polygon(border, z, 5), hexagon(x, y, z - 1, 8.02, 7))


def units(robot):
    result = {}
    web = robot["Crosshead front web"]
    for side, sign in (("Left", 1), ("Right", -1)):
        x, y = 305, 321 * sign

        def add(name, s):
            result[side + " " + name] = s

        # Purchased screw is turned from one blank. Effective length represents
        # stock remaining after cutting the threads off the two machined journals.
        shaft = cyl(x, y, 328, 15.875 / 2, 371)
        for s in [
            cyl(x, y, 310, 4.99, 18),
            cyl(x, y, 699, 4.99, 30),
            hexagon(x, y, 729, 8, 9),
            cyl(x, y, 738, 4, 8),
        ]:
            shaft = fuse(shaft, s)
        add("screw core", shaft)
        # The smooth shank of the purchased threaded-mount nut represents its
        # external 15/16-16 UN thread. The mating tapped lug represents the major
        # diameter; the nut flange bears on the lug face in the lifting direction.
        nut = fuse(
            tube(x, y, 345, 28.575, 15.875, 12.7),
            tube(x, y, 357.7, 23.8252, 15.875, 12.7),
        )
        add("travelling nut", nut)
        lug = cut(
            block(290, y - 16, 357.7, 326, y + 16, 370.4),
            cyl(x, y, 357, 23.8252 / 2, 15),
        )
        # Form the tapped thick end into a single welded-and-finished web assembly.
        # Retain 12 mm full-width connection to the front web inboard of the lug.
        tab = block(
            316,
            y - 21 if sign == 1 else y + 9,
            357.7,
            326,
            y - 9 if sign == 1 else y + 21,
            370.4,
        )
        web = fuse(fuse(web, lug), tab)
        web = cut(web, cyl(x, y, 344.95, 14.3375, 12.75))
        web = cut(web, cyl(x, y, 357.7, 23.8252 / 2, 12.71))
        web = cut(web, cyl(x, y, 370.4, 8.2, 26))
        # A lower bronze guide is radial only, attached to the column as a welded ear.
        add(
            "lower thrust bridge",
            cut(block(289, y - 12, 310, 318, y + 12, 319), cyl(x, y, 309, 7, 11)),
        )
        add("lower bronze radial bush", tube(x, y, 310, 14, 10.10, 9))
        # A welded box cap at Z700 carries the upward/negative axial directions
        # through rotating driver faces and standard hardened needle-bearing washers.
        add(
            "upper thrust bridge",
            cut(block(289, y - 15, 706, 325, y + 15, 714), cyl(x, y, 705, 5.15, 10)),
        )
        for end, z in (("under", 702), ("over", 714)):
            add(end + " thrust shaft washer", tube(x, y, z, 24, 10.05, 1))
            add(end + " AXK1024 bearing", tube(x, y, z + 1, 24, 10.05, 2))
            add(end + " thrust housing washer", tube(x, y, z + 3, 24, 10.05, 1))
        # Shoulder disk trapped directly between thread shoulder and first race.
        # Upper threaded retention is M10x0.75 on journal, tab washer/slot nut above.
        add("shoulder thrust disk", tube(x, y, 699, 24, 10.05, 3))
        add("upper thrust disk", tube(x, y, 718, 24, 10.05, 3))
        add("M10 tab washer", tube(x, y, 721, 22, 10.05, 1))
        add("M10 thrust locknut", cut(hexagon(x, y, 722, 18, 7), cyl(x, y, 721, 5, 9)))
        add("ratchet wheel", ratchet(x, y, 729))
        # Contact face lies on the radial tooth stop at phi=0. Pivot hole at X+30,
        # Y-4; the loaded reaction produces a clockwise torque into a positive stop.
        pivot_x, pivot_y = x + 30, y - 4
        pawl = polygon(
            [
                (x + 16.15, y),
                (x + 18.95, y),
                (x + 24, y - 1),
                (x + 33, y),
                (x + 35, y - 4),
                (x + 32, y - 8),
                (x + 28, y - 8),
                (x + 23, y - 4),
                (x + 18.9, y - 1.3),
            ],
            729.1,
            4.8,
        )
        pawl = fuse(pawl, block(x + 30.5, y - 4, 729.1, x + 34, y, 733.9))
        add("pawl", cut(pawl, cyl(pivot_x, pivot_y, 728, 2.03, 8)))
        add("pawl stop", block(x + 30.5, y, 729.1, x + 36, y + 2.5, 733.9))
        # ISO 7379-style 4 mm x 8 mm ground shoulder/M3 tapped top cheek.
        # Head holds lower cheek, ground shoulder spans lower cheek and pawl.
        shoulder = fuse(
            cyl(pivot_x, pivot_y, 723, 3.5, 3), cyl(pivot_x, pivot_y, 726, 1.99, 8)
        )
        shoulder = fuse(shoulder, cyl(pivot_x, pivot_y, 734, 1.49, 4))
        add("pawl pivot", cut(shoulder, hexagon(pivot_x, pivot_y, 722.9, 2, 2.2)))
        # Two structural side-cheeks sandwich the pawl. A fixed bumper takes
        # opening torque; the torsion spring closes the unloaded pawl on each tooth.
        for title, z in (("bottom", 726), ("top", 734)):
            cheek = block(x + 23, y - 9, z, x + 36, y + 2, z + 3)
            add(
                "pawl " + title + " cheek",
                cut(
                    cheek,
                    cyl(
                        pivot_x, pivot_y, z - 1, 2.04 if title == "bottom" else 1.50, 5
                    ),
                ),
            )
        # Joins cheek stock and motor-side housing to top bearing bracket only.
        # Cap hardware extends above the moving lug at full design travel.
        add("pawl spine", block(x + 36, y - 9, 714, x + 39, y + 3, 742))
        # Welded cheek and spine meet on their common vertical faces at X+36.
        add("pawl frame foot", block(325, y - 9, 711, x + 39, y + 3, 714))
        mount = block(271, y - 23, 742, 345, y + 23, 746)
        mount = cut(mount, cyl(x, y, 741, 7.30, 7))
        for i in range(6):
            a = math.pi / 2 + i * math.pi / 3
            px = x + 16 * math.cos(a)
            py = y + 16 * math.sin(a)
            mount = cut(mount, cyl(px, py, 741, 1.70, 6))
            # Six M3s install from below into vendor output case's blind taps.
            add(
                "gearbox M3-" + str(i),
                fuse(cyl(px, py, 739, 2.75, 3), cyl(px, py, 742, 1.5, 8)),
            )
        add("motor mounting plate", mount)
        # Carry this column directly to the deck. Its installed X ends at 283,
        # giving at least 3 mm to the swept 19 mm-radius ratchet.
        add("motor riser", block(274, y - 8, 711, 283, y + 8, 742))
        # Public project originals enclose the housing only; separate drive key,
        # Ø3.175 pinion and controller are charged as purchased units. The REV
        # drawings give one 10 mm cartridge and a 5 mm mounting plate.
        for label, z, h, w, d in [
            ("gear output REV-41-1604", 746, 12.25, 43.74, 40),
            ("5.23 stage REV-41-1603", 758.25, 10, 43.74, 38.5),
            ("NEO plate REV-41-1607", 768.25, 5, 43.74, 38.2),
        ]:
            box = block(x - w / 2, y - d / 2, z, x + w / 2, y + d / 2, z + h)
            if "output" in label:
                for i in range(6):
                    a = math.pi / 2 + i * math.pi / 3
                    box = cut(
                        box,
                        cyl(
                            x + 16 * math.cos(a),
                            y + 16 * math.sin(a),
                            z - 0.1,
                            1.5,
                            7.1,
                        ),
                    )
            add(label, box)
        add("NEO550 REV-21-1651", cyl(x, y, 773.25, 17.5, 44.5))
        add(
            "NEO550 connector reserve",
            cut(
                block(x + 8, y + 8, 801, x + 26, y + 26, 818), cyl(x, y, 799, 17.50, 25)
            ),
        )
        # Machine the top 8 mm of the screw as a real 5 mm hex rather than a round
        # stud. A double-ended 10/5 mm socket drives the hex input and screw.
        result[side + " screw core"] = cut(
            result[side + " screw core"], cyl(x, y, 738, 5, 9)
        )
        add(
            "shaft drive socket",
            cut(hexagon(x, y, 734, 12, 12), hexagon(x, y, 733, 8.02, 6)),
        )
        result[side + " shaft drive socket"] = fuse(
            result[side + " shaft drive socket"], hexagon(x, y, 746, 5, 8)
        )
        out = result[side + " gear output REV-41-1604"]
        result[side + " gear output REV-41-1604"] = cut(
            out, hexagon(x, y, 745.9, 5.05, 9)
        )
    result["Crosshead front web"] = web
    return result


def evaluate():
    robot = {k: s for k, _, _, _, _, s in reopen()}
    package = units(robot)
    legacy_removed = [side + " drive and pawl envelope" for side in ("Left", "Right")]
    selected = {k: v for k, v in robot.items() if k not in legacy_removed} | package
    boxes = {k: bounds(v) for k, v in selected.items()}
    clashes = []
    comparisons = 0
    for k, s in package.items():
        for other, t in selected.items():
            if (
                k == other
                or other in package
                and other < k
                or box_gap(boxes[k], boxes[other]) > 0.001
            ):
                continue
            comparisons += 1
            ov = common(s, t)
            if ov > 0.01:
                clashes.append(dict(a=k, b=other, mm3=round(ov, 4)))
    # Finite field loads, drive torque, ideal loss-of-power stop and interfaces.
    g = 9.80665
    one_side = 74 * g * 1.5 * 0.65
    lead = 0.003175
    screw_nm = one_side * lead / (math.tau * 0.32)
    shaft_rpm = 0.04 / lead * 60
    ratio = 68 / 13
    rotor_torque = screw_nm / (ratio * 0.75)
    peak_A = 1.4 + rotor_torque / (0.97 / 100)
    emf = (12 - 0.12 * 1.4) * (shaft_rpm * ratio) / 11000
    ideal_reverse = one_side * lead / math.tau
    # Test shaft/wheel motion against the finite held pawl at each sign. A
    # complete turn is unnecessary because all 24 tooth periods are identical.
    from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf
    from OCP.TopLoc import TopLoc_Location

    tooth = []
    y = 321
    for deg in (-7.5, -1.0, -0.1, 0.0, 0.1, 1.0, 7.5, 14.9):
        t = gp_Trsf()
        t.SetRotation(gp_Ax1(gp_Pnt(305, y, 729), gp_Dir(0, 0, 1)), math.radians(deg))
        rr = package["Left ratchet wheel"].Moved(TopLoc_Location(t))
        tooth.append(
            dict(
                deg=deg,
                gapMm=dist(rr, package["Left pawl"]),
                overlapMm3=common(rr, package["Left pawl"]),
            )
        )
    # Verify that the stop backs the loaded direction and leaves a release
    # direction at the actual pivot. Moving the pawl independently is a fault
    # control, not evidence of a return spring or a controller sensor.
    pivot = []
    for deg in (-18.0, -8.0, -1.0, 0.0, 1.0, 8.0):
        t = gp_Trsf()
        t.SetRotation(gp_Ax1(gp_Pnt(335, 317, 729), gp_Dir(0, 0, 1)), math.radians(deg))
        dog = package["Left pawl"].Moved(TopLoc_Location(t))
        pivot.append(
            dict(
                deg=deg,
                stopOverlapMm3=common(dog, package["Left pawl stop"]),
                wheelOverlapMm3=common(dog, package["Left ratchet wheel"]),
            )
        )
    part = HERE / "jack-transmission.step"
    write_step(package, part)
    report = dict(
        source=ROTON,
        nutSKU="90234",
        screwSKU="59285",
        vendorStepSha256=SOURCES,
        sourceBounds="Supplier geometry opened in OCCT; REV-41-1603 shell is 10 mm between mating planes. Extrema include internals. Public solids are conservatively bounded housings.",
        purchasedMotor="REV-21-1651 with REV-41-1607 plate, REV-41-1608 pinion, REV-41-1603 stage and REV-41-1604 output",
        thrustBearing=SKF,
        pairedThrustPlaneZmm=[702, 706, 714, 718],
        matchedInstalledVendorMounts=6,
        oneSideAxialN=one_side,
        nutOperatingN=1900 * 4.448221615,
        rotorA=peak_A,
        rotorRPM=shaft_rpm * ratio,
        driveTerminalV=emf + 0.12 * peak_A,
        shaftTorqueNm=screw_nm,
        pawlControls=pivot,
        toothControls=tooth,
        minorScrewDiameterMm=0.457 * 25.4,
        turnedJournalMm=9.98,
        teeth=24,
        oneToothSettlingMm=3.175 / 24,
        frictionlessStaticReverseNm=ideal_reverse,
        stopForceAtDoubleLoadN=2 * ideal_reverse / 0.0175,
        noGripOriginalShoeAtExtensionMm=36 - 18.5,
        assessedHeightRangeMm=[0, 328],
        ceilingMm=max(v[5] for k, v in boxes.items() if k in package),
        doubleCheckedPairCount=comparisons,
        stowedIntersections=clashes,
        publicStepSha256=sha(part),
        unresolved=[
            "Upper thrust reaction into the column and its welds, plus durable root and guide fit",
            "A 5 mm hex output converter to the screw and a retained, spring-closed pawl with positive casing stop",
            "Manufacturing and verification of both actual pawl and thrust load paths under proof load",
            "The cantilevered cage still sees the issued opposing shoe load; no permissible field reaction is established",
            "A paid-out unloaded downstroke needs a positively sensed pawl release",
            "Unsupported expanded shoes in the older carrier prevent permission to lift",
        ],
    )
    (HERE / "jack-transmission.json").write_bytes(
        (json.dumps(report, indent=2) + "\n").encode()
    )


if __name__ == "__main__":
    evaluate()
