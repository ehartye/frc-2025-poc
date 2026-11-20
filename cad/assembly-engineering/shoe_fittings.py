"""Positively retain the two shoe stop pins and couple the guide block to its stem."""

from OCP.gp import gp_Pnt, gp_Dir, gp_Ax2
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from attachments import cyl, cut, fuse, base, hexagon
from installed_audit import bounds


def cross(x, y, z, r, h, axis):
    return BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(x, y, z), gp_Dir(*axis)), r, h
    ).Shape()


def apply(stock, body):
    carrier = stock["Shoe dovetail carrier"]
    # Pull each wire-EDM guide through the entire 58 mm length. Four R0.50
    # corner reliefs clear a sharp-ended presized 11.50 x 12.00 mm shoe.
    for x, sgn, name in (
        (293, 1, "Positive bearing shoe"),
        (307, -1, "Negative bearing shoe"),
    ):
        for dx in (-5.8, 5.8):
            for zz in (467.95, 480.05):
                carrier = cut(carrier, cross(x + dx, -29.2, zz, 0.5, 58.4, (0, 1, 0)))
        y = sgn * 12
        # Captive custom shoulder pin, ground Ø3, OD 4.8 turned head; M2.5x.45
        # below the lower housing face. Buy matching all-metal locking nuts.
        stem = cyl(x, y, 459, 1.5, 25)
        stem = fuse(stem, cyl(x, y, 484, 2.4, 1.3))
        stem = fuse(stem, cyl(x, y, 454.8, 1.25, 4.2))
        stock[name + " stop pin"] = stem
        stock[name + " M2.5 stop locknut"] = cut(
            hexagon(x, y, 456, 5, 3), cyl(x, y, 455, 1.25, 5)
        )
    # M12x1.5 blind seating thread smooth nominal; its separate cross pin
    # resists loosening without crediting thread friction. The shaft shoulder is
    # registered to the actual 459 mm lower guide face.
    carrier = cut(carrier, cyl(300, 0, 458.8, 6, 7.5))
    bore = cross(284.8, 0, 462.35, 1.0, 30.4, (1, 0, 0))
    stem = cut(body["Latch stem"], base.block((280, -15, 459, 320, 15, 462)))
    stem = fuse(stem, cyl(300, 0, 459, 6.0, 7))
    stock["Latch stem"] = cut(stem, bore)
    stock["Shoe dovetail carrier"] = cut(carrier, bore)
    stock["Stem antirotation spring pin"] = cut(
        cross(285.25, 0, 462.35, 1.0, 29.5, (1, 0, 0)),
        cross(285, 0, 462.35, 0.55, 30, (1, 0, 0)),
    )
