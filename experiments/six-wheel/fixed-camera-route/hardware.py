"""Original harness guides, straps and supplier-envelope registrations in robot millimetres."""

import numpy as np
from paths import F, L, U, Z, DATA, POWER, T, B, PI, host, routes, points, unit
from scene import m
from camera_parts import box, cylinder, cut, unite

RADIUS = {"data": 1.70, "power": 4.35, "pigtail": 2.10, "roof-host": 3.15}
# Issued open guide troughs and removable closures reserve 1.25 mm outside the
# accepted jacket. The native half-round support geometry is exported in guides.py.

WALL = 1.25


RODS = {}


def rod(a, b, r):
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    s = cylinder(*a, r, float(np.linalg.norm(d)), unit(d))
    RODS[s] = (a, b, r)
    return s


def oriented(basis, origin, lo, hi):
    return m.vendor.transform(box(*lo, *(np.asarray(hi) - lo)), np.column_stack(basis), origin)


def near(path, z):
    p = points(path)
    return p[np.argmin(abs(p[:, 2] - z))]


def jacket_lug(path, key, base, target):
    pts = points(path)
    i = int(np.argmin(np.linalg.norm(pts - target, axis=1)))
    tangent = unit(pts[min(i + 3, len(pts) - 1)] - pts[max(0, i - 3)])
    d = np.asarray(base) - pts[i]
    direction = unit(d - tangent * (tangent @ d))
    # The end lands on an external tab. The first 1.5 mm of cap remains outside the jacket.
    return pts[i] + (RADIUS[key] + 2.4) * direction


def solids():
    path, board = routes()
    parts = {}
    # The supplier Pi drawing specifies plan-envelope and hole datums. The height
    # interval includes the board and tallest enumerated housing; use measured port Z on receipt.
    parts["host-board-and-components"] = oriented(
        (T, B, Z), PI, [-42.5, -28, 163.5], [45.5, 28, 181.1]
    )
    parts["usb-A-fixed-boot"] = oriented(
        (T, B, Z), host(-202, -200, 169), [0, -7.75, -3.75], [35, 7.75, 3.75]
    )
    parts["usb-A-roof-boot"] = oriented((T, B, Z), host(-202, -200, 177), [0, -8, -4], [38, 8, 4])
    parts["usb-C-datum-side-exit"] = oriented((F, L, U), DATA, [-29, -3.5, 0.35], [6, 3.5, 16.25])
    parts["usb-C-power-boot"] = oriented((F, L, U), POWER, [-6.20, -3.65, 0.40], [6.2, 3.65, 24.95])
    # Existing fixed-head relief and adapter have already been interference-tested.
    # Split low-profile hood: actual terminal drawing and insulation have to fit this gross gauge.
    parts["transition-capsule"] = box(*(board - [10, 8, 22]), 20, 16, 22)
    # Four dual-face nylon shoes fasten to the observer mast with two reusable
    # bands. Route guide arms join close to both straight mast runs.
    anchor = {}
    for height in (335, 520, 715, 925):
        shoe = unite(box(-340, -13, height - 5, 23, 3, 10), box(-340, -10, height - 5, 3, 21, 10))
        name = f"mast-shoe-{height}"
        parts[name] = shoe
        for k, start in (("data", [-340, 8, height]), ("power", [-335, -13, height])):
            if height == 335 and k == "data":
                middle = [-364, -4, 337]
                parts[name + "-data-elbow"] = rod(start, middle, 2)
                start = middle
            end = jacket_lug(path[k], k, start, near(path[k], height))
            parts[f"{name}-{k}-arm"] = rod(start, end, 2.0)
        anchor[name] = "observer-upright"
    # A triangulated head bracket supports the protected transition capsule and
    # a retainer close behind the first data bend. All joints are on the retained mast.
    parts["transition-support-a"] = rod([-335, -13, 925], [-335, -51, 941], 2)
    parts["transition-support-elbow"] = rod([-335, -51, 941], [-336, -69, 1009], 2)
    parts["transition-support-upper"] = rod([-336, -69, 1009], board - [7, -8, 8], 2)
    parts["transition-support-b"] = rod([-319, -13, 930], [-317, -78, 935], 2)
    parts["transition-support-outboard"] = rod([-317, -78, 935], [-302, -79, 1007], 2)
    parts["transition-support-cap"] = rod([-302, -79, 1007], board + [8, 7, -5], 2)
    parts["head-data-support"] = rod(
        [-340, 8, 930], jacket_lug(path["data"], "data", [-340, 8, 930], near(path["data"], 979)), 2
    )
    parts["head-pigtail-support"] = rod(
        board + [6, -7, -10],
        jacket_lug(path["pigtail"], "pigtail", board + [6, -7, -10], board + 8 * Z),
        1.7,
    )
    # Base plate standoff holes are the four positions on the official Pi drawing.
    for x in (-39, 19):
        for y in (-24.5, 24.5):
            pos = PI + x * T + y * B
            name = f"host-post-{int(x)}-{int(y * 10)}"
            parts[name] = cut(
                cylinder(pos[0], pos[1], 157, 2.9, 6.5), cylinder(pos[0], pos[1], 156.9, 1.35, 6.7)
            )
            anchor[name] = "observer-pi-and-converter-backplate"
    # Shoes attach on existing solid tops. Triangulated wire forms hold the
    # long off-mast runs. Feet have two M3 clearance holes for a backing plate.
    for group, origin, substrate, targets in (
        (
            "base-guide",
            [-275, -226, 157],
            "observer-pi-and-converter-backplate",
            [("data", [-253, -234, 235]), ("data", [-156, -270, 193])],
        ),
        (
            "roof-guide",
            [123, -214, 293],
            "front-electronics-platform",
            [
                ("roof-host", [-6, -266, 322]),
                ("roof-host", [112, -309, 451]),
                ("roof-host", [85, -317, 597]),
            ],
        ),
        (
            "base-power",
            [-338, -23, 158],
            "observer-perforated-foot",
            [("power", [-338, -29, 310]), ("power", [-252, -130, 320])],
        ),
    ):
        x, y, z = origin
        pad = box(x - 8, y - 5, z, 16, 10, 3)
        for dx in (-5, 5):
            pad = cut(pad, cylinder(x + dx, y, z - 0.5, 1.65, 4))
        parts[group + "-foot"] = pad
        anchor[group + "-foot"] = substrate
        for num, (k, target) in enumerate(targets):
            ref = np.asarray(target)
            start = [x, y, z + 3]
            if group == "roof-guide" and num == 1:
                middle = [102, -214, 357]
                parts[group + "-lamp-elbow"] = rod(start, middle, 2)
                start = middle
            if group == "roof-guide" and num == 2:
                middle = [65, -271, 568]
                parts[group + "-tower"] = rod(start, middle, 2)
                start = middle
            end = jacket_lug(path[k], k, start, ref)
            parts[group + "-rib-" + str(num)] = rod(start, end, 2)
    return parts, anchor


def service():
    # Pull each plug 12 mm along the mating axis after opening the local clamps.
    # Side fingers grip the long side of the shell; no finger volume is claimed inside the 8 mm stack.
    return {
        "fixed-C-data": oriented((F, L, U), DATA, [-30, -5, 0.35], [7, 5, 28.25]),
        "fixed-C-power": oriented((F, L, U), POWER, [-7.2, -5, 0.40], [7.2, 5, 36.95]),
        "host-A-fixed": oriented(
            (T, B, Z), host(-202, -200, 169), [0, -7.75, -3.75], [47, 7.75, 3.75]
        ),
        "host-A-roof": oriented((T, B, Z), host(-202, -200, 177), [0, -8, -4], [50, 8, 4]),
    }
