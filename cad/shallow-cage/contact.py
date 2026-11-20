"""Register the exact closed-shoe support and reject closing before the aperture is cleared."""

import argparse
import hashlib
import json
import sys
from pathlib import Path
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Trsf
from clearance import HERE, ROOT, move, hanging_inventory, distance
from installed import load_parts, volume


def inspect(field):
    stations, shapes = hanging_inventory(field)
    receipt, parts = load_parts(HERE)
    out = {}
    for side, slot, sign in [("red", "NAUO260", -1), ("blue", "NAUO263", 1)]:
        station = stations[slot]
        x, z = station["axis_mm"]
        tr = gp_Trsf()
        tr.SetValues(sign, 0, 0, 300 - sign * x, 0, 0, -sign, sign * z, 0, 1, 0, 0)
        cage = shapes[station["cage"]].Moved(TopLoc_Location(tr))
        cases = {}
        for name, q, spread, lower in [
            ("raised-closed", 328, 0, 0),
            ("raised-spread", 328, 36, 0),
            ("top-seated", 319.4, 36, 0),
            ("clamped", 319.4, 36, 13.584),
            ("early-spread", 317, 36, 0),
        ]:
            contacts = {}
            for key, (group, shape) in parts.items():
                mm = q if group != "Shallow cage jack frame" else 0
                if group == "Lower plate clamp":
                    mm += lower
                d = (
                    spread
                    if group.startswith("Left cage")
                    else -spread
                    if group.startswith("Right cage")
                    else 0
                )
                item = move(shape, (0, d, mm))
                gap = distance(item, cage)
                if gap < 0.05:
                    overlap = volume(BRepAlgoAPI_Common(item, cage).Shape())
                    row = dict(gapMm=gap, intersectionMm3=overlap)
                    if name == "clamped" and ("shoe" in key or "Bearing pad" in key):
                        # 0.1mm penetration of nominal plane returns actual projected bearing area.
                        step = 0.1 if "Bearing pad" in key else -0.1
                        probe = move(item, (0, 0, step))
                        row["estimatedBearingAreaMm2"] = volume(
                            BRepAlgoAPI_Common(probe, cage).Shape()
                        ) / abs(step)
                    contacts[key] = row
            cases[name] = contacts
        out[side] = cases
    return dict(
        archiveSha256=receipt["sha256"],
        fieldSha256=hashlib.sha256(field.read_bytes()).hexdigest(),
        cases=out,
        scope="Nominal native material and intended bottom-plate faces. The 0.1 mm section is a contact-patch estimate; force capacity requires structural evidence.",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("field", type=Path)
    args = p.parse_args()
    (HERE / "contact.json").write_bytes(
        (json.dumps(inspect(args.field), indent=2) + "\n").encode()
    )
