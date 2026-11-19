"""Reopen the selected structural replacements and bound their individual mass locations."""

import argparse
import json
import subprocess
from native_parcels import CAD, HERE, ROOT, digest, dump, np, prop, read
from pathlib import Path


def run(plugin, account):
    issued = read(HERE / "parcels.json")
    if digest(account) != issued["accountSha256"]:
        raise ValueError("Different robot")
    ids = subprocess.check_output(
        ["node", str(plugin / "scripts/assembly-accounting.mjs"), str(account)]
    )
    report = json.loads(ids)
    installed = (
        set(report["make"])
        | set(report["undecided"])
        | {i for a in report["buy"] for i in a["instances"]}
    )
    if len(installed) != 1189:
        raise ValueError("Different installed inventory")
    out = {}
    sources = {}
    used = {
        s for p in issued["parcels"].values() for s in p.get("bodies", [p.get("body")])
    }
    for folder, file, key, register in [
        ("guide-setting", "setting.step", "volumesMm3", "fit.json"),
        ("guide-mounts", "guide-mounts.step", "volumesMm3", "fit.json"),
        ("carriage-support", "carriage-support.step", "volumesMm3", "fit.json"),
        ("mast-attachment", "mast-joints.step", "volumeMm3", "fit.json"),
        ("deck-joints", "deck-through-ties.step", "volumeMm3", "fit.json"),
    ]:
        fit = read(ROOT / folder / register)
        step = ROOT / folder / file
        if digest(step) != fit["stepSha256"]:
            raise ValueError(file)
        sources[folder] = digest(step)
        shapes = CAD.read_named(step, [{"id": n} for n in fit[key]])
        for name, s in shapes.items():
            article = "match-drilled/" + name if folder == "guide-setting" else name
            if article not in installed or article in used:
                continue
            # Drill/replacement layers supersede earlier physical parts even where
            # the two source exchanges use the same display name.
            if folder != "guide-setting" and "match-drilled/" + name in installed:
                continue
            v, c, b = prop(s)
            if abs(fit[key][name] - v) > max(0.1, 1e-6 * v):
                raise ValueError(name)
            # Simulated shop bounds. Allow a 3% min/max volume excursion and a wider
            # effective density interval for thread reductions, wrench flats and hardware.
            if name.endswith("-liner"):
                low, high = 0.30, 0.70
            elif any(token in name for token in ("screw", "washer", "nut")):
                low, high = 5.0, 8.20
            elif name.endswith("-anchor"):
                # An insert may be made from a denser stock than the base plate.
                low, high = 2.50, 8.20
            elif any(
                token in name
                for token in (
                    "-rail",
                    "-mast",
                    "-splice",
                    "-sleeve",
                    "-bracket",
                    "-arm",
                    "-bridge",
                    "-crossbar",
                    "-cross-",
                    "-backing",
                    "-bar",
                    "-wedge",
                    "-deck",
                )
            ):
                low, high = 2.50, 2.95
            else:
                continue
            # The center of a bought or composite part can lie anywhere in its envelope.
            # Uniform shop stock has an added/missing-volume center tolerance of 3% of
            # the full span / 0.97, separately on all three axes.
            if high < 3 and not name.endswith("-liner"):
                e = 0.03 * (b[3:] - b[:3]) / 0.97
                loc = np.r_[c - e, c + e]
            else:
                loc = b
            out[article] = {
                "body": article,
                "kg": [0.97 * v * low / 1e6, 1.03 * v * high / 1e6],
                "positionMm": loc.tolist(),
                "source": folder,
                "volumeMm3": v,
            }
            used.add(article)
    return {"registeredSources": sources, "installed": len(installed), "parcels": out}


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("plugin", type=Path)
    p.add_argument("map", type=Path)
    a = p.parse_args()
    dump(HERE / "kits.json", run(a.plugin, a.map))
