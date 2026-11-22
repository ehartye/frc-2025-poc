"""Verify each cold R3 pan stock at the independent commissioned placement."""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from output_mount import ROOT, digest, read
from pan_mechanics import rotation

DATA = ROOT / "artifacts/red2-integrated"
THRUST = DATA / "pan-thrust"


def compare(plugin):
    sys.path.insert(0, str(plugin / "scripts"))
    from component_shape import compare_components
    from showcase_contacts import load_solids

    shop = read(DATA / "pan-shop-plan.json")
    plan_file = THRUST / "r3-native-plan.json"
    plan = read(plan_file)
    native = THRUST / "native"
    cold_file = native / "pan-r3-stock-machining-01.json"
    cold = read(cold_file)
    step = native / "integrated-340mm-pan-r3-machined.step"
    archive = step.with_suffix(".f3d")
    authority = read(ROOT / "cad/fusion-authority.json")["source"]["archive"]
    if (authority != plan["authority"] or digest(ROOT / authority["file"]) != authority["sha256"]
            or cold["authority"]["archiveSha256"] != authority["sha256"]
            or digest(archive) != cold["nativeSha256"] or digest(step) != cold["stepSha256"]
            or cold["status"] != "machining-verified" or not cold["restored"]):
        raise ValueError("R3 archive and STEP must be issued from the resolved source")
    paths = {}
    placed, surfaces = load_solids(step, paths)
    if len(placed) != 297 or surfaces or cold["cold"]["totalBodies"] != len(placed):
        raise ValueError("R3 body roster changed")
    r2 = read(DATA / "pan-native/first-party-stock-r2.json")
    if r2["stepSha256"] != plan["parentStep"] or len(r2["allStock"]) != 68 or r2["failed"]:
        raise ValueError("Unexpected prior shop issue")
    candidates = {}
    for index, item in enumerate(shop["items"], 1):
        name = item["id"]
        old = r2["allStock"][name]
        candidate = DATA / "pan-native/independent-stock" / f"{index}.step"
        candidates[name] = (item["moving"], candidate, old["independentSourceSha256"])

    revised = cold["cold"]["parts"]
    if set(revised) != set(plan["stock"]):
        raise ValueError("Machining receipt does not cover the six commissioned stocks")
    for label, row in revised.items():
        reference = plan["stock"][label]
        moving = row["path"].startswith("340 mm pan rotating installation:1+")
        candidates[row["body"]] = (moving, ROOT / reference["file"], reference["sha256"])
    if len(candidates) != 71:
        raise ValueError("Expected 65 unchanged and six machined articles")

    jobs = []
    rows = {}
    for name, (moving, source, signature) in candidates.items():
        parent = "340 mm pan " + ("rotating" if moving else "fixed") + " installation"
        matches = [key for key, info in paths.items() if parent in info["occurrencePath"]
                   and name in info["occurrencePath"]]
        if len(matches) != 1 or digest(source) != signature:
            raise ValueError(f"Stock geometry or installed identity changed for {name}: {matches}")
        source_solids, other = load_solids(source)
        if len(source_solids) != 1 or other:
            raise ValueError("Independent stock is not a single solid: " + name)
        matrix = np.eye(4)
        if moving:
            matrix[:3] = rotation(70)
        solid = matches[0]
        if solid in rows:
            raise ValueError("Two expected articles resolve to " + solid)
        transform = matrix.flatten().tolist()
        jobs.append((solid, source, next(iter(source_solids)), transform))
        rows[solid] = dict(article=name, installedPath=paths[solid]["occurrencePath"],
                           source=source.relative_to(ROOT).as_posix(), sourceSha256=signature,
                           expectedTransformMm=transform)

    diffs = compare_components(step, jobs)
    if len(diffs) != 71 or set(diffs) != set(rows):
        raise ValueError("Incomplete shape comparison")
    failed = []
    for key, diff in diffs.items():
        rows[key]["differenceMm3"] = diff
        if any(not math.isfinite(v) or not 0 <= v < .025 for v in diff.values()):
            failed.append(key)
    result = dict(authority=authority, authorityRegisterSha256=digest(ROOT / "cad/fusion-authority.json"),
                  nativeSha256=digest(archive), stepSha256=digest(step),
                  coldReceiptSha256=digest(cold_file), sourcePlanSha256=digest(plan_file),
                  pluginSha256=digest(plugin / "scripts/component_shape.py"),
                  comparison="component_shape.compare_components", nativeSolids=len(placed),
                  stocks=rows, failed=failed)
    output = THRUST / "r3-cold-independent-stock.json"
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(f"{len(rows)} stock comparisons. Failures {failed}", flush=True)
    return not failed


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plugin", type=Path)
    raise SystemExit(0 if compare(parser.parse_args().plugin) else 1)
