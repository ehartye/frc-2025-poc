"""Re-run the installed accounting engine on the native issue and an incorrect foam assignment."""

import argparse
import csv
import hashlib
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

from issue import document


def run(folder, checker):
    def invoke(directory):
        return subprocess.run(
            ["node", str(checker), str(directory / "mapping.json")],
            capture_output=True,
            encoding="utf8",
        )

    scope = invoke(folder)
    if scope.returncode:
        raise RuntimeError(scope.stderr)
    result = json.loads(scope.stdout)["nativeMass"]
    original = json.loads((folder / "engineering-reopen.json").read_bytes())
    with tempfile.TemporaryDirectory() as scratch:
        tmp = Path(scratch)
        mapping = json.loads((folder / "mapping.json").read_bytes())
        for item in mapping["sources"]:
            shutil.copyfile(folder / item["file"], tmp / item["file"])
        target = next(
            r
            for r in original["bodies"]
            if r.get("stock") == "Expanded polyethylene, 37 kg per cubic metre"
        )
        target["materialId"] = "PrismMaterial-018"
        altered = tmp / "engineering-reopen.json"
        document(altered, original)
        next(row for row in mapping["sources"] if row["id"] == "native")["sha256"] = (
            hashlib.sha256(altered.read_bytes()).hexdigest()
        )
        document(tmp / "mapping.json", mapping)
        bad = invoke(tmp)
        if bad.returncode == 0 or "physical material mismatch" not in bad.stderr:
            raise RuntimeError("Material counterexample failed: " + bad.stderr)
    report = {k: v for k, v in result.items() if k != "allowanceItems"}
    report["allowanceItemCount"] = len(result["allowanceItems"])
    report["installedCheckerSha256"] = hashlib.sha256(checker.read_bytes()).hexdigest()
    report["negativeMaterialControl"] = bad.stderr.strip()
    values = {}
    with (folder / "material-register.csv").open(encoding="utf8") as stream:
        for row in csv.DictReader(stream):
            values[
                hashlib.sha256(
                    ("team-1780-engineering-v1/" + row["id"]).encode()
                ).hexdigest()
            ] = float(row["nativeNominalDensityKgM3"])
    delta = {}
    for r in json.loads((folder / "engineering-reopen.json").read_bytes())["bodies"]:
        observed = r["massKg"] / (r["volumeCm3"] * 1e-6)
        expected = values[r["identity"]]
        if abs(observed - expected) > 1.0:
            key = r["materialName"]
            d = delta.setdefault(
                key,
                dict(
                    bodies=0,
                    observedKgM3=round(observed, 2),
                    billNominalKgM3=expected,
                    rawMinusBillNominalKg=0,
                ),
            )
            d["bodies"] += 1
            d["rawMinusBillNominalKg"] += r["massKg"] - r["volumeCm3"] * 1e-6 * expected
    report["displayDensityDifferences"] = delta
    document(folder / "mass-review.json", report)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder", type=Path)
    p.add_argument("checker", type=Path)
    a = p.parse_args()
    run(a.folder.resolve(), a.checker.resolve())
