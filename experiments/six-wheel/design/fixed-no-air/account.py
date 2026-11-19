"""Reconcile the dedicated fixed-height match robot with the pneumatic group omitted."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE.parent / "signal-light"))
from accounting import build_map, locks

GROUP = "compressor, reservoir, valve, regulator, gauges and fittings"
PREFIX = ("tank-", "viair-", "gauge-regulator-", "regulated-pawl-")


def write(path, value):
    path.write_bytes((json.dumps(value,indent=2) + "\n").encode())


def qualify(mapping, installed, branches, mode):
    if mode not in ("high", "low"):
        raise ValueError("Air deletion requires a fixed-height configuration")
    for item in mapping["exports"]:
        assigned = {r["instance"] for r in mapping["assignments"] if r["export"] == item["id"]
                    and r["role"] == "installed" and r["occurrence"].startswith(PREFIX)}
        if installed != assigned:
            raise ValueError("Removal must cover the complete pneumatic group in every pose")
    channels = {k:v for k,v in branches["sensorChannels"].items() if k.startswith("PH")}
    if set(channels) != {"PHswitch","PH0"} or "M4V210" not in channels["PH0"]:
        raise ValueError("Pneumatic channels require functional review")


def make_record():
    native = locks.read(HERE/"native.json")
    source = HERE.parent/"electrical-inspection/inspection.json"
    low,high = locks.read(source)["projectMassAllowancesKg"][GROUP]
    power_file = ROOT/"fixed-low-power/results.json"
    result = locks.read(power_file)
    circuit_file = HERE.parent/"electrical-inspection/circuits.json"
    if result["cadAndContactSourceHashes"][circuit_file.relative_to(ROOT).as_posix()] != locks.sha(circuit_file):
        raise ValueError("Circuit map no longer matches the electrical replay")
    fixed = result["cases"]["park372-fixed"]
    if fixed["compressorA"] or fixed["branches"]["Lift"]["peakPhaseA"] or fixed["branches"]["PH"]["peakBranchA"] <= 0:
        raise ValueError("The fixed-low replay does not isolate lift and compressor while retaining the PH")
    installed = sorted(native["cases"]["high"]["air_installed_neighbours_within_2mm"])
    for case in native["cases"].values():
        if sorted(case["air_installed_neighbours_within_2mm"]) != installed:
            raise ValueError("Kit membership differs by pose")
    paths = [source,power_file,circuit_file,HERE/"native.json",HERE.parent/"stability/stability.py",
             HERE.parent/"competition-package/shop-duplex/mass.json"]
    return {"source_sha256":{p.relative_to(ROOT).as_posix():locks.sha(p) for p in paths},
            "removed":installed,"signed_group_delta_kg":[-high,-low],"fixed_low_ph_peak_A":fixed["branches"]["PH"]["peakBranchA"]}


def no_air(plugin, output, mapping, name, mode, record, circuits):
    qualify(mapping,set(record["removed"]),circuits,mode)
    source = HERE/"deletion.json"
    mapping["sources"].append({"id":"air-package","file":Path(os.path.relpath(source,output)).as_posix(),
                               "sha256":locks.sha(source)})
    change = {"id":"no-match-air","exclusiveGroup":"pneumatic-configuration","remove":record["removed"],
              "add":[],"delta":locks.evidence("air-package","/signed_group_delta_kg",record["signed_group_delta_kg"])}
    mapping["changes"].append({**change,"configuration":mapping["configuration"],"revision":mapping["revision"]})
    mapping["selected"].append(change["id"])
    path = output/(name+"-no-air.json")
    locks.write_map(path,mapping)
    check = json.loads(subprocess.check_output(["node",str(plugin/"scripts/assembly-accounting.mjs"),str(path)]))
    ids = set(check["make"]+check["undecided"])
    for article in check["buy"]:
        ids.update(article["instances"])
    if set(record["removed"]) & ids or not {"rev-ph","sport-ratchet-and-mount-reserve","long-common-shaft"} <= ids:
        raise ValueError("Removed and retained equipment disagree with the selected configuration")
    return ids,check


def compare(plugin, output):
    record = make_record()
    source = HERE/"deletion.json"
    write(source,record)
    mass_delta = record["signed_group_delta_kg"]
    inspection = locks.read(HERE.parent/"electrical-inspection/inspection.json")
    circuits = locks.read(HERE.parent/"electrical-inspection/circuits.json")
    bumpers = locks.read(HERE.parent/"shared-height/inspection.json")["bumperAllowanceKg"]
    nominal = locks.read(HERE.parent/"signal-light/accounting.json")["cases"]
    results = {}
    for color in ("red","blue"):
        for case in (("high","low","low-camera") if color=="red" else ("high","low")):
            mode = "high" if case=="high" else "low"
            before = nominal[mode]["representedRangeKg"]
            if case=="low-camera":
                observer = ROOT/"moving-observer"
                subprocess.run([sys.executable,str(observer/"accounting.py"),"--plugin",str(plugin),
                                "--output",str(output)],check=True)
                mapping = locks.read(output/"moving-camera.json")
                before = locks.read(observer/"accounting.json")["represented_kg"]
            else:
                mapping = build_map(plugin,output,mode,color)
            ids,check = no_air(plugin,output,mapping,f"{case}-{color}",mode,record,circuits)
            if case=="low-camera" and {k for k in ids if k.startswith("roof-")} != {
                    "roof-"+k for k in locks.read(observer/"mount.json")["parts"]}:
                raise ValueError("Installed optical articles changed")
            # The group is an independent summand of the issued baseline itself. At each
            # bound cancel exactly the same term. The checker separately reports an
            # outer bound when the delta and prior baseline vary independently.
            matched = [before[0]+mass_delta[1],before[1]+mass_delta[0]]
            outer = check["mass"]["representedRangeKg"]
            if outer[0] > matched[0] or outer[1] < matched[1] or check["coverage"]["missing"]:
                raise ValueError("Native coverage or paired-budget reconciliation failed")
            results[f"{case}-{color}"] = {"represented_items":len(ids),"checker_mass_kg":outer,
                "checker_upper_headroom_kg":check["mass"]["representedMarginKg"][0],
                "paired_R103_kg":matched,"paired_upper_headroom_kg":inspection["R103LimitKg"]-matched[1],
                "paired_R408_kg":[m+b for m,b in zip(matched,bumpers)],
                "paired_bumper_on_upper_headroom_kg":inspection["R408LimitKg"]-matched[1]-bumpers[1],
                "finished_margin_kg":check["mass"]["configurationMarginKg"]}
    return {"plugin":locks.read(plugin/"package.json")["version"],"cases":results}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plugin",type=Path)
    parser.add_argument("output",type=Path)
    args = parser.parse_args()
    write(HERE/"inspection.json",compare(args.plugin.resolve(),args.output.resolve()))
