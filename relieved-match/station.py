"""Check both same-revision feeds against the exact registered bodies and stamped stereo frames."""
import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT/"return-station"))
import registration
spec = importlib.util.spec_from_file_location("station_occupancy", ROOT/"return-station/native.py")
native = importlib.util.module_from_spec(spec)
spec.loader.exec_module(native)
import images


def write(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf8", newline="\n")


def prepare(cache):
    normalized = []
    for label, offset in (("first",19), ("return",0)):
        feed = json.loads((HERE/(label+"-feed.json")).read_bytes())
        for row in feed["cases"]:
            if row["name"].endswith("displaced-source"):
                continue
            raw = cache/(row["name"]+".npy")
            if native.sha(raw) != row["trajectorySHA256"]:
                raise ValueError("Changed receiver trajectory")
            tr = np.load(raw)
            tr[:,0] += offset   # Station envelope checker uses time 119 ms as the instant of lip departure.
            alias = "probe-bias-"+label+"-"+str(round(row["dt"]*1e6))
            dest = cache/(alias+".npy")
            np.save(dest,tr)
            normalized.append(dict(name=alias, actualReturn=row["actualReturn"], loss=row["loss"], hardStop=row["hardStop"],
                                   trajectorySHA256=native.sha(dest)))
    for kind in ("probe","hold"):
        write(cache/(kind+"-feed.json"), normalized)


def station_native(cache,lamp):
    prepare(cache)
    native.run(cache,lamp,relief=True,probe=True,records=cache)
    native.run(cache,lamp,relief=True,probe=True,hold=True,records=cache)
    names=["probe-native","hold-native"]
    bins=json.loads((cache/"probe-native.json").read_bytes())["bins"]
    if any(b["outerEnvelopeOverlapMm3"] for b in bins):
        raise ValueError("First or second arrival needs an individual rigid-material analysis")
    for name in names:
        shutil.copyfile(cache/(name+".json"),HERE/(name+".json"))


def station_images(cache, prior, field):
    # Image time remains in each source's actual camera epoch. Do not normalize the electronic clock.
    if native.sha(prior/"station-views-relief.npz") != json.loads((ROOT/"return-station/relief-dock.json").read_bytes())["stowedCameraBackgroundSHA256"]:
        raise ValueError("Incorrect station robot")
    shutil.copyfile(prior/"station-views-relief.npz",cache/"station-views-relief.npz")
    imgdir=cache/"images"
    imgdir.mkdir(exist_ok=True)
    feeds = [json.loads((HERE/(label+"-feed.json")).read_bytes())["cases"][0] for label in ("first","return")]
    write(imgdir/"hold-feed.json",feeds)
    images.run(cache,field,probe=True,hold=True,records=imgdir)
    shutil.copyfile(imgdir/"hold-images.json",HERE/"station-images.json")


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation",choices=["native","images"])
    parser.add_argument("cache",type=Path)
    parser.add_argument("source",type=Path)
    parser.add_argument("--prior",type=Path)
    a=parser.parse_args()
    if a.operation=="native":
        station_native(a.cache,a.source)
    else:
        station_images(a.cache,a.prior,a.source)
