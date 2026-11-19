"""Join the actual installed two-damper shelf bill to the capped, short-shaft mast build."""
import argparse
import subprocess
import sys
from pathlib import Path
from fit import HERE, ROOT, j


def run(plugin, map_path, shelf, work):
    work.mkdir(parents=True,exist_ok=True)
    selected = j.read(map_path)
    if selected["selected"][-2:]!=["mast-foot-splices","fixed-secondary-haul"]:
        raise ValueError("Expected both selected mechanical changes")
    temp = work/"shelf-comparison.json"
    subprocess.run([sys.executable,str(shelf/"mass.py"),str(plugin),str(map_path),
                    str(work),str(temp),"--installed"],check=True)
    data = j.read(temp)
    face = j.read(ROOT/"station-guide/mass.json")["hypotheticalLinerDensityKgM3"]["500"]["guideFaceKg"]
    if data["guideUpperExampleKg"]!=face:
        raise ValueError("Guide face basis changed")
    cases = {}
    for key in ("densityMatchedRemovalKg","noCutCreditKg"):
        row = data["installedChecker"][key]
        if any(row["coverage"].values()) or row["physicalMarginKg"] is not None:
            raise ValueError("Coverage or mass basis changed")
        margin = row["representedMarginKg"][0]
        cases[key] = {"shelfAndMastUpperMarginKg":margin,"exampleFaceOnlyUpperMarginKg":margin-face,
                      "installedShelfDeltaKg":row["deltaKg"]}
    j.write(HERE/"combined.json",{"plugin":data["plugin"],"selectedChanges":data["sourceSelection"],
        "shelfBuildSha256":j.digest(shelf/"installed.json"),"shelfAccountingSha256":j.digest(shelf/"mass.py"),
        "mastMapWithHoistSha256":j.digest(map_path),"hypotheticalGuideFaceKg":face,"cases":cases,
        "guideMountMassKg":None,"physicalInspectionMarginKg":None})


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("plugin","map","shelf","work"):
        parser.add_argument(name,type=lambda s:Path(s).resolve())
    args=parser.parse_args()
    run(args.plugin,args.map,args.shelf,args.work)
