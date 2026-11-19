"""Render the joined guide and pipe exposure states on the two registered cameras."""
import argparse
import json
from pathlib import Path
import sys
from field_view import HERE, ROOT, assembled_mesh, digest


def run(assets,field,packet_path,expected,folder):
    sys.path.insert(0,str(ROOT/"mounted-guide-vision"))
    import station
    if digest(packet_path)!=expected:
        raise ValueError("Exposure input no longer matches its simulation receipt")
    if any(row["outwardPerSideMm"]!=0 for row in station.read(packet_path)["packets"]):
        raise ValueError("This selection models the nominal left and right guide locations")
    def build(installation,setting):
        return assembled_mesh(station,folder,installation,setting)
    result=station.run(assets,field,ROOT/"guide-setting",packet_path,mesh_builder=build,
        output=folder/"station-views",installations={"nominal":0.},packet_sha=expected,
        packet_label="guide-arm-route/mechanics/station-frames.json")
    record=result["set-nominal"]
    record["register"]["returnedArmStepSha256"]=digest(folder/"route.step")
    (folder/"station.json").write_text(json.dumps(record,indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("assets","field_step","packets","folder"):
        p.add_argument(name,type=lambda v:Path(v).resolve())
    p.add_argument("packet_sha256")
    a=p.parse_args()
    run(a.assets,a.field_step,a.packets,a.packet_sha256,a.folder)
