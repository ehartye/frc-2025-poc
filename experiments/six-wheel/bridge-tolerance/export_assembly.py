"""Export the 593-solid aft-camera robot from original project and owner-supplied geometry."""
import argparse
import json
import sys
from pathlib import Path
from tempfile import TemporaryDirectory
from design import HERE,ROOT,mount,parts
from replay_candidate import mod,sha

def export(lampfile,output,pose):
    sys.path.insert(0,str(ROOT/"design/fixed-no-air"))
    import whole
    original=mod("no_air_export",ROOT/"design/fixed-no-air/export_robot.py")
    cad=mount.cad
    source=json.loads((ROOT/"design/fixed-no-air/whole-robot.json").read_bytes())
    if sha(lampfile)!=source["supplied_lamp_sha256"]:raise ValueError("Unregistered light")
    names=[*parts()[0],"receiver-R6-drilled-for-camera"]
    roof=cad.read_named(HERE/"moving-bridge.step",[{"id":k} for k in names])
    deck=cad.read_named(whole.LIGHT/"deck-mount.step",[{"id":k} for k in ("front-electronics-platform","light-spacer")])
    row=source["cases"][pose]
    _,shapes=whole.model(row["stroke_mm"],roof,deck,whole.mounted(whole.product(lampfile),whole.X,whole.Y,whole.PANEL))
    changed={k for k in shapes if k.startswith("roof-") or k=="receiver-R6-drilled-for-camera"}
    expected={**row["identical_brep_sha256"],**row["new_or_changed"]}
    if set(expected)!=set(shapes) or len(changed)!=22:raise ValueError("Changed inventory")
    for k in shapes.keys()-changed:
        if whole.fingerprint(shapes[k])!=expected[k]:raise ValueError("Unexpected change to "+k)
    output.parent.mkdir(parents=True,exist_ok=True)
    with TemporaryDirectory(dir=output.parent) as scratch:
        candidate=Path(scratch)/output.name
        original.write_assembly(candidate,shapes)
        receipt=original.verify(candidate,shapes,row,pose)
        candidate.replace(output)
    receipt.update(aft_bridge_sha256=sha(HERE/"moving-bridge.step"),unchanged_native_bodies=len(shapes)-len(changed),
                   changed_native_bodies=sorted(changed))
    output.with_suffix(".json").write_text(json.dumps(receipt,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--lamp",required=True,type=Path)
    p.add_argument("--output",required=True,type=Path)
    p.add_argument("--pose",choices=("receive","transport","place"),default="place")
    a=p.parse_args();export(a.lamp,a.output,a.pose)
