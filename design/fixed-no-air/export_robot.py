"""Export a private whole-robot STEP and reopen every named solid against the registered pose."""
import argparse
import json
from pathlib import Path
from tempfile import TemporaryDirectory

from service import HERE,LIGHT,ROOT,PANEL,X,Y,cad,common,digest,finalize_step,mounted,product,read
from whole import fingerprint,model
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer


def write_assembly(path, shapes):
    doc,table=cad.xcaf_doc()
    for name,shape in sorted(shapes.items()):
        label=table.AddShape(shape,False)
        cad.TDataStd_Name.Set_s(label,cad.TCollection_ExtendedString(name))
    writer=cad.STEPCAFControl_Writer()
    if not writer.Transfer(doc,cad.STEPControl_AsIs) or writer.Write(str(path))!=cad.IFSelect_RetDone:
        raise ValueError("Could not write named robot assembly")
    finalize_step(path)


def root_solids(path):
    reader=STEPControl_Reader()
    if reader.ReadFile(str(path))!=cad.IFSelect_RetDone or not reader.TransferRoots():
        raise ValueError("Could not reopen STEP roots")
    it=TopExp_Explorer(reader.OneShape(),TopAbs_SOLID)
    volumes=[]
    while it.More():
        volumes.append(cad.volume(it.Current())[0])
        it.Next()
    return sorted(volumes)


def export(source, output, pose):
    manifest=read(HERE/"whole-robot.json")
    for name,expected in manifest["source_sha256"].items():
        if digest(ROOT/name)!=expected:
            raise ValueError("Source changed: "+name)
    row=manifest["cases"][pose]
    observer=ROOT/"moving-observer"
    roof=cad.read_named(observer/"moving-bridge.step",[{"id":n} for n in
                        [*read(observer/"mount.json")["parts"],"receiver-R6-drilled-for-camera"]])
    deck=cad.read_named(LIGHT/"deck-mount.step",[{"id":n} for n in ("front-electronics-platform","light-spacer")])
    _,shapes=model(row["stroke_mm"],roof,deck,mounted(product(source),X,Y,PANEL))
    expected={**row["identical_brep_sha256"],**row["new_or_changed"]}
    if {k:fingerprint(s) for k,s in shapes.items()}!=expected:
        raise ValueError("Assembled pose differs from its native register")
    output.parent.mkdir(parents=True,exist_ok=True)
    with TemporaryDirectory(dir=output.parent) as scratch:
        candidate=Path(scratch)/output.name
        write_assembly(candidate,shapes)
        receipt=verify(candidate,shapes,row,pose)
        candidate.replace(output)
    output.with_suffix(".json").write_bytes((json.dumps(receipt,indent=2)+"\n").encode())


def verify(output,shapes,row,pose):
    reopened=cad.read_named(output,[{"id":k} for k in shapes])
    volume_errors, bound_errors, center_errors, material_errors = [],[],[],{}
    for k,shape in shapes.items():
        other=reopened[k]
        v,center=cad.volume(shape)
        w,center2=cad.volume(other)
        volume_errors.append(abs(v-w))
        bound_errors.extend(abs(a-b) for a,b in zip(cad.bounds(shape),cad.bounds(other)))
        center_errors.extend(abs(a-b) for a,b in zip(center,center2))
        if k in row["new_or_changed"] or k.startswith("roof-") or k=="receiver-R6-drilled-for-camera":
            material_errors[k]=abs(v+w-2*common(shape,other))
    raw=root_solids(output)
    expected=sorted(cad.volume(s)[0] for s in shapes.values())
    if len(raw)!=len(expected):
        raise ValueError("Independent STEP reader changed the number of solids")
    raw_error=max(abs(a-b) for a,b in zip(raw,expected))
    errors=max(volume_errors),max(bound_errors),max(center_errors),max(material_errors.values()),raw_error
    if any(e>t for e,t in zip(errors,(.01,.001,.001,.02,.01))):
        raise ValueError(f"STEP round trip changed registered material: {errors}")
    return {"pose":pose,"native_manifest_sha256":digest(HERE/"whole-robot.json"),"step_sha256":digest(output),
             "independent_root_solids":len(raw),"max_independent_volume_error_mm3":raw_error,
             "reopened_named_solids":len(reopened),"max_volume_error_mm3":max(volume_errors),
             "max_bounds_error_mm":max(bound_errors),"max_centroid_error_mm":max(center_errors),
             "changed_or_optical_material_difference_mm3":material_errors}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("signal_light_step",type=Path)
    p.add_argument("output_step",type=Path)
    p.add_argument("--pose",choices=("receive","transport","place"),default="place")
    a=p.parse_args()
    export(a.signal_light_step,a.output_step,a.pose)
