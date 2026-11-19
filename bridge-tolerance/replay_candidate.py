"""Render both cameras on the 593-body candidate and replay the source trajectories and packet harness."""
import argparse
import csv
import hashlib
import importlib.util
import json
import shutil
import sys
import tempfile
from pathlib import Path
from unittest.mock import patch
import numpy as np
from design import HERE,ROOT,SHIFT,mount,optics,parts
import bridge_fit

def mod(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def model(lampfile):
    sys.path.insert(0,str(ROOT/"design/fixed-no-air"))
    import whole
    m=mount.cad
    roof=m.read_named(HERE/"moving-bridge.step",[{"id":k} for k in [*parts()[0],"receiver-R6-drilled-for-camera"]])
    deck=m.read_named(whole.LIGHT/"deck-mount.step",[{"id":k} for k in ("front-electronics-platform","light-spacer")])
    lamp=whole.mounted(whole.product(lampfile),whole.X,whole.Y,whole.PANEL)
    _,combined=whole.model(370,roof,deck,lamp)
    if len(combined)!=593:raise ValueError("unexpected whole robot")
    return combined

def run(cache,lamp,jdk,gradle):
    cache=cache.resolve()
    hw=parts()
    optics.ORIGIN=np.array([98+SHIFT,0.,842.])
    prior=ROOT/"moving-observer"
    # Pin the exact private payload prior to reading the existing modeled trajectories.
    move=mod("bridge_traj",prior/"replay.py")
    link=mod("bridge_join",prior/"handoff.py")
    raw=mod("bridge_raw",ROOT/"moving-frame-timing/exposures.py")
    packet=mod("bridge_packet",ROOT/"moving-frame-timing/verify.py")
    travel=mod("bridge_travel",prior/"verify.py")
    from imaging import images_for_pose
    full=model(lamp)
    _,groups,colors,_=mount.pose_layers(560,370,"red")
    mesh={"parts":[]}
    for key,shape in sorted(full.items()):
        v,t=mount.original.triangulate(shape)
        mesh["parts"].append([key,groups.get(key,"Mount"),colors.get(key,"#737d85"),v,t])
    # Render fixed view before switching the renderer to its vertical moving head.
    fixed=images_for_pose(mesh)
    optics.configure()
    with patch.object(mount,"parts",return_value=hw):
        moving=images_for_pose(optics.moving_mesh())
    scene=cache/("moving-custody-"+sha(HERE/"moving-bridge.step")[:16]+".npz")
    np.savez_compressed(scene,left=moving[0][0],left_z=moving[0][1],right=moving[1][0],right_z=moving[1][1])
    # Source adapters only redirect output, selected native views and the aft head's image estimator.
    with patch.object(optics,"fit",bridge_fit.fit),patch.object(move,"HERE",HERE):
        move.run(cache)
    # Reuse the issued numerical exposure extractor with its registrations pointed to the
    # candidate. The historical source and its checked-in receipts stay independently replayable.
    import imaging
    import stereo
    imaging.ORIGIN=np.array(stereo.ORIGIN)
    imaging.C,imaging.S=stereo.C,stereo.S
    imaging.DX,imaging.DY,imaging.DZ=imaging.C+imaging.Q*imaging.S,-(imaging.U-imaging.W/2)/imaging.F,-imaging.S+imaging.Q*imaging.C
    with tempfile.TemporaryDirectory() as projected:
        root=Path(projected)
        for group,names in (("low-observation",("results.json",)),("moving-observer",("moving-bridge.step","replay.json"))):
            folder=root/group
            folder.mkdir()
            for name in names:
                shutil.copyfile((ROOT/group if group=="low-observation" else HERE)/name,folder/name)
        with patch.object(raw,"ROOT",root),patch.object(raw,"HERE",HERE),patch.object(raw,"optics",link),patch.object(link,"register",return_value=mesh),patch.object(imaging,"images_for_pose",return_value=fixed),patch.object(optics,"fit",bridge_fit.fit):
            raw.build(cache)
    source=ROOT/"moving-frame-timing"
    with tempfile.TemporaryDirectory() as tmpdir:
        temp=Path(tmpdir)
        for root,names in ((source,("OpticalWindow.java","PacketClockTest.java","exposures.py","verify.py")),(prior,("MovingWitnessTest.java",)),(HERE,("frames.csv","replay.json","exposures.csv"))):
            for name in names:shutil.copyfile(root/name,temp/name)
        with patch.object(travel,"HERE",temp):travel.run(jdk,gradle)
        shutil.copyfile(temp/"hal.json",HERE/"travel-hal.json")
        moving_id=sha(scene)
        fixed_id=hashlib.sha256((sha(ROOT/"design/fixed-no-air/whole-robot.json")+sha(HERE/"moving-bridge.step")+sha(HERE/"replay_candidate.py")+sha(HERE/"bridge_fit.py")).encode()).hexdigest()
        def contexts():
            with (source/"contexts.csv").open(newline="") as inp,(temp/"contexts.csv").open("w",newline="") as out:
                reader=csv.DictReader(inp);writer=csv.DictWriter(out,fieldnames=reader.fieldnames,lineterminator="\n");writer.writeheader()
                for row in reader:
                    row.update(moving_register=moving_id,fixed_register=fixed_id);writer.writerow(row)
        with patch.object(packet,"HERE",temp),patch.object(packet,"contexts",contexts):
            packet.run(jdk,gradle,cache/"aft-bridge-packet-sweep.csv")
        summary=json.loads((temp/"results.json").read_bytes())
        shutil.copyfile(temp/"contexts.csv",HERE/"contexts.csv")
        (HERE/"timing.json").write_text(json.dumps(summary,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for n in ("scratch","lamp","jdk","gradle"):p.add_argument("--"+n,required=True,type=Path)
    a=p.parse_args();run(a.scratch,a.lamp,a.jdk,a.gradle)
