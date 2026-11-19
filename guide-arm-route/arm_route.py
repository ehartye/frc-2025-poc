"""Route the lower-left arm from its registered pin and screw joint to a new guide height."""
import argparse
import sys
from pathlib import Path

from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepCheck import BRepCheck_Analyzer

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "guide-setting"))
from setting_geometry import carriage, j
from parts import center_x, face, surface_y, ycone

ARM = "guide-L-687-mount-arm"
BACK = "guide-L-backing"
NOSE = ["guide-L-687-" + key for key in ("wedge", "tapped-bar", "face-screw-0", "face-screw-1")]
Z = 687.
Y = surface_y(Z) + 8.15


def inputs():
    kit = carriage.selected(ROOT/"guide-mounts", "guide-mounts.step", "fit.json", "stepSha256", "volumesMm3")
    pins = carriage.selected(ROOT/"guide-setting", "setting.step", "fit.json", "stepSha256", "volumesMm3")
    for key in j.read(ROOT/"guide-setting/fit.json")["changedInstalled"]:
        kit[key] = pins[key]
    return kit


def clip(body, low, high):
    box = j.box(low, high, 100, 260, 600, 850)
    return BRepAlgoAPI_Common(body, box).Shape()


def span(start, end, z, rise):
    x = start
    def prism(inset):
        y1, y2 = Y+inset, Y+30-inset
        z1, z2 = z-6+inset, z+6-inset
        return j.extrude([(x,y1,z1),(x,y2,z1),(x,y2,z2),(x,y1,z2)],(end-start,0,rise))
    return j.cut(prism(0.), prism(.8))


def backing(z):
    blank = face(1, False)
    for station in (z, 787.):
        y = surface_y(station)
        for offset in (-8.,8.):
            x = center_x(station)+offset
            seat = y+3+.18
            counter = j.fuse(j.rod(x,1,seat-1,seat,station,3), ycone(x,1,seat,station,3,1.5,1.5))
            blank = j.cut(j.cut(blank, counter), j.rod(x,1,y+2,surface_y(station+6)+18.65,station,1.65))
    return blank


def revision(kit, rise):
    nose_start = center_x(Z)-17
    dx = .46*rise
    shift = (dx,0,rise)
    root = clip(kit[ARM], -45, 4)
    angled = span(4,75,Z,rise)
    beam = span(75,nose_start+dx,Z+rise,0)
    nose = j.cad.translated(clip(kit[ARM],nose_start,230),shift)
    arm = j.fuse(j.fuse(j.fuse(root,angled),beam),nose)
    updated = {ARM:arm,BACK:backing(Z+rise)}
    updated.update({k:j.cad.translated(kit[k],shift) for k in NOSE})
    return updated


def exchange(kit, layout, prefix):
    if layout.startswith("rise-"):
        rise=int(layout.removeprefix("rise-"))
        bodies=revision(kit,rise)
    else:
        import planform, return_arm
        rise=None
        bodies=return_arm.build(kit,layout) if layout in return_arm.RETURNS else planform.build(kit,layout)
    out = prefix / layout
    out.mkdir(parents=True,exist_ok=True)
    path = out/"route.step"
    j.write_step(path, bodies)
    j.finalize_step(path)
    measured = j.cad.read_named(path,[{"id":k} for k in bodies])
    volume = {}
    for k,v in measured.items():
        if j.cad.solid_count(v)!=1 or not BRepCheck_Analyzer(v).IsValid():
            raise ValueError("Invalid exchanged solid " + k)
        volume[k] = j.cad.volume(v)[0]
        if abs(volume[k] - j.cad.volume(bodies[k])[0]) > .003:
            raise ValueError("Body volume changed on reopen: " + k)
    if rise==0:
        for k,v in measured.items():
            difference = volume[k]+j.cad.volume(kit[k])[0]-2*j.cad.intersection(v,kit[k])
            if abs(difference) > .01:
                raise ValueError((k,"zero-rise reconstruction", difference))
    j.write(out/"revision.json",{"layout":layout,"riseMm":rise,"volumesMm3":volume,"settingSha256":j.digest(ROOT/"guide-setting/setting.step"),
        "guideSha256":j.digest(ROOT/"guide-mounts/guide-mounts.step"),"stepSha256":j.digest(path),
        "armVolumeChangeMm3":volume[ARM]-j.cad.volume(kit[ARM])[0]})


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("output",type=Path)
    p.add_argument("layouts",nargs="+")
    args=p.parse_args()
    source=inputs()
    for layout in args.layouts:
        exchange(source,layout,args.output)
        print("Reopened",layout,flush=True)
