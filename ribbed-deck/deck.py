"""Machined deck candidate. Dimensions are measured from the current stowed robot."""
import argparse
from pathlib import Path
import sys

from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "drive-side-attachment"))
import joint as shop

TOP = 154.
SKIN = 1.7
RIB_WIDTH = 3.
STATIONS = tuple(range(-300,301,50))
# Total section depths at positive Y. Run each rib into the original coplanar
# rail land; a rising terminal ramp gives the end screws a clear envelope.
PROFILE = ((0,18), (72,17), (150,13), (200,9), (220,6), (226,4))
AXES = ((-339,-225), (-339,72), (-339,225), (339,-225), (339,-72), (339,225))
PART = "integral-rib-deck"

def unify(shape):
    result = ShapeUpgrade_UnifySameDomain(shape, True, True, True)
    result.Build()
    return result.Shape()

def bearing_pads(scene):
    pads = {}
    for name, (a, _) in scene.items():
        x,y,z,xx,yy,_ = a
        if abs(z-TOP) < .01 and x < 346 and xx > -346 and y < 258 and yy > -258:
            pads[name] = [max(-346,x-4),min(346,xx+4),max(-258,y-4),min(258,yy+4)]
    return pads

def native(scene):
    deck = shop.box(-346,346,-258,258,TOP-SKIN,TOP)
    pads = bearing_pads(scene)
    lands = list(pads.values()) + [
        [-346,-321,-258,258], [321,346,-258,258],
        [-346,346,-258,-212], [-346,346,212,258],
    ]
    for x,xx,y,yy in lands:
        deck = shop.fuse(deck,shop.box(x,xx,y,yy,150,TOP-SKIN))
    ends = [(-y,d) for y,d in reversed(PROFILE[1:])] + list(PROFILE)
    for x in STATIONS:
        side = x-RIB_WIDTH/2
        section = [(side,-226,TOP-SKIN),(side,226,TOP-SKIN)]
        section.extend((side,y,TOP-d) for y,d in reversed(ends))
        deck = shop.fuse(deck,shop.extrude(section,(RIB_WIDTH,0,0)))
    deck = unify(deck)
    before = shop.cad.volume(deck)[0]
    for x,y in AXES:
        deck = shop.cut(deck,shop.cad.brep(["cyl",[x,y,149,0,0,1,2.75,6]]))
    return unify(deck),pads,before

def run(registry,out):
    scene=shop.read(registry)
    if scene["deck-main"][0]!=[-346.,-258.,150.,346.,258.,154.]:
        raise ValueError("Deck datums changed")
    deck,pads,before=native(scene)
    if not BRepCheck_Analyzer(deck).IsValid():
        raise ValueError("Invalid deck solid")
    out.mkdir(parents=True,exist_ok=True)
    step=out/"ribbed-deck.step"
    shop.write_step(step,{PART:deck})
    shop.finalize_step(step)
    opened=shop.cad.read_named(step,[{"id":PART}])[PART]
    exp=TopExp_Explorer(opened,TopAbs_SOLID)
    count=0
    while exp.More():
        count+=1
        exp.Next()
    volume=shop.cad.volume(opened)[0]
    if count!=1 or abs(volume-shop.cad.volume(deck)[0])>.002:
        raise ValueError("Exchange no longer preserves one deck solid")
    if abs(before-volume-6*3.141592653589793*2.75**2*4)>.002:
        raise ValueError("Deck tie holes did not pass through exactly 4 mm of stock")
    full=692*516*4
    receipt={"stepSha256":shop.digest(step),"registrySha256":shop.digest(registry),
        "solidCount":count,"stockVolumeMm3":full,"solidVolumeMm3":volume,
        "savedVolumeBeforeTieDrillingMm3":full-before,"tieBoreVolumeMm3":before-volume,
        "candidateMassKgAt2700":volume*2.7e-6,"nominalRecoveryWithoutTieChipsKg":(full-before)*2.7e-6,
        "upperBearingLands":pads,"sectionDepthFromTopMm":PROFILE,"webMm":RIB_WIDTH,
        "stationsMm":STATIONS,"skinMm":SKIN,"physicalMarginKg":None}
    shop.write(out/"geometry.json",receipt)
    print(receipt["nominalRecoveryWithoutTieChipsKg"],volume,count)

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("registry",type=Path)
    parser.add_argument("--out",type=Path,default=HERE)
    args=parser.parse_args()
    run(args.registry,args.out)
