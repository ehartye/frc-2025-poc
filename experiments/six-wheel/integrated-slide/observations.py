"""Register field-backed stereo observations and run the released 28 packet policies."""
import argparse
from pathlib import Path
import sys
from types import SimpleNamespace

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"selected-optics"))


def run(args):
    out=args.output.resolve()
    frames=out/"marked"
    if args.stage=="field":
        import field_roof
        field_roof.run(SimpleNamespace(scenes=frames,cache=args.trajectories,field=args.field),out)
    else:
        import controller
        controller.run(SimpleNamespace(images=frames,cache=args.trajectories,out=out/"observer-hal",
                                       jdk=args.jdk,gradle=args.gradle),out)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("stage",choices=["field","packets"])
    for name in ("output","trajectories","field","jdk","gradle"):
        p.add_argument(name,type=Path)
    run(p.parse_args())
