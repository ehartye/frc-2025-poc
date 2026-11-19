"""Reposition the registered rigid bridge on a newly matched shelf. Robot coordinates are millimetres."""
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"moving-observer"))
import mount
import optics

SHIFT=-50.0

def parts(shift=SHIFT):
    kit,materials,holes=mount.parts()
    return ({k:mount.cad.translated(s,(shift,0,0)) for k,s in kit.items()},materials,
            [[kind,[x+shift,*other]] for kind,(x,*other) in holes])


def evaluate(shift):
    from unittest.mock import patch
    kit,materials,bores=parts(shift)
    with patch.object(mount,"parts",return_value=(kit,materials,bores)),patch.object(mount,"HERE",HERE):
        mount.evaluate(None)

if __name__=="__main__":
    evaluate(SHIFT)
