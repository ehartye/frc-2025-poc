"""Regenerate the four complete robot poses with the revised drive architecture."""
import argparse
import json
from pathlib import Path
from drive_parts import parts
from build_assembly import build,MODEL

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("output",type=Path)
    a=p.parse_args()
    result=build(a.output,MODEL["poses"],parts())
    print(json.dumps({k:[v["shapeCount"],v["interferencesOver0.001mm3"]] for k,v in result["poses"].items()}))
