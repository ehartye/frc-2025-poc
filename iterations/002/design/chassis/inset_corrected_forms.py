"""Calculate source geometry for the seven corrected shop forms without regenerating other stock."""
from hashlib import sha256
from pathlib import Path
import json,sys
from inset_stock_check import HERE,form

def run(plugin):
    sys.path.insert(0,str(plugin))
    from fusion_authority import register_path,verify
    from mechanical_motion import bounds
    from component_shape import volume
    reg=register_path(HERE.parents[3]);authority=verify(HERE.parents[3],reg,reg.parent/"issue-006/lineage.json")
    src=HERE/"inset-corrected-brace-foot-clearance.json";d=json.loads(src.read_bytes())
    ancestor=json.loads((HERE/"connected-native-03-cold.json").read_bytes())
    placed=json.loads((HERE/"connected-654-exchange.json").read_bytes())
    if authority["archiveSha256"]!=placed["native"] or placed["held"] or len(placed["pairs"])!=654 or ancestor["authority"]!=d["authority"] or len(d["stockChanges"])!=7:
        raise ValueError("Unexpected shop revision or changed placed parent")
    out={}
    for name,row in d["stockChanges"].items():
        skipped=[];body=form(row,skipped)
        if "matrixMm" in row:raise ValueError("Uncommissioned local-to-world transform")
        out[name]=dict(bounds=bounds(body),volumeMm3=volume(body),alreadyRemovedCutIndices=skipped)
    return dict(authority=authority,forms=out,sourceSHA256=sha256(src.read_bytes()).hexdigest(),parentColdExchange=placed["export"])

if __name__=="__main__":
    d=run(Path(sys.argv[1]));(HERE/"inset-corrected-seven-shapes.json").write_text(json.dumps(d,indent=1)+"\n")
