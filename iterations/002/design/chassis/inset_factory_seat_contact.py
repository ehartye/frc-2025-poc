"""Check finite opposed keeper lands against the true module plate edges without changing the purchased shape."""
from pathlib import Path
import hashlib,json,re,sys
from crossmember_envelope import box
from four_corner_packaging import rotated
from structural_perimeter import relocate_lower
HERE=Path(__file__).resolve().parent


def run(plugin):
    sys.path.insert(0,str(plugin/"scripts"))
    from fusion_authority import register_path,verify
    from mechanical_motion import bounds,distance,common
    from showcase_contacts import load_solids,box_gap
    root=HERE.parents[3];reg=register_path(root);line=reg.parent/"issue-005/lineage.json"
    authority=verify(root,reg,line);stock,sheets=load_solids(line.with_name("assembly.step"))
    if sheets or len(stock)!=552:raise ValueError("Wrong actual module")
    defs=json.loads((HERE/"socket-holes.json").read_bytes())["rails"]
    stock=relocate_lower({k:v for k,v in stock.items() if int(re.search(r"\[1\.(\d+)\.",k)[1])<=8},defs,12.7)
    ext={k:bounds(v) for k,v in stock.items()}
    out=[]
    for z,l,h,selector in (("lower",36.3,43.40,"[1.1.1.21.16]"),("upper",95.03,98.98,"[1.1.1.21.17]")):
        plate,= [name for name in stock if selector in name]
        for direction,y in ((-1,305.575),(1,330.975)):
            inner,outer=sorted([y,y+direction*3.175])
            land=box((170.9,inner,l),(223.,outer,h));bb=bounds(land)
            neighbors={}
            for k,v in stock.items():
                if box_gap(bb,ext[k])>.15:continue
                gap=distance(land,v);vol=common(land,v) if gap<.001 else 0.
                neighbors[k]=dict(gapMm=gap,commonMm3=vol)
            disp={}
            for d in (-.1,-.05,-.025,.05):
                translated=rotated(land,0,(0,d*direction,0))
                disp[str(d)]=common(translated,stock[plate])
            out.append(dict(level=z,side="outboard" if direction==1 else "inboard",plate=plate,
                registerMm=[170.9,223.,inner,outer,l,h],neighbors=neighbors,
                signedOutwardDisplacementAndCommonMm3=disp,
                areaProxyMm2=(disp["-0.025"]-neighbors.get(plate,{"commonMm3":0.})["commonMm3"])/.025))
    return dict(authority=authority,rows=out,scope="Flat positive keeper bearing research on the original seated 7075 SDS plate perimeter, both faces of the selected actual socket, both signs. The inward displacement between the limited rectangular test land and the real article measures the locally available loaded area. A loose pad is not a bracket; the keeper needs a positive connection to the opposite socket wall, and actual tolerance, screw clamp, out-of-plane bending and tool/removal access remain open.")


if __name__=="__main__":
    d=run(Path(sys.argv[1]));(HERE/"inset-factory-seat-contact.json").write_text(json.dumps(d,indent=2)+"\n")
    for r in d["rows"]:print(r["level"],r["side"],r["areaProxyMm2"],r["signedOutwardDisplacementAndCommonMm3"],[(k,v["commonMm3"]) for k,v in r["neighbors"].items() if v["commonMm3"]>.02])

