"""Conservatively reserve the old-face translation for four full mounted supplier cassettes."""
from pathlib import Path
import json,re,sys,hashlib
import numpy as np
from four_corner_packaging import outline,rotated
from inset_electrical_package import bb,box_corners,gap

HERE=Path(__file__).resolve().parent


def run(pkg):
    sys.path.insert(0,str(pkg/"scripts"))
    from mechanical_motion import bounds
    from showcase_contacts import load_solids
    from fusion_authority import register_path,verify
    root=HERE.parents[3];reg=register_path(root);line=reg.parent/"issue-005/lineage.json"
    authority=verify(root,reg,line)
    file=HERE/"inset-cassette-decks.json";m=json.loads(file.read_bytes())
    if authority!=m["authority"]:raise ValueError("Nonmatching issued source")
    driving=[q for n,q in m["outcomes"].items() if "keyed" in n]
    largest=max(v["maxTransverseMountTravelMm"] for q in driving for v in q["cases"].values())
    travel=np.ceil(largest*10)/10
    parts,sheets=load_solids(line.with_name("assembly.step"))
    groups={1:(1,1),4:(1,-1),6:(-1,-1),8:(-1,1)}
    interior=np.array(outline(370,365,42,38.1))
    devices=json.loads((HERE/"inset-electrical-package.json").read_bytes())["candidatePlacements"]
    fixed={k:d["boundsMm"] for k,d in devices.items()}
    from inset_electronics_deck import definition
    for s,p in definition(True).items():
        x0,y0,x1,y1=p["limits"];ya,yb=sorted((s*y0,s*y1))
        fixed["finite deck blank "+str(s)]=[x0,ya,203.2,x1,yb,205.3844]
        fixed["sill "+str(s)]=[-311.55,s*115.3-12.7,111.125,311.55,s*115.3+12.7,161.925]
    for x in (110,250):fixed["upper crossmember "+str(x)]=[x-25.4,-242,161.925,x+25.4,242,187.325]
    rows={};near=[]
    for group,(sx,sy) in groups.items():
        subset={k:rotated(v,0,(-12.7*sx,-12.7*sy,0)) for k,v in parts.items() if re.match(r".*\[1\."+str(group)+r"\.1\.",k)}
        if len(subset)!=121 or sheets:raise ValueError("Incomplete source corner")
        hard=[];frame=[]
        for k,g in subset.items():
            b=bounds(g);e=np.array(b)+[-travel,-travel,0,travel,travel,0]
            for n,v in fixed.items():
                d=gap(e,v);hard.append((d,k,n))
                if d<.05:near.append(dict(corner=group,row=k,peer=n,sourceBoundsMm=b))
            # Exact distance from a rectangular enclosure to each inward half-space.
            # A positive reserve allows any XY vector of length <= travel through
            # the closed field interval, including simultaneous X/Y changes.
            if b[5]<101.6 or b[2]>203.2:continue
            corners=box_corners(b)[:,:2]
            for j,(a,c) in enumerate(zip(interior,np.roll(interior,-1,axis=0))):
                d=c-a;normal=np.array([-d[1],d[0]])/np.linalg.norm(d)
                if np.dot(-a,normal)<0:normal=-normal
                bound=float(np.min((corners-a)@normal)-travel);frame.append((bound,k,j))
        rows[str(group)]=dict(sourceBodies=len(subset),minimumMovingToDevicesAndSillsMm=min(hard),minimumToFramePolygonMm=min(frame),
            unresolvedFrameBounds=[dict(gapMm=g,id=k,face=j) for g,k,j in sorted(frame) if g<0])
    return dict(authority=authority,sourceResponseSha256=hashlib.sha256(file.read_bytes()).hexdigest(),
        recordedOldFaceTravelMm=largest,physicalXYEnvelopeMm=travel,corners=rows,unresolvedOtherBounds=near,
        scope="Full parked original four supplier 121-body groups shifted to the inset axes. The ring uses its six true internal half-spaces and actual vertical slab, the decks are bounded with stock covering every service opening. Positive separation covers the complete planar translation bound. Unresolved enclosures require narrower BRep checks; chassis twist, local yaw, vertical shock, the corrected motor/axle, retaining hardware and actual cable curves need their own envelopes.")


if __name__=="__main__":
    r=run(Path(sys.argv[1]));(HERE/"inset-cassette-travel.json").write_text(json.dumps(r,indent=2)+"\n")
    for k,v in r["corners"].items():print(k,v["minimumMovingToDevicesAndSillsMm"],v["minimumToFramePolygonMm"],len(v["unresolvedFrameBounds"]))
    print("other unresolved",len(r["unresolvedOtherBounds"]))
