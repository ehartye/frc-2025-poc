"""Test the actual end relief against wall reactions and the two-sill gravity-pan force register."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
from inset_key_contact import section

HERE=Path(__file__).resolve().parent


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def main(pkg):
    sys.path.insert(0,str(pkg/"scripts"))
    from fusion_authority import register_path,verify
    root=HERE.parents[3];reg=register_path(root);authority=verify(root,reg,reg.parent/"issue-005/lineage.json")
    load_path=HERE/"inset-key-contact.json";trace=json.loads(load_path.read_bytes())
    geometry_path=HERE/"inset-cassette-narrow.json";geometry=json.loads(geometry_path.read_bytes())
    if trace["authority"]!=authority or geometry["authority"]!=authority:raise ValueError("Stale compared chassis")
    pipe_path=root/"artifacts/red2-integrated/pan-thrust/two-sill-cartridge-selected-duty.json";pipe=json.loads(pipe_path.read_bytes())
    if sha(root/"artifacts/red2-integrated/single-pivot-spatial-rim-installed-reactions-frame-anchors.json")!=pipe["sourceMatched"]:raise ValueError("Lost pan row source")
    sec=section(3.,True);full=section(3.,False)
    def normal(force,m,s):
        return abs(force[1])/s["area"]+abs(m[0])/s["Zvertical"]+abs(m[2])/s["Zhorizontal"]
    wall=[]
    for variant,data in trace["outcomes"].items():
        for case,row in data["cases"].items():
            for end in row.get("relievedSectionEndWrenches",[]):
                f=np.asarray(end["localN_N_Nmm"])
                wall.append(dict(variant=variant,case=case,y=end["y"],sectionMPa=abs(f[0])/sec["area"]+abs(f[2])/sec["Zhorizontal"],forceN=f[:2].tolist(),inPlaneMomentNmm=f[2]))
    pan=[]
    for axis,row in pipe["rows"].items():
        x,y=[float(a) for a in axis.split("/")]
        if x!=250.:continue
        for criterion,ev in row["actualMatchedReactions"].items():
            f=np.asarray(ev["reactionN"])
            for support,height in (("rail face with top reaction already resolved",0.),("unbraced post from upper cartridge",480.-187.325)):
                arm=np.array([0.,np.sign(y)*10.5,height])
                M=np.cross(arm,f)
                pan.append(dict(model=support,at=[x,y],criterion=criterion,case=ev["matchedCase"],timeS=ev["s"],forceN=f.tolist(),
                                momentNmm=M.tolist(),sectionMPa=normal(f,M,sec),torsionAboutRailNm=M[1]/1000))
    travel=max(q["maxTransverseMountTravelMm"] for v in trace["outcomes"].values() for q in v["cases"].values())
    controls=[]
    for label,g in geometry["rows"].items():
        if g["host"]!="transverse x250":continue
        b=g["sourceBoundsMm"];inner=min(abs(b[1]),abs(b[4]))
        controls.append(dict(label=label,distanceBeyondCoreAtTravelMm=inner-travel-231.5,zBelowEndCapMm=174-b[5],
                            oldFullTubeCommonMm3=g["shiftedCommonMm3"],oldEndpointRelievedCommonMm3=g["shiftedRelievedCommonMm3"]))
    screened={k:max((e for e in pan if e["model"]==k),key=lambda e:e["sectionMPa"]) for k in dict.fromkeys(r["model"] for r in pan)}
    return dict(authority=authority,files={p.name:sha(p) for p in (pipe_path,load_path,geometry_path)},wall=wall,
                worstWallSection=max(wall,key=lambda r:r["sectionMPa"]),matchedPan=pan,peakPanBySupport=screened,
                sectionMinimumMm=sec,fullMinimumMm=full,translationControl=dict(travel=travel,rows=controls),
                scope="Actual 10.5 mm end notch lowers the last cut section to a 13.325 mm deep U. In-plane wall and signed factored historic pan station reactions are separate cases. Original pan interfaces are x110/250 y±242 z480. All 002 elements above z187.325 are unissued. The two stated transfer models distinguish a pre-resolved reaction from a vertical cantilever post. Reported elastic normal stress alone suffices to reject an overloaded end; torsion, connection and notch concentration cannot provide additional reserve. Bounding distances apply to pure in-plane translation only.")
if __name__=="__main__":
    result=main(Path(sys.argv[1]));(HERE/"inset-crossbar-pan.json").write_text(json.dumps(result,indent=2)+"\n")
    print(result["worstWallSection"])
    for k,v in result["peakPanBySupport"].items():print(k,v)
