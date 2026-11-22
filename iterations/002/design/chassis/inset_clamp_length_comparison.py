"""Compare the 5/8 and 1 inch authentic clamp screws in the seven-part candidate on the issued 654."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt
from inset_stock_check import HERE,form
from port_anchor_fit import oriented

def run(scripts):
    sys.path.insert(0,str(scripts))
    from fusion_authority import register_path,verify
    from showcase_contacts import load_solids,box_gap
    from mechanical_motion import bounds,common,distance,bearing_area
    reg=register_path(HERE.parents[3]);auth=verify(HERE.parents[3],reg,reg.parent/"issue-006/lineage.json")
    load=lambda n:json.loads((HERE/n).read_bytes())
    binding=load("connected-654-exchange.json");issue=load("connected-654-issuance.json");shapes=load("inset-corrected-seven-shapes.json")
    plan=load("inset-corrected-brace-foot-clearance.json");pin=load("inset-mount-purchases.json")["originals"]
    if auth!=shapes["authority"] or auth["archiveSha256"]!=issue["native"] or issue["canonicalStep"]!=auth["stepSha256"]:raise ValueError("Changed issue")
    all_bodies,open_shells=load_solids(reg.parent/"issue-006/assembly.step")
    if open_shells or len(all_bodies)!=654:raise ValueError("Incomplete solid issue")
    by_role={r["target"]:r["exchange"] for r in binding["pairs"]}
    current={k:all_bodies[n] for k,n in by_role.items()}
    for k,row in plan["stockChanges"].items():current[k]=form(row)
    boxes={k:bounds(b) for k,b in current.items()}
    tests=[]
    for sku,length in [("91251A539",15.875),("91251A542",25.4)]:
        source=pin[sku];role="FL/"+source["source"];s=current[role]
        shift=np.array(boxes[role])-np.array(source["boundsMm"])
        if max(abs(shift[:3]-shift[3:]))>.005:raise ValueError("Supplier screw has changed shape or orientation")
        point=np.array(source["underHeadCenterMm"])+shift[:3];axis=source["intoThreadUnit"]
        for x,at in [(234,132),(266,158)]:
            for sign in [-1,1]:
                y=sign*at;z=193.675
                host=f"Captive truss landing nutbar 250 {145*sign}";foot=f"Truss landing 250 {145*sign}"
                screw=oriented(s,point,axis,[x,y,z],[0,0,-1]);flat=bearing_area(screw,current[foot],[0,0,-1])
                def cyl(r,start,travel):
                    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x,y,start),gp_Dir(0,0,1)),r,travel).Shape()
                # Full-head-diameter envelope contains the screw during its axial extraction.
                # The installed thread/through bore below the original seat is a separate path.
                gauges={"parked":screw,"lift head until tip clears foot":cyl(4.7625,z+.001,length+6.35),"bit insertion":cyl(3.68,z+3.05,47)}
                observations={}
                for gate,q in gauges.items():
                    gauge_bounds=bounds(q)
                    candidates=sorted((box_gap(gauge_bounds,bb),k) for k,bb in boxes.items())
                    tight=[];minimum=float("inf");nearest=None
                    for b,k in candidates:
                        if b>max(.0005,minimum) and b>2:break
                        if gate=="parked" and k in (host,foot):continue
                        gap=distance(q,current[k]);v=[common(q,current[k]),common(current[k],q)] if gap<.001 else [0.,0.]
                        if gap<minimum:minimum=gap;nearest=k
                        if gap<2 or max(v)>.001:tight.append(dict(part=k,gapMm=gap,commonMm3=v))
                    observations[gate]=dict(nearest=nearest,minimumMm=minimum,tight=tight)
                tests.append(dict(sku=sku,source=by_role[role],sourceSeatMm=point.tolist(),underHead=[x,y,z],axis=[0,0,-1],
                      tappingHost=host,nominalPlanarHeadBearingMm2=flat,sourceToTargetRotation="180 about world X" if axis[2]>.9 else "identity",
                      observations=observations))
                (HERE/"inset-clamp-length-progress.json").write_text(json.dumps(tests,indent=1)+"\n")
    return dict(authority=auth,retainedIssue=issue,changedFormReference=shapes["sourceSHA256"],results=tests,
                scope="Parked and conservative upward head/bit envelopes against all 654 on the private machining issue with exactly seven represented planned substitutions. The tapped interface is deliberately excluded pending a real modeled female thread and supplier clock. Assembly and preload are separately reviewed.")
if __name__=="__main__":
    d=run(Path(sys.argv[1]));(HERE/"inset-clamp-length-comparison.json").write_text(json.dumps(d,indent=1)+"\n")
