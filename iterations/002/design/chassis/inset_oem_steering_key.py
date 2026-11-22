"""Fit the actual short supplier key in the 2025 L2 steering pinion on its measured shaft datum."""
import hashlib,json,sys
from pathlib import Path
import numpy as np
from inset_chassis_shop import HERE

def run(pkg,ref,key_file,motor_file):
    sys.path.insert(0,str(pkg/"scripts"))
    from mechanical_motion import bounds,common,bearing_area
    from showcase_contacts import load_solids
    from fusion_authority import register_path,verify
    from OCP.gp import gp_Trsf
    from OCP.TopLoc import TopLoc_Location
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    root=HERE.parents[3];reg=register_path(root);auth=verify(root,reg,reg.parent/"issue-005/lineage.json")
    installed=json.loads((HERE/"inset-steering-key-installation.json").read_bytes())
    if auth!=installed["authority"] or sha(key_file)!="2b665b82f813da7d774cc5521402ec93a9480aa855a723c4f4d649b128804dd7" or sha(motor_file)!=installed["standaloneV11"] or sha(ref)!=installed["alternateDatumStudy"]:raise ValueError("Unbound comparison")
    kit,sheets=load_solids(key_file);motor=next(iter(load_solids(motor_file)[0].values()));keys=[(k,v) for k,v in kit.items() if abs(bounds(v)[5]-bounds(v)[2]-8.5)<.001]
    if sheets or len(kit)!=2 or len(keys)!=1:raise ValueError("Key kit changed")
    ident,key=keys[0];kb=bounds(key);refdata=json.loads(ref.read_bytes());c=-refdata["keyFloorDirection"][2];s=refdata["keyFloorDirection"][0]
    S=np.eye(4);S[:3,:3]=[[c,s,0],[0,0,1],[s,-c,0]];S[:3,3]=refdata["translationMm"]
    def place(shape,M):
        tf=gp_Trsf();tf.SetValues(*M[:3].ravel());return shape.Moved(TopLoc_Location(tf))
    donor_file=HERE.parent/"private-inputs/matched-sds/corner.step"
    if sha(donor_file)!=installed["cornerStep"]:raise ValueError("Donor mismatch")
    donor,_=load_solids(donor_file);register=next(v for v in installed["rows"] if v["corner"]=="FL" and v["role"]=="steering")
    stock=donor[register["motor"]];gear=donor[register["gear"]];motor=place(motor,S);gb=bounds(gear)
    # Sweep the actual purchased key, with its flat fully on the selected R2.8 floor.
    rows=[]
    for at in (123.9125,125.6625,126.1125,127.4125):
        T=np.eye(4);T[:3,3]=[0,2.8-kb[1],at-S[1,3]-kb[2]];item=place(key,S@T)
        pairs=[]
        for name,part in (("configured donor NEO",stock),("specified REV-21-1650 V1.1",motor),("configured 14T pinion",gear)):
            dirs=[S[:3,0],-S[:3,0],-S[:3,1]]
            pairs.append(dict(name=name,commonMm3=[common(item,part),common(part,item)],
                parkedKeyFaceAreaMm2=[bearing_area(item,part,dir) for dir in dirs]))
        rows.append(dict(sourceYIntervalMm=[at,at+8.5],axialAreaOverlapUpperLengthMm=max(0,min(gb[4],at+8.5)-max(gb[1],at)),pairs=pairs))
    return dict(authority=auth,cold508=installed["cold508"],donorStep=sha(donor_file),keyKit=sha(key_file),keyArticle=ident,
        keyBoundsAsDelivered=kb,motorSHA256=sha(motor_file),referenceShaftPoseMm=S.tolist(),rows=rows,
        scope="One common local interface, after rechecking all four source-bound steering pairs. Exact purchased metal and torque-face direction. No assumed shaft grade, axial retainer, rated flank allowable or power transmission from axial envelope. Relative movement under torque/runout and the kit's axial retention still require assessment.")

if __name__=="__main__":
    d=run(*map(Path,sys.argv[1:]));(HERE/"inset-oem-steering-key.json").write_text(json.dumps(d,indent=1)+"\n")
