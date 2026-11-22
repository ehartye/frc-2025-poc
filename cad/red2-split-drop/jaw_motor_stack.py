"""Use the source-assembled REV two-stage NEO 550 stack to register the jaw drive."""
import argparse
import json
import math
import sys
from pathlib import Path
import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder,GeomAbs_Plane
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE,TopAbs_SOLID
from OCP.TopoDS import TopoDS
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from output_mount import ROOT,OUT,read,digest,place,matrix


def faces(shape):
    ex=TopExp_Explorer(shape,TopAbs_FACE)
    while ex.More():
        f=TopoDS.Face(ex.Current());ex.Next()
        yield f,BRepAdaptor_Surface(f)


def run(plugin,cache):
    sys.path[:0]=[str(plugin/'scripts'),str(ROOT/'experiments/swerve-drive/packaging')]
    from showcase_contacts import load_solids,box_gap
    from mechanical_motion import bounds,distance,common
    from envelopes import load_raw
    master=read(ROOT/'cad/fusion-authority.json')['source']['archive'];cold=read(OUT/'r5-load-01.json')
    assert digest(ROOT/master['file'])==master['sha256']==cold['authority']['archiveSha256']
    assert digest(OUT/'r5-export.step')==cold['step']['sha256']
    dr=read(ROOT/'experiments/swerve-drive/packaging/module.json')
    src=cache/'REV-21-3005.STEP';assert digest(src)==dr['sourceStepSha256']
    actual=load_raw(cache);chosen={k:s for k,s,_ in actual if k in
        {'1-REV-21-1651','2-REV-41-1601','3-REV-41-1602','4-REV-41-1608','5-REV-41-1607'}}
    assert len(chosen)==5
    def group(s):
        ex=TopExp_Explorer(s,TopAbs_SOLID);result=[]
        while ex.More():result.append(TopoDS.Solid(ex.Current()));ex.Next()
        return result
    def vol(s):
        qp=GProp_GProps();BRepGProp.VolumeProperties_s(s,qp);return qp.Mass()
    # The largest body in the 3:1 stage is its ring housing. Its 6 assembly
    # holes establish the true rotation, and the output-side flat its station.
    source_stack=chosen['2-REV-41-1601']
    ring=max(group(source_stack),key=vol)
    holes=set();planes={}
    for face,surf in faces(ring):
        if surf.GetType()==GeomAbs_Cylinder:
            c=surf.Cylinder();u=c.Axis().Direction()
            if abs(u.Y())>.9999 and abs(c.Radius()-1.6)<.005:
                p=c.Axis().Location();holes.add((round(p.X(),6),round(p.Z(),6)))
        if surf.GetType()==GeomAbs_Plane:
            p=surf.Plane()
            if abs(p.Axis().Direction().Y())>.9999:
                qp=GProp_GProps();BRepGProp.SurfaceProperties_s(face,qp)
                z=round(p.Location().Y(),6);planes[z]=planes.get(z,0)+qp.Mass()
    assert len(holes)==6,(holes,planes)
    center=np.mean(list(holes),axis=0);pattern=sorted(math.atan2(v-center[1],u-center[0]) for u,v in holes)
    # The 137 mmÂ² flat is the female end of REV-41-1601, measured as
    # z=-5 in the independently issued cartridge. Other end has male lobes.
    yface,=[k for k,v in planes.items() if abs(v-137.16)<.5]
    bare=OUT/'supplier/REV-41-1615.STEP';output=load_solids(bare)[0]
    output_pose=matrix(-18.75,[-147.,0.,515.]);stock={k:place(s,output_pose) for k,s in output.items()}
    fixed=load_solids(OUT/'r5-export.step')[0];boxes={n:bounds(s) for n,s in fixed.items()}
    # Six bolt orientations align. Each keeps the source-assembled gear
    # mesh, keyed connection, pinion and motor in its actual supplier pose.
    cases={}
    for turn in range(6):
        angle=pattern[-1]+turn*math.pi/3
        c,s=math.cos(angle),math.sin(angle)
        # Supplier XZ maps to output XY, +Y to output +Z. To keep a proper
        # rotation, one transverse axis reverses.
        ortho=np.array([[c,0,s],[s,0,-c],[0,1,0]])
        anchor=np.array([*center[:1],yface,center[1]])
        local=np.eye(4);local[:3,:3]=ortho;local[:3,3]=[0,0,11.25]-ortho@anchor
        exp=np.eye(4);exp[:3,:]=output_pose
        world=(exp@local)[:3,:];names={};clash=[];housing_contacts=[]
        for sku,shape in chosen.items():
            for i,solid in enumerate(group(shape)):
                n=sku+'/'+str(i);part=place(solid,world);names[n]=part
                bb=bounds(part)
                for k,other in stock.items():
                    if box_gap(bb,bounds(other))>.001:continue
                    gap=distance(part,other)
                    if gap<.001:
                        v=common(part,other) if gap<1e-6 else 0.
                        housing_contacts.append(dict(article=n,other=k,gapMm=gap,commonMm3=v))
                for k,other in fixed.items():
                    if box_gap(bb,boxes[k])>.001:continue
                    if distance(part,other)<1e-6:
                        v=common(part,other)
                        if v>.001:clash.append(dict(article=n,other=k,commonMm3=v))
        cases[str(turn)]=dict(orientationMatrixMm=world.tolist(),groupSolids=len(names),contacts=housing_contacts,
                              installedR5Clashes=clash,positiveInternal=[r for r in housing_contacts if r['commonMm3']>.001])
        print(turn,len(clash),[(r['article'],r['other'],round(r['commonMm3'],2)) for r in housing_contacts if r['commonMm3']>.001],flush=True)
        (OUT/'jaw-drive-source-stack.json').write_text(json.dumps(dict(authority=master,fixture=cold['sha256'],
            sourceModule=digest(src),outputSource=digest(bare),sourceGroup=list(chosen),sourceHolesXZMm=sorted(holes),
            sourceAxisXZMm=center.tolist(),cartridgeOutputFaceYmm=yface,outputRearMountPlaneZmm=11.25,
            targetPhiDeg=-18.75,qualifiedCases=len(cases),cases=cases),indent=2)+'\n')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('plugin',type=Path);p.add_argument('cache',type=Path)
    a=p.parse_args();run(a.plugin,a.cache)
