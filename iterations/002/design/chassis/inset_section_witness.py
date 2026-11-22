"""Read the centroidal modulus from independent solid cuts. Retain the original wrong-fibre-distance regression."""
import hashlib,json,math,sys
from pathlib import Path

import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Ax1,gp_Pnt,gp_Dir

from crossmember_envelope import box,cylinder
from four_corner_packaging import outline,polygon
from port_anchor_fit import cut
from structural_perimeter import tube_segment
from direct_saddle_inertia import ring_section

HERE=Path(__file__).resolve().parent


def cross_section(solid,x,span):
    from mechanical_motion import bounds
    min_y,max_y,bottom,top=span
    plane=polygon([[x,min_y,bottom],[x,max_y,bottom],[x,max_y,top],[x,min_y,top]])
    op=BRepAlgoAPI_Common(solid,plane)
    if not op.IsDone() or op.Shape().IsNull():raise ValueError("No finite stock cut")
    face=op.Shape()
    integral=GProp_GProps()
    (getattr(BRepGProp,"SurfaceProperties_s",None) or BRepGProp.SurfaceProperties)(face,integral)
    if integral.Mass()<=0:raise ValueError("Empty section")
    p=integral.CentreOfMass()
    I=integral.MomentOfInertia(gp_Ax1(p,gp_Dir(0,0,1)))
    bb=bounds(face)
    c=max(p.Y()-bb[1],bb[4]-p.Y())
    return dict(areaMm2=integral.Mass(),centroidMm=list(p.Coord()),IzzMm4=I,extremeMm=c,
                modulusMm3=I/c,stressPerMillionNmm=1e6*c/I,boundsMm=bb)


def run(package):
    sys.path.insert(0,str(package/"scripts"))
    from fusion_authority import register_path,verify
    from component_shape import volume
    root=HERE.parents[3];reg=register_path(root);line=reg.parent/"issue-005/lineage.json"
    source=verify(root,reg,line)
    controls=[]
    for w in (20.,30.):
        h=40.;a=cross_section(box((-75,-w/2,0),(75,w/2,h)),0,(-25,25,-3,50))
        expected=h*w*w/6
        if abs(a["modulusMm3"]-expected)>1e-4 or abs(a["extremeMm"]-w/2)>1e-5:
            raise ValueError("Failed independently known solid rectangular modulus")
        controls.append(dict(widthMm=w,heightMm=h,closedFormMm3=expected,measured=a))
    if abs(controls[1]["measured"]["modulusMm3"]/controls[0]["measured"]["modulusMm3"]-2.25)>1e-6:
        raise ValueError("Widened solid section scaling failed")
    # Model the 552-derived candidate's real, mitered starboard segment. Solids below
    # are supplied as candidate input only until cold Fusion has issued this topology.
    source_fit=HERE/"structural-perimeter-101.6-38.1-101.6-inset-12.7.json"
    pose=json.loads(source_fit.read_bytes())
    if source!=pose["authority"] or pose["cornerInsetXYmm"]!=12.7:raise ValueError("Changed candidate source")
    sections={}
    for wall in (3.,3.175):
        outer=outline(370,365,42,0);inner=outline(370,365,42,38.1)
        base=tube_segment(outer,inner,4,38.1,wall,101.6,101.6)
        for stagger in (False,True):
            shape=base
            for c in (-127.3,127.3):
                for z,spread in ((121.,6. if stagger else 13.),(149.,13.)):
                    for x in (c-spread,c+spread):
                        shape=cut(shape,cylinder((x,-366,z),(0,1,0),4.05,44.))
            from OCP.STEPControl import STEPControl_Writer,STEPControl_AsIs
            from OCP.IFSelect import IFSelect_RetDone
            name=f"candidate-starboard-{wall:g}-{'staggered' if stagger else 'aligned'}.step"
            folder=HERE.parent/"cad/private/section-trials"
            folder.mkdir(parents=True,exist_ok=True)
            step=folder/name;writer=STEPControl_Writer()
            if writer.Transfer(shape,STEPControl_AsIs)!=IFSelect_RetDone or writer.Write(str(step))!=IFSelect_RetDone:
                raise ValueError("Candidate section STEP did not serialize")
            from showcase_contacts import load_solids
            solids,sheets=load_solids(step)
            if len(solids)!=1 or sheets:raise ValueError("Expected one stock article")
            source_id,shape=next(iter(solids.items()))
            rows={}
            for x in (-114.3,-121.3,-119.3,-127.3,-140.3,0.):
                obs=cross_section(shape,x,(-370.,-310.,95.,210.))
                if wall==3.:
                    A,I,c=ring_section(np.array([x,-345.95]),np.array([1.,0.]),12.7,stagger)
                    error=max(abs(obs["areaMm2"]-A),abs(obs["IzzMm4"]-I))
                    if error>.02 or abs(obs["extremeMm"]-c)>1e-5:raise ValueError(f"Section authority discrepancy {x} {error}")
                rows[str(x)]=obs
            sections[f"{wall:g}/{'staggered' if stagger else 'aligned'}"]=dict(sourceStep=str(step.relative_to(HERE.parent).as_posix()),stepSha256=hashlib.sha256(step.read_bytes()).hexdigest(),sourceSolid=source_id,stockVolumeMm3=volume(shape),
                analyticWrongFibreNegative=12.7,independentReads=rows)
    return dict(sourceAuthority=source,projectedStockRecordSha256=hashlib.sha256(source_fit.read_bytes()).hexdigest(),
        closedFormControl=controls,sections=sections,scope="The currently registered native assembly is 552 only: it lacks this ring. These are geometric cuts of the independently authored exact mitered candidate stock with the actual commissioned drill sites at minimum and nominal wall; their section properties must be repeated against the corresponding cold Fusion derivative after issue. The deliberately retained 12.7 mm original fibre radius understates bending stress by one third.")


if __name__=="__main__":
    d=run(Path(sys.argv[1]));(HERE/"inset-perimeter-section-witness.json").write_text(json.dumps(d,indent=2)+"\n")
    for k,v in d["sections"].items():
        r=v["independentReads"]["-114.3"];print(k,r["areaMm2"],r["IzzMm4"],r["extremeMm"],r["modulusMm3"])
