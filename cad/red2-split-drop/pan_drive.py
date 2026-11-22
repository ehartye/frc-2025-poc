"""Register real supplier drivetrain solids in the one-axis pan shaft frame."""
import json
import math
import hashlib
import sys
from pathlib import Path
import numpy as np
from OCP.BRep import BRep_Builder
from OCP.BRepTools import BRepTools
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS,TopoDS_Shape
from output_mount import place, matrix, ROOT, OUT, digest, read

DIR=ROOT/'artifacts/red2-integrated'
SOURCE=DIR/'alternative-supplier'
def hom(mat):
    result=np.eye(4);result[:3,:]=mat
    return result

def props(shape):
    p=GProp_GProps();BRepGProp.VolumeProperties_s(shape,p)
    return p.Mass(),np.array(p.CentreOfMass().Coord())

def main(plugin,project):
    sys.path.insert(0,str(plugin/'scripts'))
    from showcase_contacts import load_solids
    from mechanical_motion import bounds
    base=read(ROOT/'cad/red2-split-drop/chassis-foundation-r2.json')
    assert digest(DIR/'red2-integrated-foundation-r2.f3d')==base['nativeArchiveSha256']
    old=read(OUT/'jaw-drive-source-stack.json')
    assert digest(ROOT/old['authority']['file'])==old['authority']['sha256']==base['authority']['archiveSha256']
    kit=project/'artifacts/swerve-package'
    assert digest(kit/'REV-21-3005.STEP')==old['sourceModule']
    output=hom(matrix(-18.75,[-147.,0.,515.]))
    donor_to_output=np.linalg.inv(output)@hom(old['cases']['0']['orientationMatrixMm'])
    target=DIR/'pan-supplier';target.mkdir(exist_ok=True)
    rows=[]
    def register(sku,src,coordinate,article,provenance):
        rec=dict(id=sku,source=src.relative_to(ROOT).as_posix(),sourceSha256=digest(src),sourceBasis=provenance,frameMm=coordinate[:3].tolist(),boundsInShaftFrameMm=bounds(place(article,coordinate[:3])))
        rows.append(rec)
    def whole(file):
        from OCP.STEPControl import STEPControl_Reader
        q=STEPControl_Reader()
        assert q.ReadFile(str(file))==IFSelect_RetDone
        q.TransferRoots();return q.OneShape()
    def elements(shape):
        e=TopExp_Explorer(shape,TopAbs_SOLID);ret=[]
        while e.More():
            ret.append(TopoDS.Solid(e.Current()));e.Next()
        return ret
    donor={}
    for i,sku in enumerate(['REV-21-1651','REV-41-1601','REV-41-1602','REV-41-1608','REV-41-1607'],1):
        original=kit/'vendor-native'/(str(i)+'-'+sku+'.brep')
        s=TopoDS_Shape()
        assert BRepTools.Read_s(s,str(original),BRep_Builder())
        donor[sku]=s
        dest=target/(sku+'-from-original-module.step')
        if not dest.exists():
            w=STEPControl_Writer();w.Transfer(s,STEPControl_AsIs);assert w.Write(str(dest))==IFSelect_RetDone
        shift=np.eye(4)
        if sku in ('REV-21-1651','REV-41-1608','REV-41-1607'):shift[2,3]=10.
        register(sku,dest,shift@donor_to_output,s,{'assembly':old['sourceModule'],'originalKernel':digest(original),'occurrence':i})
    # Source issue plates use the same patterned cartridge model for all ratios.
    # Fit the standalone 5:1 housing to the previously assembled 4:1 ring using
    # actual centres of all three independent planets and the two retainer plates.
    orig=elements(donor['REV-41-1602'])
    extra=whole(SOURCE/'REV-41-1603.STEP')
    own=elements(extra)
    assert len(orig)==len(own)==13
    entries=[[props(v) for v in seq] for seq in (own,orig)]
    snap=[[(float(v),pt.tolist()) for v,pt in part] for part in entries]
    (DIR/'pan-cartridge-landmarks.json').write_text(json.dumps(snap,indent=2))
    from jaw_motor_stack import faces
    from OCP.GeomAbs import GeomAbs_Cylinder
    def holes(ring):
        positions=set()
        for _,sf in faces(ring):
            if sf.GetType()!=GeomAbs_Cylinder:continue
            cyl=sf.Cylinder();axis=cyl.Axis();pt=axis.Location()
            if abs(cyl.Radius()-1.6)<.01 and abs(axis.Direction().Z())>.9999:
                positions.add((round(pt.X(),4),round(pt.Y(),4)))
        if len(positions)!=6:raise ValueError('Expected six true mounting bores '+str(positions))
        return np.array(sorted(positions))
    ringA=max(own,key=lambda x:props(x)[0]);ringB=max(orig,key=lambda x:props(x)[0])
    oldring=place(ringB,donor_to_output[:3]);xy0,xy1=holes(ringA),holes(oldring)
    zoff=props(oldring)[1][2]-props(ringA)[1][2]
    # Distinct external holes fix six possible clocks. Original 4:1 and
    # extra 5:1 carriers have four planet pins; choose the registered clock
    # that least changes their supplier-issued physical clock.
    def rot2(phi):
        a,b=math.cos(phi),math.sin(phi);return np.array([[a,-b],[b,a]])
    aa=np.array([pt[:2] for vol,pt in entries[0] if 60<vol<68])
    bb=np.array([(donor_to_output@[*pt,1])[:2] for vol,pt in entries[1] if 60<vol<68])
    def err(v,w):return max(min(np.linalg.norm(a-b) for b in w) for a in v)
    config=[]
    for v in xy1:
        phi=math.atan2(v[1],v[0])-math.atan2(xy0[0][1],xy0[0][0])
        q=rot2(phi);config.append((err(aa@q.T,bb),err(xy0@q.T,xy1),phi))
    pinerr,holeerr,angle=min(config)
    if holeerr>.002 or not 25<zoff<28:raise ValueError('Supplier registration changed '+str(config))
    registered=np.eye(4);registered[:2,:2]=rot2(angle);registered[2,3]=round(zoff*4)/4+10
    register('REV-41-1603',SOURCE/'REV-41-1603.STEP',registered,extra,dict(reference='as-assembled REV-41-1602',physicalHoleErrorMm=holeerr,pinClockDifferenceMm=pinerr,insertionPitchMm=10.,measuredStageDatumMm=zoff))
    output_part=OUT/'supplier/REV-41-1615.STEP'
    register('REV-41-1615',output_part,np.eye(4),whole(output_part),'Supplier V2 output stage; default inner phase unqualified.')
    native_adapter=hom(np.array([[math.cos(math.radians(11.25)),math.sin(math.radians(11.25)),0,-147],[math.sin(math.radians(11.25)),-math.cos(math.radians(11.25)),0,0],[0,0,-1,521]]))
    adapter=OUT/'supplier/REV-41-1620.STEP'
    register('REV-41-1620',adapter,np.linalg.inv(output)@native_adapter,whole(adapter),'Actual R4 output adapter seat relative to V2 housing.')
    for screw in read(OUT/'output-mount-fit.json')['faceScrews'].values():
        article=OUT/'supplier/2800-0003-0008.STEP'
        register('M3x8 adapter '+str(screw['outputAngleDeg']),article,np.linalg.inv(output)@hom(screw['transform']),whole(article),'Three existing real output flange fasteners.')
    for angle in (30,90,150,210,270,330):
        a=math.radians(angle)
        pose=np.eye(4)
        # Supplier screw head seat is at z=-12. First face of a 4 mm
        # bracket lies at z=-4; the 8 mm shank takes 4 mm of M3 engagement.
        pose[:3,3]=[16*math.cos(a),16*math.sin(a),8]
        article=OUT/'supplier/2800-0003-0008.STEP'
        register('M3x8 housing '+str(angle),article,pose,whole(article),'M3 front face 32 mm housing bolt circle; independent clockwise datum.')
    bearing=SOURCE/'REV-21-1915.STEP'
    for side in ['drive','idler']:
        shift=np.eye(4)
        # Shaft frame origin maps to +216 mm Y; bearing centres +/-195.
        shift[2,3]=195-216 if side=='drive' else -195-216
        if side=='idler':shift[1,1]=-1;shift[2,2]=-1
        register('REV-21-1915 '+side,bearing,shift,whole(bearing),'True purchased flanged half-inch hex bearing.')
    data=dict(authority=old['authority'],foundation=base['nativeArchiveSha256'],components=rows,configuration='5:1 then 4:1 then 3:1, NEO 550 to V2 output',limitations=['Manufacturer models the internals of all three purchasable cartridge ratios with the same nominal solid geometry. They support actual external packaging, but not ratio-specific internal tooth acceptance.','Default REV-41-1615 output-shaft phase interfered with the donor carrier in the prior native issue. Resolve shaft rotation, retention and tooth clearance before power or service acceptance.'])
    (DIR/'pan-drive-registration.json').write_text(json.dumps(data,indent=2)+'\n')
    print([(r['id'],[round(x,2) for x in r['boundsInShaftFrameMm']]) for r in rows])

if __name__=='__main__':main(Path(sys.argv[1]),Path(sys.argv[2]))
