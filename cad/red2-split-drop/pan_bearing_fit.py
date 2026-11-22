"""Retain the misseated flanges and recheck rigid placement of original REV bearings."""
import sys,json,math
from pathlib import Path
import numpy as np
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt
from pan_mechanics import stock,rotation
from output_mount import ROOT,place,digest,read
D=ROOT/'artifacts/red2-integrated'
def hom(r):
    m=np.eye(4);m[:3]=r;return m
def vol(shape):
    g=GProp_GProps();BRepGProp.VolumeProperties_s(shape,g);return g.Mass()
def study(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import distance,common
    from showcase_contacts import load_solids
    plan=read(D/'pan-shop-plan.json');drive=read(D/'pan-drive-registration.json')
    assert digest(ROOT/plan['authority']['file'])==plan['authority']['sha256']
    shift=np.array([[1,0,0,205],[0,0,1,216],[0,-1,0,580],[0,0,0,1]])
    ps={v['id']:v for v in drive['components']};ss={v['id']:v for v in plan['items']}
    def genuine(name):
        r=ps[name];src=ROOT/r['source'];assert digest(src)==r['sourceSha256']
        return next(iter(load_solids(src)[0].values()))
    original=genuine('REV-21-1915 drive')
    cutter=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0,0,-5),gp_Dir(0,0,1)),9.52,10).Shape()
    outer=BRepAlgoAPI_Cut(original,cutter).Shape()
    phi=math.radians(31.7);c,s=math.cos(phi),math.sin(phi)
    spun=place(outer,np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0]]))
    outer_sym=vol(outer)-common(outer,spun)
    parts={}
    for name,label,side in [('drive','Port',1),('idler','Starboard',-1)]:
        spec=ps['REV-21-1915 '+name]
        carrier=shift@hom(spec['frameMm'])
        housing=stock(ss[label+' bearing tower'])
        if name=='drive':
            pin=place(genuine('REV-41-1620'),(shift@hom(ps['REV-41-1620']['frameMm']))[:3])
        else:
            pin=stock(ss['Idler headed stub hex shank'])
        observations=[]
        for desc,offset,clock in [('as-drawn',0,0),('flange-only',.775,0),('hex-only',0,30),('seated-clocked',.775,30)]:
            rg=carrier.copy();rg[1,3]+=side*offset
            angle=math.radians(clock);c,s=math.cos(angle),math.sin(angle)
            twist=np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]])
            rg=rg@twist
            for angle in (0,70,34,-2):
                # Entire supplier article is a single solid. The volume outside
                # the bearing's inner race is provably symmetric within the
                # numeric volume tolerance. This represents the internal hex spin
                # and leaves the round outer envelope registered to its tower.
                pose=hom(rotation(angle))
                moved=place(original,(pose@rg)[:3]);axis=place(pin,pose[:3])
                observations.append(dict(case=desc,panAngleDeg=angle,housingSeatGapMm=distance(moved,housing),housingCommonMm3=common(moved,housing),shaftGapMm=distance(moved,axis),shaftCommonMm3=common(moved,axis),installedMatrixMm=(pose@rg)[:3].tolist()))
        parts[name]=observations
    out=dict(authority=plan['authority'],foundation=plan['foundationSha256'],drivePlanSha256=digest(D/'pan-drive-registration.json'),stockPlanSha256=digest(D/'pan-shop-plan.json'),sourceSha256=ps['REV-21-1915 drive']['sourceSha256'],outerRaceGaugeRadiusMm=9.52,rotatedOuterDifferenceMm3=outer_sym,parts=parts,limit='The model is one purchased solid. Captured outer profile is a surface of revolution; no independent bearing axial shock rating or clearance tolerance is implied. The article remains unaltered.')
    (D/'pan-bearing-placement.json').write_text(json.dumps(out,indent=2)+'\n')
    print(outer_sym,[(n,[(r['case'],round(r['shaftCommonMm3'],4),round(r['housingCommonMm3'],4)) for r in p if r['panAngleDeg']==0]) for n,p in parts.items()])
if __name__=='__main__':study(Path(sys.argv[1]))
