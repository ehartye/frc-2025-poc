"""Screen editable pan stock and integral mount contacts with exact finite solids."""
import sys
import math
import json
from pathlib import Path
import numpy as np
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Pnt,gp_Dir,gp_Ax2
from output_mount import ROOT,read,digest,place
DIR=ROOT/'artifacts/red2-integrated'

def loft(profiles):
    f=BRepOffsetAPI_ThruSections(True,True)
    for section in profiles:
        p=BRepBuilderAPI_MakePolygon()
        for xyz in section:p.Add(gp_Pnt(*xyz))
        p.Close();f.AddWire(p.Wire())
    f.Build()
    if not f.IsDone():raise ValueError('Could not construct sections')
    return f.Shape()

def stock(part):
    s=loft(part['sections'])
    def cut(geometry,tool):
        f=BRepAlgoAPI_Cut(geometry,tool);f.Build()
        if not f.IsDone():raise ValueError(part['id'])
        return f.Shape()
    if part['void']:s=cut(s,loft(part['void']))
    for b in part['bores']:
        s=cut(s,BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*b['start']),gp_Dir(*b['direction'])),b['radius'],b['depth']).Shape())
    for h in part['hexes']:
        r=h['flats']/math.sqrt(3);x,z=h['center']
        s=cut(s,loft([[[x+r*math.cos(math.radians(60*j+30)),y,z+r*math.sin(math.radians(60*j+30))] for j in range(6)] for y in (h['y0'],h['y1'])]))
    return s

def rotation(deg):
    c=math.cos(math.radians(deg));s=math.sin(math.radians(deg))
    return np.array([[c,0,-s,205-205*c+580*s],[0,1,0,0],[s,0,c,580-205*s-580*c]])

def study(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from showcase_contacts import box_gap
    from mechanical_motion import bounds,common
    rec=read(DIR/'pan-shop-plan.json')
    assert digest(ROOT/rec['authority']['file'])==rec['authority']['sha256']
    assert digest(DIR/'red2-integrated-foundation-r2.f3d')==rec['foundationSha256']
    solids={p['id']:stock(p) for p in rec['items']}
    rows={};shapes={};masses={};source={p['id']:p for p in rec['items']}
    for name,shape in solids.items():
        p=source[name];g=GProp_GProps();BRepGProp.VolumeProperties_s(shape,g)
        rho=2.68e-6 if '5052' in p['material'] else 2.70e-6
        mass=g.Mass()*rho;x,y,z=g.CentreOfMass().Coord()
        iy=g.MatrixOfInertia().Value(2,2)*rho/1e6+mass*((x-205)**2+(z-580)**2)/1e6
        masses[name]=dict(massKg=mass,centreMm=[x,y,z],moving=p['moving'],momentOfInertiaAroundPanAxisKgM2=iy)
        if mass<=0:raise ValueError('Empty stock '+name)
    for label,deg in [('flat',0),('stowed',70),('transition',34),('dump',-2)]:
        parts={k:place(s,rotation(deg)) if source[k]['moving'] else s for k,s in solids.items()}
        boxes={k:bounds(s) for k,s in parts.items()};pairs=[]
        for i,(k,a) in enumerate(parts.items()):
            for h,b in list(parts.items())[i+1:]:
                if label!='flat' and source[k]['moving']==source[h]['moving']:continue
                if box_gap(boxes[k],boxes[h])>.001:continue
                vol=common(a,b)
                if vol>.01:pairs.append(dict(a=k,b=h,commonMm3=vol))
        rows[label]=pairs
    movable=[r for r in masses.values() if r['moving']]
    weights=[r['massKg'] for r in movable];centres=np.array([r['centreMm'] for r in movable])
    cg=np.average(centres,axis=0,weights=weights);inertia=sum(r['momentOfInertiaAroundPanAxisKgM2'] for r in movable)
    gear=(68/13)*(76/21)*(84/29);results=[]
    for deg in range(-5,74):
        theta=math.radians(deg)
        horizontal=((cg[0]-205)*math.cos(theta)-(cg[2]-580)*math.sin(theta))/1000
        acc=6*math.radians(75)/1.6**2
        torque=abs(sum(weights)*9.80665*horizontal)+inertia*acc+2.721
        results.append(torque)
    fpan=rec['impacts']['panImpact']['normalN'];force=1.5*fpan
    # Single midspan concentrated top-edge strike on the physical 20x20x2
    # square tube, spanning one 45 mm web interval. No assumed sheet load width.
    I=(20**4-16**4)/12;stress=force*45*10/(4*I)
    guide=1.5*rec['impacts']['guideImpact']['normalN']
    # One 8x16 solid post, 300 mm unsupported. Parallel to long dimension
    # gives optimistic compliance; independent weak-axis case is retained.
    entries=[]
    for width,depth,innerW,innerD in [(25,25,20,20),(8,16,0,0),(16,8,0,0)]:
        Ipost=(width*depth**3-innerW*innerD**3)/12
        entries.append(dict(sectionMm=[width,depth,innerW,innerD],installed=width==25,stressMPa=guide*300*depth/(2*Ipost),deflectionMm=guide*300**3/(3*69000*Ipost)))
    out=dict(authority=rec['authority'],foundation=rec['foundationSha256'],shopPlan=digest(DIR/'pan-shop-plan.json'),masses=masses,stockPairClashes=rows,
        movingShopStockKg=sum(weights),fixedShopStockKg=sum(v['massKg'] for v in masses.values() if not v['moving']),movingShopCentreMm=cg.tolist(),movingStockInertiaKgM2=inertia,
        driver=dict(actualRatio=gear,gearboxOutputLoadBoundNm=max(results),includes='Moving shop gravity, acceleration and peak independent pipe reaction; hub, fastener and rotor inertia excluded until supplier masses are confirmed.',
                    sensitivity=[dict(efficiency=eta,amps=max(results)/(.97/100*gear*eta)) for eta in (.45,.60,.75)],
                    motionPeakRPM=math.radians(75)*1.5/1.6*60/(2*math.pi)*gear),
        movingGuide=dict(forceN=force,edgeTubeBendingMPa=stress,edgeTubeDeflectionMm=force*45**3/(48*69000*I),postWeldLimitComparisonMPa=105,captureLimits='Local dent, weld transfer and pipe-rim contact-width demand remain open.'),
        fixedInlet=dict(forceN=guide,unbracedWorstLeverMm=300,postCases=entries),disposition='Private partial dimensional and elastic study. Assembly-level bolting, failure, duty and native qualification remain open.')
    (DIR/'pan-mechanics.json').write_text(json.dumps(out,indent=2)+'\n')
    print('Shop mass',out['movingShopStockKg'],out['fixedShopStockKg'],'drive',out['driver'],'clashes',rows)
if __name__=='__main__':study(Path(sys.argv[1]))
