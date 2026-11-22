"""Budget the weight, CG, wheel reactions and yaw inertia of the proposed 002 load path."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

from perimeter_elastic import corners

BASE=Path(__file__).parent
ROOT=BASE.parents[3]
LB=.45359237
FT=.3048
G=9.80665


def file_hash(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def budget(plug_density, pocket):
    items=[]
    def article(name, mass, position, dimensions, basis, group='robot'):
        x,y,z=position
        dx,dy,dz=dimensions
        items.append(dict(name=name,kg=mass,cgMm=[x,y,z],group=group,basis=basis,
                          estimatedOriginYawInertiaKgM2=mass*(x*x+y*y+(dx*dx+dy*dy)/12)*1e-6))
    for x in (-282,282):
        for y in (-277,277):
            article('SDS NEO L2 kit including two NEOs, encoder, plate, wire and screws',6*LB,[x,y,100],[205,175,150],
                'Original supplier catalog 6.0 lb/module. Kingpin-centered CG and uniformly filled extents bound used only as nominal approximation; centroid and internal mass distribution remain unqualified.')
    rails=json.loads((BASE/'socket-holes.json').read_bytes())['rails']
    for name,rail in rails.items():
        lo,hi=rail['minimumMm'],rail['maximumMm']
        dims=[b-a for a,b in zip(lo,hi)]
        length=max(dims)
        article(name,.809*LB/FT*length/1000,[(a+b)/2 for a,b in zip(lo,hi)],dims,'Full ASTM B221 stock cut weight from Alro, before holes; two actual rails, two projected')
    pts=corners()
    for i,(a,b) in enumerate(zip(pts,pts[1:]+pts[:1])):
        length=math.dist(a,b)/1000
        article(f'miter-ring segment {i}',1.430*LB/FT*length,[(a[0]+b[0])/2,(a[1]+b[1])/2,152.4],[abs(a[0]-b[0]),abs(a[1]-b[1]),101.6],'1 x 4 x .125 supplied stock weight on actual nominal centerline, excluding local holes')
    # Compare a complete mass allowance, even though only one member is dimensionally trialled.
    for x in (-100,0,100):
        article('continuous crossbeam',1.397*LB/FT*.6792,[x,0,139.7],[50.8,679.2,76.2],'2 x 3 x .125 listed stock weight')
    for y in (-100,0,100):
        for lo,hi in ((-344.6,-125.4),(-74.6,-25.4),(25.4,74.6),(125.4,344.6)):
            article('segmented longitudinal tie',1.397*LB/FT*(hi-lo)/1000,[(lo+hi)/2,y,139.7],[hi-lo,50.8,76.2],'Crossbeam occupies intersections; requires eighteen structural interior splices. Interconnection design open.')
    # Four end screws and six staggered side screws per end. Solid stock allowances include unused volumes.
    volume=44.45*69.85*76.2
    if pocket:
        volume-=(44.45-2*10)*65*(76.2-23)
    for face in ('side','end'):
        for station in (-100,0,100):
            for sign in (-1,1):
                x,y=(station,sign*302) if face=='side' else (sign*307,station)
                article('captured end block',volume*plug_density*1e-9,[x,y,139.7],[44.45,76.2,69.85],'Cut stock, nominal rectangular pocket with 10 mm side lugs and 23 mm closed end. No final load-qualified drawing.')
                # An 82x70x4 mm backer plus four annular locating sleeves from steel.
                washer=(82*70*4 + 4*(math.pi/4*(14**2-6.7**2)*10))
                article('steel internal spreader and four positive shear sleeves',washer*7850e-9,[x,y,139.7],[82,5,70],'Gross envelope before drilling plate; actual retention and readback pending')
                article('ten purchased end and side screws plus load washers',.18,[x,y,139.7],[54,70,64],'180 g station receipt limit. Reweigh from exact installed hardware; no mass credit from missing items.')
    article('18 grid splices, four rail ties, six miter splice joints and fastening allowance',2.80,[0,0,142],[500,500,100],'New drawing allowance of 100 g per connection, conditional; it covers no load capacity')
    article('battery and registered lead',6.17,[-150,0,118],[190,80,170],'Prior issued field battery bound, proposed new receiving location','battery')
    article('non-module robot electronics, eight controllers, radio, power distribution, main leads, sensors and panel',7,[0,0,195],[350,300,80],'Commissioned complete electronics/loom/panel allowance, independent of module-included harness')
    article('protected pan drive, formed tray, guides, bearings, real supports and attached cables',8.2,[220,0,540],[445,460,260],'Design allocation. Prior fixed and moving fabricated stock alone weighs 6.497 kg, excluding purchased drive, supports and complete cable; new hardware is unissued.')
    article('miscellaneous build allowance',1.5,[0,0,210],[400,400,150],'Explicit remaining finish and overlooked installed hardware')
    article('removable bumper kit, one alliance color installed',7.5,[0,0,137],[0,0,138],'Historical upper field allowance for comparable section; proposed six backing/foam/cover segments, clamps and numbers require a new mass. Ring and fixed hardware above stay with robot.','bumper')
    # Preserve perimeter mass distribution in yaw (the cover/foam are absent at centre).
    items[-1]['estimatedOriginYawInertiaKgM2']=7.5*(.408**2+.404**2)/2*4/3
    article('one CORAL',.816466,[240,0,550],[114.3,301.6,114.3],'Historical article value; maximum held count one','pipe')
    m=sum(r['kg'] for r in items)
    cog=[sum(r['kg']*r['cgMm'][j] for r in items)/m for j in range(3)]
    yaw=sum(r['estimatedOriginYawInertiaKgM2'] for r in items)-m*(cog[0]**2+cog[1]**2)*1e-6
    bare=sum(r['kg'] for r in items if r['group']=='robot')
    return dict(items=items,fieldKg=m,bareKg=bare,bareWithBumpersKg=bare+7.5,estimatedCgMm=cog,yawInertiaKgM2=yaw,
        fieldAllocationRemainingKg=65-m,inspectionBareLimitMarginKg=52.16312255-bare,bumperInspectionMarginKg=61.23496995-(bare+7.5))


def main():
    p=argparse.ArgumentParser();p.add_argument('plugin',type=Path);a=p.parse_args()
    import sys
    sys.path.insert(0,str(a.plugin/'scripts'))
    from fusion_authority import register_path,verify
    register=register_path(ROOT)
    authority=verify(ROOT,register,register.parent/'issue-002/lineage.json')
    works={'pocketedAluminum':budget(2700,True),'fullAluminum':budget(2700,False),'fullSteel':budget(7850,False)}
    timed=ROOT/'iterations/002/design/strategy/timed-bus.json'; prior=ROOT/'artifacts/red2-integrated/pan-mechanics.json'
    duty=json.loads((BASE/'bumper-duty.json').read_bytes())
    for which,case in works.items():
        m=case['fieldKg'];v=duty['routine']['freeApproachMps']
        scale=.5*m*v*v/duty['routine']['energyJ']
        case['weakPackDockKineticJ']=duty['routine']['energyJ']*scale
        case['cruiseDesignJ']=.5*m*1.8**2
        case['conditionalCruiseForceNAt18mm']=2*case['cruiseDesignJ']/.018
    receipt=dict(authority=authority,receipts={str(p.relative_to(ROOT)):file_hash(p) for p in (timed,prior,BASE/'through-frame-paths.json',BASE/'bumper-duty.json')},studies=works,sourceUrls={
        'module':'https://www.swervedrivespecialties.com/products/mk4i-swerve-module',
        'stock':'https://www.alro.com/Resources/WebResources/AlroCom/MetalsGuideCatalog/PDFs/006-Aluminum.pdf',
        'socketScrew':'https://shop.boeing.com/cpd/bdsis4_91251a542'},
        scope='Mass-budget comparison before purchasing or authoring the whole six-member grid. All unissued system allocations, uncertain complete joints and component CGs remain visible. Rear modules are still projections. Uniform extents estimate yaw, not a supplier inertia or exact four-wheel reaction. For a broad class of offsets, a torque bound on actual wheel reactions and a timed branch-current replay are recorded separately.')
    (BASE/'crossframe-budget.json').write_text(json.dumps(receipt,indent=2)+'\n')
    for k,r in works.items():print(k,round(r['fieldKg'],3),round(r['bareKg'],3),[round(o,2) for o in r['estimatedCgMm']],round(r['yawInertiaKgM2'],3))

if __name__=='__main__':main()
