"""Charge measured stock and visible component uncertainties to the low-line frame's actual two trip clocks."""
import json
import math
from pathlib import Path
from crossframe_clock import run as replay, digest

HERE=Path(__file__).parent
def run():
    budget=HERE/'crossframe-budget.json'; shape=HERE/'low-line-frame.json'
    heavy=json.loads(budget.read_bytes())['studies']['pocketedAluminum']
    low=json.loads(shape.read_bytes())
    common_names={'SDS NEO L2 kit including two NEOs, encoder, plate, wire and screws',
        'Port longitudinal socket rail','Starboard longitudinal socket rail','Front transverse socket rail','Rear transverse socket rail','battery and registered lead',
        'non-module robot electronics, eight controllers, radio, power distribution, main leads, sensors and panel',
        'protected pan drive, formed tray, guides, bearings, real supports and attached cables','miscellaneous build allowance',
        'removable bumper kit, one alliance color installed','one CORAL'}
    kept=[r.copy() for r in heavy['items'] if r['name'] in common_names or r['name'].startswith('miter-ring segment ')]
    low_mass=low['currentFieldAllocationKg']
    structural=low_mass-sum(r['kg'] for r in kept)
    def point_assembly(m,z,radius,basis):
        return dict(name=basis,kg=m,cgMm=[0,0,z],estimatedOriginYawInertiaKgM2=m*(radius/1000)**2)
    # Separate real spatial domains; do not place robot-perimeter hardware at centre for inertia.
    center_tube=1.397*.45359237/.3048*.6284
    web_area=(628.4+638.4)*230-230**2
    top=web_area*3.175*2680e-9
    profile=low['addedMassKg']
    distributed=[
        point_assembly(center_tube,127,math.hypot(628.4,76.2)/math.sqrt(12),'center throughmember'),
        point_assembly(top,154,210,'upper cross-shaped sheet'),
        point_assembly(profile['eightVerticalWebAndBottomReturnsKg'],126,260,'eight formed arm edges'),
        point_assembly(profile['twoLowerCollectorsKg']+profile['twoCenterWebsKg'],118,100,'central collectors and webs'),
        point_assembly(profile['fourEndTubesKg'],139.7,340,'four end tubes')]
    remainder=structural-sum(v['kg'] for v in distributed)
    distributed.append(point_assembly(remainder,140,355,'symmetric structural interfaces, shoulders, bolts, spacers and splice allowances; edge-zone radius'))
    baseline=kept+distributed
    def properties(items):
        mass=sum(r['kg'] for r in items)
        cg=[sum(r['kg']*r['cgMm'][j] for r in items)/mass for j in range(3)]
        yaw=sum(r['estimatedOriginYawInertiaKgM2'] for r in items)-mass*(cg[0]**2+cg[1]**2)*1e-6
        return dict(massKg=mass,cgMm=cg,yawKgM2=yaw)
    nominal=properties(baseline)
    # Finite allowances. Keep the actual area masses for all other parts.
    uncertain=[r.copy() for r in baseline]
    def shift(match,pos):
        row=next(r for r in uncertain if r['name'].startswith(match));q=row['cgMm']
        row['estimatedOriginYawInertiaKgM2']+=row['kg']*((pos[0]**2+pos[1]**2)-(q[0]**2+q[1]**2))*1e-6
        row['cgMm']=pos
    shift('protected pan',[300,25,650]);shift('battery',[-100,35,135])
    shift('non-module robot electronics',[35,45,245])
    extras=[('joint and bracket stock over current allowance',1.25,[285,170,170]),
            ('unissued upper allowance excess',.50,[300,100,660]),
            ('loom and enclosure allowance excess',.25,[200,200,220])]
    for label,m,pos in extras:
        uncertain.append(dict(name=label,kg=m,cgMm=pos,estimatedOriginYawInertiaKgM2=m*(pos[0]**2+pos[1]**2)*1e-6))
    adverse=properties(uncertain)
    scenarios={k:(v['massKg'],v['yawKgM2'],v['cgMm']) for k,v in [('low-line-budget',nominal),('heavy-offset-control',adverse)]}
    results=replay(scenarios)
    duty=json.loads((HERE/'bumper-duty.json').read_bytes())
    for key,case in results['cases'].items():
        m=case['fieldKg'];case.update(conditionalLowSpeedEnergyJ=m*duty['routine']['freeApproachMps']**2/2,
            conditionalCruiseEnergyJ=m*1.8**2/2,conditional18mmCruiseForceN=m*1.8**2/.018,
            bareRuleHeadroomKg=52.16312255-(m-.816466-6.17-7.5),fieldPlanningTargetDifferenceKg=m-65.)
    return dict(frameDesignSha256=digest(shape),predecessorBudgetSha256=digest(budget),
        newStructureKg=structural,remodeledStructuralItems=distributed,nominal=nominal,heavyOffset=adverse,
        uncertaintyTest=extras,clock=results,scope='Nominal is a prebuild allocation and position model. The heavy offset carries a further 2.0 kg at explicitly named sites while moving the battery, upper and electronics within the stated experimental bounds. Mass, CG and inertia are still estimated. Peak tractive margin uses diagonal acceleration plus measured XY offset at every entry. Compare against the 65 kg planning objective, 52.163 kg bare rule and 61.235 kg robot-with-bumpers rule separately. Trace timing, current and voltage remain conditional on preserving the role load, steering and pan capabilities.')

if __name__=='__main__':
    r=run();(HERE/'low-line-frame-clock.json').write_text(json.dumps(r,indent=2)+'\n')
    print(r['nominal'],r['heavyOffset'])
    for key,v in r['clock']['cases'].items():
        print(key,v['bareRuleHeadroomKg'],min(t['marginN'] for t in v['traction'].values()),min(t['worst']['minimumBusV']['value'] for t in v['branchResults'].values()),v['conditional18mmCruiseForceN'])
