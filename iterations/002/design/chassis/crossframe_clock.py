"""Exercise the actual station clocks with the 244-derived chassis proposal and explicit mass alternatives."""
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile

HERE=Path(__file__).parent
ROOT=HERE.parents[3]
STRATEGY=HERE.parent/'strategy'
sys.path.insert(0,str(STRATEGY))
import clocked_bus


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def run(options=None,module_half=(.282,.277)):
    ledger=HERE/'crossframe-budget.json'
    proposal=json.loads(ledger.read_bytes())['studies']['pocketedAluminum']
    trace=STRATEGY/'pan-on-match-clock.json'
    envelope=STRATEGY/'drive-envelope.json'
    cases={}
    design_mass=proposal['fieldKg']
    geometric_i=proposal['yawInertiaKgM2']
    # The second case tests a complete independently enforceable lower allocation
    # including all smaller transfer joints. Nominal/adverse tire friction is kept.
    # The nominal inertia and CG of this case are requirements, not properties already realized.
    if options is None:
        options={'full-grid':(design_mass,geometric_i,proposal['estimatedCgMm']),
             'limited-frame':(65,7.2,[30,15,250])}
    with tempfile.TemporaryDirectory() as folder:
        scratch=Path(folder)
        for which,(mass,inertia,cg) in options.items():
            revised=json.loads(envelope.read_bytes())
            normals={}
            revised['moduleHalfSpacingM']=module_half
            half_x,half_y=module_half
            for condition,r in revised['cases'].items():
                inputs=r['inputs'];inputs['mass']=mass;inputs['yawInertia']=inertia
                g=9.80665
                # Rigid body, four equal vertical wheel compliances, diagonal acceleration bound.
                imbalance=mass*g/4*(abs(cg[0])/(half_x*1000)+abs(cg[1])/(half_y*1000))
                dynamic=mass*inputs['limits'][1]*(cg[2]/1000)/4*math.hypot(1/half_x,1/half_y)
                min_n=mass*g/4-imbalance-dynamic; max_n=mass*g/4+imbalance+dynamic
                r['maxLoadedNormalN']=max_n; r['minLoadedNormalN']=min_n
                normals[condition]=dict(minimumN=min_n,maximumN=max_n,minimumTractionN=inputs['mu']*min_n)
            modified=scratch/(which+'-inputs.json');modified.write_text(json.dumps(revised))
            output=scratch/(which+'.json')
            clocked_bus.run(ROOT,modified,trace,output)
            raw=output.with_suffix('.csv')
            summary=json.loads(output.read_bytes());traction={}
            with raw.open() as f:
                for row in csv.DictReader(f):
                    key=row['case']+' / '+row['designSchedule']
                    value=normals[row['case'].split('/')[0]]['minimumTractionN']-float(row['wheelN'])
                    if key not in traction or value<traction[key]['marginN']:
                        traction[key]=dict(marginN=value,wallS=float(row['wallS']),phase=row['chassisPhase'])
            cases[which]=dict(fieldKg=mass,moduleHalfSpacingM=module_half,estimatedYawInertiaKgM2=inertia,cgMm=cg,loadedNormals=normals,
                              traction=traction,branchResults=summary['cases'],rawTimedCsvSha256=digest(raw))
    return dict(budgetSha256=digest(ledger),roleTraceSha256=digest(trace),originalDriveEnvelopeSha256=digest(envelope),cases=cases,
        limits='Schedule is accepted only if positive simultaneous voltage headroom, traction and timing remain on both matched trips for that condition. Actual pan trace is a carried original load on the same clock and still exceeds the nominal 30 A cap in some static holds. Six occupied robots, tracking, tread slip, strong impacts and recognized scores remain separate. A mass allocation cannot qualify unmodeled joints or hardware; the 70.39 kg grid must be lightened on inspection grounds regardless of drive capability.')

if __name__=='__main__':
    result=run();(HERE/'crossframe-clock.json').write_text(json.dumps(result,indent=2)+'\n')
    for option,r in result['cases'].items():
        print(option,'traction',round(min(v['marginN'] for v in r['traction'].values()),2))
