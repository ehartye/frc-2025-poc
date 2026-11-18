import pathlib,json,math,hashlib,importlib.util,itertools,collections
ROOT=pathlib.Path(__file__).resolve().parents[1]
s=importlib.util.spec_from_file_location('runner',ROOT/'scripts/run-coral-startup.py');runner=importlib.util.module_from_spec(s);s.loader.exec_module(runner)
PINS=['scripts/coral-startup-report.py','scripts/run-coral-startup.py','case-studies/coral-startup-sensitivity.json','case-studies/coral-mass-evidence.json']
def parse(line):
 if not line.startswith('CORAL_SENSITIVITY '):raise ValueError('record required')
 h,r=line[len('CORAL_SENSITIVITY '):].split(' Result[');a=[v.split('=') for v in h.split()];b=[v.split('=') for v in r.removesuffix(']').split(', ')]
 if len(a)!=5 or len(b)!=5 or any(len(v)!=2 for v in a+b):raise ValueError('field count')
 d=dict(a+b)
 if set(d)!={'massKg','mu','pitchDegrees','spinupS','volts','outcome','elapsed','x','speed','residual'}:raise ValueError('field set')
 outcome=d.pop('outcome');d={k:float(v) for k,v in d.items()}
 if outcome not in ('EXIT','ROLLBACK','TIMEOUT') or not all(math.isfinite(v) for v in d.values()) or abs(d['residual'])>2e-5:raise ValueError('result domain')
 return dict(**d,outcome=outcome,frictionBasis='independent-hypothesis-not-manual',orientationBasis='guide-elevation-not-full-tube-pose',scoring=None)
def report():
 r=json.loads((ROOT/PINS[2]).read_text());e=json.loads((ROOT/PINS[3]).read_text())
 if set(r['sourceCodeSha256'])!=set(runner.CODE):raise ValueError('dependency set')
 for p,h in r['sourceCodeSha256'].items():
  if hashlib.sha256((ROOT/p).read_text(encoding='utf-8-sig').encode()).hexdigest()!=h:raise ValueError('stale native input')
 rows=[parse(l) for l in r['observations']];expected=set(itertools.product(e['massBoundsKg'],(.05,.2,.8),(0,15,30),(0,.5,2),(9,12)))
 if len(rows)!=108 or {(x['massKg'],x['mu'],x['pitchDegrees'],x['spinupS'],x['volts']) for x in rows}!=expected:raise ValueError('grid inventory')
 return dict(schema='frc-coral-startup-report/v1',runId=r['runId'],summary=dict(collections.Counter(x['outcome'] for x in rows)),cases=rows,sourceCodeSha256={p:hashlib.sha256((ROOT/p).read_text(encoding='utf-8-sig').encode()).hexdigest() for p in PINS},limitations=['Manual supplies mass endpoints and source variability, not friction or orientation/contact calibration','Startup study is separate from generic0.5kg catcher; no mid-transfer mass change','Guide elevation is0/15/30degrees, not arbitrary pipe yaw/roll or a finite nip geometry model','Prescribed15N per roller and friction0.05/0.2/0.8 are independent hypotheses','ROLLBACK is a bounded failure, not a rescued successful trajectory','No installed transform, scoring or hardware acceptance'],engineeringAcceptance='blocked')
if __name__=='__main__':
 import argparse
 a=argparse.ArgumentParser();a.add_argument('--output',required=True);p=pathlib.Path(a.parse_args().output)
 with p.open('x',encoding='utf-8') as f:json.dump(report(),f,indent=2,allow_nan=False);f.write('\n')
