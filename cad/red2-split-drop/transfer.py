"""Freeze the real R1/R2 source geometry and the matched partial gravity trial."""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/split-drop-native'
def read(file):return json.loads(file.read_bytes())
def write(file,value):file.write_text(json.dumps(value,indent=2))
def ref(file):return dict(file=file.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(file.read_bytes()).hexdigest())

def prepare(rev,plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import bounds
    from showcase_contacts import load_solids
    issued=read(OUT/(rev+'-load-01.json'))
    loaded=read(OUT/(rev+'-load-02.json'))
    source=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    if loaded['authority']['archiveSha256']!=source['sha256'] or ref(ROOT/source['file'])!=source:
        raise ValueError('Changed registered complete Fusion design')
    if ref(OUT/('red2-split-drop-fixture-'+rev+'.f3d'))['sha256']!=loaded['sha256'] or loaded['sha256']!=issued['sha256']:
        raise ValueError('Changed fixture archive')
    step=ROOT/issued['step']['file']
    if ref(step)['sha256']!=issued['step']['sha256']:raise ValueError('Changed installed derivative')
    # Reuse the exact same piece STEP bytes and registered position for both tests.
    piece=OUT/'r1-load-02-coral-A.step'
    if ref(piece)['sha256']!=read(OUT/'r1-load-02.json')['loadedWitness']['pieceStep']['sha256']:
        raise ValueError('Changed official-dimension piece')
    shapes,non_solids=load_solids(step)
    home=loaded['native']['states'][0]['bodies']
    mapping={}
    for solid,shape in shapes.items():
        box=bounds(shape)
        hits=[key for key,value in home.items() if max(abs(u-v) for u,v in zip(value,box))<.025]
        if len(hits)!=1:raise ValueError('No unique native/STEP fixture match for '+solid+' '+str(hits))
        mapping[solid]=hits[0]
    if non_solids or len(set(mapping.values()))!=len(home):raise ValueError('Incomplete fixture derivative')
    piece_shapes,piece_surfaces=load_solids(piece)
    if len(piece_shapes)!=1 or piece_surfaces:raise ValueError('Incomplete finite piece')
    pkey=next(iter(piece_shapes))
    center=read(OUT/'r1-gamepiece-control.json')['scenarios']['nominal']['originMm']
    drop=center[2]-570
    elapsed=math.sqrt(2*drop/9806.65)
    # At 110+travel the inside face reaches the measured end at 150.8125.
    # Assume a symmetric, constant 600 mm/s immediately around loss of support.
    # This is a prescribed comparative actuator challenge, not motor telemetry.
    loss=150.8125-110
    speed=600
    opening=loss/speed
    full=(90-loss)/speed
    def scene(left,right,z):
        result={}
        for key,name in mapping.items():
            if name.startswith('Longitudinal carriage:'):result[key]=[404,0,0]
            if name.startswith('Left splitting support:'):result[key]=[404,left,0]
            if name.startswith('Right splitting support:'):result[key]=[404,-right,0]
        result['coral-A']=[0,0,z-570]
        return result
    def frame(t,c,left,right,z):return dict(timeS=t,custody=c,offsetsMm=scene(left,right,z))
    holder=next(k for k,v in mapping.items() if v.startswith('Left splitting support:'))
    basis='Native fixture world positions and the station acquisition seat. Prescribed symmetric jaw speed near release. Gravity from rest, zero horizontal travel, no contact guide. Partial corridor ends before field support.'
    cases=[]
    for tag,duty,frames in [
        ('transfer','delivery',[frame(0,'held',0,0,center[2]),frame(opening,'free',loss,loss,center[2]),
            frame(opening+full,'free',90,90,center[2]-.5*9806.65*full**2),frame(opening+elapsed,'free',90,90,570)]),
        ('jam','jam-reset',[frame(0,'held',0,0,center[2]),frame(90/speed,'jammed',0,90,center[2])])]:
        name=rev+'-'+tag
        trace=dict(case=name,source=dict(native=ref(OUT/(rev+'-load-02.json')),trajectory=ref(OUT/'r1-gamepiece-control.json')),
                   holder=holder,pieceId='coral-A',pieceSha256=ref(piece)['sha256'],stepSha256=ref(step)['sha256'],
                   stockIds=[],frames=frames,resetObserved=False)
        file=OUT/(name+'-trace.json');write(file,trace)
        # These bounds delimit the terminal witness measurement, not a receiving target.
        cases.append(dict(id=name,duty=duty,trace=ref(file),admissionGapMm=.15,minGapMm=0,maxDepth=3,
                          windowS=[frames[0]['timeS'],frames[-1]['timeS']],terminalBoundsMm=[500,-160,490,690,160,680]))
    study=dict(schema='agent-frc/loaded-transfer@1',id=rev+'-nominal-pipe',basis=basis,gravityMmS2=9806.65,
               traceToleranceMm=.002,step=ref(step),piece=dict(id='coral-A',file=ref(piece),solid=pkey),stock=[],cases=cases)
    write(OUT/(rev+'-transfer-study.json'),study)
    write(OUT/(rev+'-derivative-fixture-map.json'),dict(archive=loaded['sha256'],step=ref(step),native=ref(OUT/(rev+'-load-02.json')),solids=mapping))

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('plugin',type=Path)
    args=parser.parse_args()
    for r in ('r1','r2'):prepare(r,args.plugin.resolve())
