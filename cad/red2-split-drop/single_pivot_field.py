"""Replay actual dump-pan exits against the season field; classify geometry-control outcomes."""
import argparse
import csv
import hashlib
import json
import math
import subprocess
import sys
from collections import Counter
from pathlib import Path
import numpy as np
from field_plane import rotation

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/red2-integrated'
FIELD=ROOT/'experiments/six-wheel/spatial-contact'


def replay(jdk,label):
    sys.path.insert(0,str(FIELD))
    from physics import panel_file
    spatial=label in ('spatial','rim')
    datafile=OUT/('single-pivot-spatial-'+('rim' if label=='rim' else 'challenge')+'.json' if spatial else 'single-pivot-'+label+'-acquisition.json')
    study=json.loads(datafile.read_bytes())
    casefile=ROOT/'experiments/swerve-drive/low-placement/results.json'
    boxes=json.loads(casefile.read_bytes())['cases']
    dest=OUT/('single-pivot-'+label+'-field');dest.mkdir(exist_ok=True)
    panel_file(json.loads((ROOT/'experiments/six-wheel/mission/target.json').read_bytes()),dest/'panels.csv')
    rows=[];states={}
    runs=study['runs' if spatial else 'results']
    if spatial and len(runs)!=40:
        raise ValueError('Incomplete signed spatial challenge')
    for name,result in runs.items():
        if spatial:
            case,piece=result['scenario'],result['id']
            pitch,step,q=result['pitch'],result['dt'],result['free']
            if not q:continue
            local_pos,local_vel,local_axis,local_spin=q['p'],q['v'],q['axis'],q['omega']
        else:
            case,piece,step,pstr=name.split('/')
            pitch=float(pstr);q=result['release']
            if not q or pitch==0:continue
            local_pos=[q['x'],.025 if case=='adverse' else .003,q['z']]
            local_vel=[q['vx'],0,q['vz']]
            local_axis=[0,1,0];local_spin=[0,q['omega'],0]
        registered=boxes['plus' if case=='nominal' else 'minus']['chassis']
        for i,(dx,dy,yaw) in enumerate([registered['terminalMmMmRad'],*registered['enclosure']]):
            c=np.array([(dx-490)/1000,dy/1000+(0 if piece=='coral-A' else .36),.005 if pitch>0 else -.005])
            mat=rotation(pitch,yaw)
            pos=mat@np.array(local_pos)+c
            vel=mat@np.array(local_vel)
            axis=mat@np.array(local_axis);spin=mat@np.array(local_spin)
            tag=name+'/'+str(i)
            rows.append([tag,*pos,*vel,*axis,*spin,c[0],c[1],yaw,-.001,2.5,float(step),17,.10 if case=='adverse' else .35])
            states[tag]=dict(originM=c.tolist(),rotation=mat.tolist(),freeWallS=q['wallS'],scenario=case,piece=piece,pitch=pitch)
    (dest/'source-frames.json').write_bytes((json.dumps(states,indent=2)+'\n').encode())
    with (dest/'cases.csv').open('w',newline='') as f:csv.writer(f,lineterminator='\n').writerows(rows)
    # Remove historic rack from this CONTROL. Presence of the actual tipping
    # tray must be evaluated with its own installed post-impact geometry.
    (dest/'old-rack-removed.csv').write_bytes(b't,withdraw_m\n0,.110\n.001,.110\n')
    subprocess.run([jdk/'bin/javac.exe','-d',dest,FIELD/'SpatialContact.java'],check=True)
    subprocess.run([jdk/'bin/java.exe','-cp',dest,'SpatialContact',dest/'panels.csv',dest/'old-rack-removed.csv',
                    dest/'cases.csv',dest/'result.csv',dest/'trace.csv','.005','2.5','sweep-y'],check=True)
    if spatial:
        clocks=json.loads((ROOT/'artifacts/split-drop-native/launch-inputs.json').read_bytes())['clocks']
        with (dest/'assessment-clock.csv').open('w',newline='') as f:
            writer=csv.writer(f,lineterminator='\n')
            for tag,state in states.items():
                clock=clocks[state['scenario']][state['piece']]
                writer.writerow([tag,state['pitch'],state['originM'][2],state['freeWallS'],clock.get('withdraw',-1),clock.get('stow',-1)])
        assessment=Path(__file__).with_name('PanContactStudy.java')
        subprocess.run([jdk/'bin/javac.exe','-cp',dest,'-d',dest,assessment],check=True)
        subprocess.run([jdk/'bin/java.exe','-Xmx512m','-cp',dest,'PanContactStudy',dest],check=True)
    with (dest/'result.csv').open() as f:out=list(csv.DictReader(f))
    if len(out)!=len(rows) or not rows:
        raise ValueError('Incomplete field control cohort')
    counts=Counter(row['outcome'] for row in out)
    (dest/'summary.json').write_text(json.dumps(dict(sourceStateSha256=hashlib.sha256(datafile.read_bytes()).hexdigest(),
        outcomes=counts,recognizedCompleteRobotScores=0),indent=2)+'\n',encoding='utf8')
    print(counts)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('jdk',type=Path)
    parser.add_argument('label',default='baseline',nargs='?')
    args=parser.parse_args()
    replay(args.jdk,args.label)
