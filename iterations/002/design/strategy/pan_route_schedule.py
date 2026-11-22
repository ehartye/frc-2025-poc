"""Expand the existing synchronized Red 2 route into signed body-frame inertial events."""
import json
import math
import sys
from pathlib import Path

HERE=Path(__file__).parent
ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'experiments/six-wheel/scripts'))
import registered_transit as field
from drive_envelope import file_id
from clocked_bus import loaded_route


def run(stand_off=0.,module_half=(.282,.277),clock_file=None):
    src=HERE/'drive-envelope.json';environment=json.loads(src.read_bytes())['cases']
    current=clock_file or HERE.parent/'chassis/low-line-frame-clock.json'
    loaded=json.loads(current.read_bytes())['clock']['cases']['heavy-offset-control']['branchResults']
    result={}
    for condition,case in environment.items():
        peak,accel,yaw,turn_accel,rate,steer_accel=case['inputs']['limits']
        if condition=='adverse':peak=1.7
        field.DRIVE.update(maximumWheelMps=peak,maximumWheelAccelMps2=accel,maximumTurnRadS=yaw,turnWheelAccelMps2=turn_accel)
        for piece,meta in case['pieces'].items():
            cursor=meta['carryStartS'];orientation='drive';events=[]
            meta=dict(meta,finalStandOffM=stand_off)
            for seg in loaded_route(field,meta).segments:
                if seg['kind']!=orientation:
                    _,_,_,time=field.wheel_profile(math.atan2(*module_half),rate,steer_accel)
                    cursor+=time+.020
                angular=seg['kind']=='turn'
                a=math.copysign(seg['a'],seg['demand'])*(2/field.DRIVE['trackM'] if angular else 1.)
                t=cursor
                for j,duration in enumerate((seg['rise'],seg['coast'],seg['rise'])):
                    if duration>0:events.append(dict(start=t,end=t+duration,kind=seg['kind'],
                        accel=(a,0,-a)[j],initialSpeed=(0,a*seg['rise'],a*seg['rise'])[j]))
                    t+=duration
                cursor+=seg['dt'];orientation=seg['kind']
            k=next(k for k in loaded if k.startswith(f'{condition}/{piece}/') and
                   k.endswith('extended-cruise' if condition=='adverse' else 'baseline'))
            earlier=loaded[k]['plannedParkAtS']-cursor
            if (not stand_off and abs(earlier)>.000001) or (stand_off and not 0<earlier<.02):raise ValueError('Broken synchronization')
            result[condition+'/'+piece]=dict(events=events,parkS=cursor,advanceVsOriginalS=earlier,clock=meta,packV=case['inputs']['pack'],
                peakBusDemandLog=loaded[k]['worst'],panEfficiency=case['inputs']['panEfficiency'])
    return dict(envelopeSha256=file_id(src),chassisClockSha256=file_id(current),routeSolverSha256=file_id(Path(field.__file__)),cases=result)


if __name__=='__main__':
    stand_off=.012 if '--stand-off-12' in sys.argv else 0.
    r=run(stand_off);file='pan-route-12mm.json' if stand_off else 'pan-route-schedule.json'
    (HERE/file).write_text(json.dumps(r,indent=2)+'\n')

