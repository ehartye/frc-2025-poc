"""Bound selected pan drive and compare simultaneous loaded feed, empty return and camera holds."""
import csv
import json
import math
from pathlib import Path

import numpy as np

from output_mount import ROOT,digest,read


def run():
    d=ROOT/'artifacts/red2-integrated'
    shop=read(d/'pan-mechanics.json')
    duty=read(d/'single-pivot-spatial-rim-installed-reactions-duty.json')
    reaction=read(d/'single-pivot-spatial-rim-installed-reactions.json')
    original=read(d/'single-pivot-spatial-rim.json')
    assert len(duty['runs'])==len(reaction['runs'])==len(original['runs'])==40
    for key,r in duty['runs'].items():
        assert {k:v for k,v in r.items() if k not in ['loadPeaks','duty']}==original['runs'][key]
        assert all(r['loadPeaks'][k]==v for k,v in reaction['runs'][key]['loadPeaks'].items())
    stock=read(d/'pan-shop-plan.json')
    assert digest(ROOT/stock['authority']['file'])==original['sourceSha256']==stock['authority']['sha256']
    assert digest(d/'red2-integrated-foundation-r2.f3d')==duty['foundationSha256']
    f=d/'single-pivot-rim-field'
    clocks={r[0]:r for r in csv.reader((f/'assessment-clock.csv').open())}
    cases=list(csv.DictReader((f/'post-impact-reset.csv').open()))
    assert len(cases)==120 and set(clocks)=={r['case'] for r in cases}
    ratio=(68/13)*(76/21)*(84/29)
    m=shop['movingShopStockKg'];I=shop['movingStockInertiaKgM2']
    dx=shop['movingShopCentreMm'][0]/1000-.205
    dz=shop['movingShopCentreMm'][2]/1000-.580
    rows={}
    for row in cases:
        name=row['case']
        source=name.rsplit('/',1)[0]
        r=duty['runs'][source]
        pitch=float(clocks[name][1])
        free=float(clocks[name][3])
        assert abs(free-r['free']['wallS'])<1e-7
        elapsed=float(row['elapsed_s'])
        finish=float(row['reset_wall_s'])
        start=finish-1.6-free if finish>=0 else None
        n=max(2,math.ceil(elapsed/.001))
        ts=np.linspace(0,elapsed,n+1)
        u=np.clip((ts-start)/1.6,0,1) if start is not None else np.zeros_like(ts)
        low=math.radians(-2)-pitch
        sweep=math.radians(70)-low
        angle=low+sweep*u**2*(3-2*u)
        acc=(sweep*(6-12*u)/(1.6**2))*((ts>=start)&(ts<start+1.6)) if start is not None else np.zeros_like(ts)
        rx=np.cos(angle)*dx-np.sin(angle)*dz
        rz=np.sin(angle)*dx+np.cos(angle)*dz
        net=m*9.80665*(rx*math.cos(pitch)-rz*math.sin(pitch))
        required=-I*acc-net
        # Bound the torque between 1-ms samples using |dT/dt|. At the two
        # acceleration discontinuities visit the moving side at u=0 and 1.
        lip= m*9.80665*math.hypot(dx,dz)*(1.5*sweep/1.6)+I*12*sweep/1.6**3
        peak=float(np.max(abs(required)))+lip*elapsed/n
        if start is not None:
            for theta,alpha in [(low,6*sweep/1.6**2),(math.radians(70),-6*sweep/1.6**2)]:
                rx0=math.cos(theta)*dx-math.sin(theta)*dz
                rz0=math.sin(theta)*dx+math.cos(theta)*dz
                peak=max(peak,abs(-I*alpha-m*9.80665*(rx0*math.cos(pitch)-rz0*math.sin(pitch))))
        cycles={};peakall=max(r['loadPeaks']['shaftDutyNm']['value'],peak)
        for drive,history in r['duty'].items():
            if history['torqueCurrentA']!=30:continue
            eta=history['efficiency']
            total=history['allRecordedS']+elapsed
            integral=history['integratedTorqueSquaredNm2s']+float(np.trapezoid(required**2,ts))
            cycles[drive]=dict(feedbackTorqueCurrentLimitA=history['torqueCurrentA'],efficiency=eta,
                calculatedPeakTorqueCurrentA=peakall/(.97/100*ratio*eta),
                calculatedWindowRmsTorqueCurrentA=math.sqrt(integral/total)/(.97/100*ratio*eta),
                feedToReleaseExcessNms=history['excessAngularImpulseNms'],feedToReleaseLongestExcessS=history['longestOverLimitS'],
                postReleaseExceeds=peak>history['shaftLimitNm'])
        rows[name]=dict(frameCase=source,pitchRad=pitch,releaseS=free,resetEndS=finish,sourceAfterReleaseSeconds=elapsed,
            resetMode='return' if start is not None else 'hold',afterReleaseUpperTorqueNm=peak,
            throughReleasePeak=r['loadPeaks']['shaftDutyNm'],projectedCurrent=cycles)
    samples=[]
    for metric in next(iter(reaction['runs'].values()))['loadPeaks']:
        value,key=max((r['loadPeaks'][metric]['value'],n) for n,r in reaction['runs'].items())
        samples.append(dict(metric=metric,case=key,**reaction['runs'][key]['loadPeaks'][metric]))
    inputs=['single-pivot-spatial-rim-installed-reactions-duty.json','single-pivot-spatial-rim-installed-reactions.json',
            'single-pivot-spatial-rim.json','pan-shop-plan.json','pan-mechanics.json','pan-drive-registration.json',
            'single-pivot-rim-field/assessment-clock.csv','single-pivot-rim-field/post-impact-reset.csv']
    rec=dict(authority=stock['authority'],foundation=duty['foundationSha256'],inputs={p:digest(d/p) for p in inputs},
        signedPeaks=samples,driveRatio=ratio,sourceMotor='REV-21-1651, 12 V, 0.97 N m at 100 A stall',
        rows=rows,maximumReturnTorqueNm=max(r['afterReleaseUpperTorqueNm'] for r in rows.values() if r['resetMode']=='return'),
        limits='Measured source scenarios and shop stock only. Current estimates use unverified gearbox efficiency, exclude no-load and current-loop delays, rotor/adapter inertia, case losses and thermal rise. Pipe remains on field once released. Two ideal radial supports do not qualify axial shoulder, motor holding action, crush, fatigue, friction or mounted chassis.')
    (d/'pan-duty-summary.json').write_text(json.dumps(rec,indent=2))
    print(len(rows),sum(r['resetMode']=='return' for r in rows.values()),rec['maximumReturnTorqueNm'])


if __name__=='__main__':run()
