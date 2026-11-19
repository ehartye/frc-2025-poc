"""Compare actual clocks and summarize four-site loads without converting acquisition into score."""
import argparse
import numpy as np
from pathlib import Path
import hashlib
import json
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
def read(f):return json.loads(f.read_bytes())
def write(f,d):f.write_text(json.dumps(d,indent=2,allow_nan=False)+"\n",encoding='utf8',newline='\n')
def digest(f):return hashlib.sha256(f.read_bytes()).hexdigest()

def analyze(assets):
    pins=read(ROOT/'fixed-hoist/lock-transfer.json')
    pose=read(ROOT/'conditioned-match/opening.json')
    p=np.asarray(read(HERE/'boundary.json')['positionsMm'])/1000
    v=np.column_stack((np.ones(4),p[:,0],p[:,1],p[:,0]*p[:,1]))
    nodal=np.linalg.inv(v)
    summary=[];draw={}
    for group in ('source','refine','limits','series'):
        records=read(HERE/(group+'.json'))
        for r in records['cases']:
            # Express the two original pin stresses with the same cantilever and diameter.
            row={k:r[k] for k in ('approach','trial','case','sourceRobotMmDeg','stepS','lastState','firstBandJavaMs','lastJavaMs',
                'peakSurfaceNormalN','damperStrokeMm','trajectorySha256','portTransitions')}
            if r['approach']=='powered-dock' and r['sourceRobotMmDeg']!=pose[r['case']]['sourceRobotMmDeg']:raise ValueError('Wrong approach')
            if 'landHistorySha256' in r:
                asset=assets/r['approach']/r['trial']/r['case']/f"us-{r['stepS']*1e6:g}"/'land-reactions.csv'
                if digest(asset)!=r['landHistorySha256']:raise ValueError('Different native-clock trace')
                t=np.loadtxt(asset,skiprows=1,delimiter=',')
                if not (len(t)>10 and np.all(np.diff(t[:,0])>0)):raise ValueError('Nonmonotonic histories')
                sites=t[:,1:5];pins_n=t[:,7:9];brackets=t[:,9:13]
                row.update(pinPeakN=r['maxAbsOldPinForceN'],pinPeakBendingMPa=(32*np.abs(r['maxAbsOldPinForceN'])*18/(np.pi*8**3)).tolist(),
                    siteDownMm=np.max(1000*sites,axis=0).tolist(),largestGuideLiftMm=float(-np.min(r['guidesRelativeRangeMm'][0])),
                    topStopTicks=r['topStopContactTicks'],damperBaseTravelMm=r['damperBaseTravelMm'],
                    firstPulseAbsBracketLoadN=np.max(abs(brackets),axis=0).tolist(),landSha256=r['landHistorySha256'])
                # The pockets run X−71…−19 about X−70…−20 masts (1 mm fore/aft)
                # and have their upper edge 47.5 mm above the pin axis. A
                # perfectly centered nominal shoe begins contact when this gap is consumed.
                directions=np.array([[0,1,0, yy] for yy in (.2225,-.2225)])@nodal
                pitch=sites@directions.T
                row['shoeForeAftClosingMmFromVerticalTilt']=np.max(abs(pitch)*47.5,axis=0).tolist()
                if r['trial']=='bracket' and group=='source':
                    # Group apertures retain extrema of every independently plotted
                    # signal in the timeline rather than sampling away the initial impulse.
                    keep=set([0,len(t)-1])
                    for ids in np.array_split(np.arange(len(t)),60):
                        for col in (1,2,5,6,7,8):
                            keep.add(int(ids[t[ids,col].argmin()]));keep.add(int(ids[t[ids,col].argmax()]))
                    chart=t[sorted(keep)][:,[0,1,2,5,6,7,8]]
                    chart[:,0]=(chart[:,0]-t[0,0])*1000;chart[:,1:5]*=1000
                    draw[r['approach']+'/'+r['case']]={'impactLipS':float(t[0,0]),'data':np.round(chart,4).tolist(),
                        'java':r['lastState'],'firstBandMs':r['firstBandJavaMs'],'pinPeakN':r['maxAbsOldPinForceN']}
            summary.append(row)
    write(HERE/'comparison.json',{'hashOfPriorPinWitness':digest(ROOT/'fixed-hoist/lock-transfer.json'),
        'publishedCantileverBaselineMPa':[max(x['bilateralVerticalCouple'][s]['nominalPinBendingMPa'] for s in ('left','right')) for x in pins['cases'].values()],
        'cases':summary,'physicalDatum':None})
    tmpl=(HERE/'chart.template.html').read_text(encoding='utf8')
    (HERE/'chart.html').write_text(tmpl.replace('/*DATA*/',json.dumps(draw,separators=(',',':'))),encoding='utf8',newline='\n')
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('assets',type=Path);analyze(p.parse_args().assets)
