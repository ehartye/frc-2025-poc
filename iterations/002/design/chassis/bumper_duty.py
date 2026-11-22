"""Bracket a routine delayed wall position and the independent high-energy impact design case."""
import csv,json,math
from pathlib import Path
from hashlib import sha256
p=Path(__file__).parent;d=p.parent
rows=[r for r in csv.DictReader((d/'strategy/timed-bus.csv').open()) if r['case'].startswith('adverse/coral-B') and r['designSchedule']=='extended-cruise']
moving=[i for i,r in enumerate(rows) if r['chassisPhase']=='drive'];i=moving[-1];end=i
while i and rows[i-1]['chassisPhase']=='drive':i-=1
last=rows[i:end+1];residual=0
for a,b in zip(last[-2::-1],last[:0:-1]):
    v0,v1=map(float,[a['wheelMps'],b['wheelMps']]);step=float(b['wallS'])-float(a['wallS'])
    leg=.5*(v0+v1)*step
    if residual+leg>=.100:
        x=.1-residual;acc=(v0-v1)/step;dt=(-v1+math.sqrt(v1*v1+2*acc*x))/acc
        impact_speed=v1+acc*dt;impact_time=float(b['wallS'])-dt;bound=[a,b];break
    residual+=leg
else:raise ValueError('Final actual drive leg does not have a 100 mm segment')
A=.1*.138;t=.0695;E=.5*67*impact_speed**2
# Sealed Air Ethafoam 220, September 2024. Piecewise linear interpolation,
# 25%/50% compression, no invented continuation of the published data.
pts=[(0.,0.),(.25*t,65000*A),(.5*t,125000*A)]
areas=[.5*(f+g)*(y-x) for (x,f),(y,g) in zip(pts,pts[1:])]
spent=0
for ((x,f),(y,g)),work in zip(zip(pts,pts[1:]),areas):
    if spent+work>=E:
        k=(g-f)/(y-x);load=math.sqrt(f*f+2*k*(E-spent));stroke=x+(load-f)/k;break
    spent+=work
upper=65*1.8**2/.018;peak_source=max([r for r in csv.DictReader((d/'strategy/timed-bus.csv').open()) if r['case'].startswith('nominal/coral-B')],key=lambda r:float(r['wheelMps']))
def section(b,h,t):return (h*b**3-(h-2*t)*(b-2*t)**3)/(6*b)
screens={}
for height in (25.4,50.8,76.2,101.6):
    W=section(25.4,height,3.0);span=331.-150.;overhang=365.-330.975
    result=dict(tubeMm=[25.4,height,3.175],minimumWallMm=3.,sectionModulusMm3=W,
        ratioStressYieldAt2xDesignLoadingStraight=upper*span/4/(120*W),
        ratioStressYieldAt2xDesignLoading35mmOffset=upper*overhang/(120*W),
        maxSpanMmAtUpper=4*120*W/upper,maxSpanMmAtRoutine=4*120*W/load,
        ratioStressYieldAt2xDesignLoading200mmMiddle=upper*200/4/(120*W),
        massFullOutlineNominalKg=(25.4*height-(25.4-6.35)*(height-6.35))*2.890794*.00270)
    screens[str(height)]=result
out=dict(csvSha256=sha256((d/'strategy/timed-bus.csv').read_bytes()).hexdigest(),
routine=dict(case=last[0]['case'],schedule=last[0]['designSchedule'],obstructionMmBeforeDock=100,actualBracket=bound,impactTimeS=impact_time,
    freeApproachMps=impact_speed,massKg=67,energyJ=E,foamFaceMm=[100,138],foamDepthMm=t*1000,quasistaticInterpolatedStopMm=stroke*1000,quasistaticInterpolatedPeakN=load,
    complete50PctWorkJ=sum(areas),sourceTableURL='https://dam.sealedair.com/asset/8abcf0fc-1b7a-47e1-bc2f-4a190360bfb4/PROT_Foam_Ethafoam_Brochure_EN_0924-pdf.pdf'),
upper=dict(case=peak_source,massKg=65,wall='immovable',effectiveStrokeMm=18,peakN=upper,energyJ=.5*65*1.8**2,
    limit='Independent triangular impact-force stress case using full scheduled cruise. Peak is conditional on 18 mm runout and that force-displacement curve. Foam measurements end at 50% and do not establish a dynamic peak. Contact from another moving machine and hard bottoming can exceed it.'),
candidateGeometry=dict(greatestStraightInternalSpanMm=200,lastAnchorToCornerCrossTieMm=181,cornerCantileverMm=34.025,beamBending=screens),
open='A positive supported path from both mitered outer members into both continuous rail webs must exist at the upper design load. Beam screens do not implicitly install the second rail, backing, bolted splice or tools, and require adequate reverse-direction retention and metal frame/truss joints.')
(p/'bumper-duty.json').write_text(json.dumps(out,indent=2)+'\n')
print(out['routine']['impactTimeS'],impact_speed,E,stroke*1000,load,sum(areas));print(upper,screens['101.6'])
