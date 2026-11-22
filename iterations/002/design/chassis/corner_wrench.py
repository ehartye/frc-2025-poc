"""Evaluate the actual six screw axes for wheel duty and bounded finite-stop bumper loads."""
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

D=Path(__file__).resolve().parents[1]
read=lambda f:json.loads(f.read_bytes())
sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()


def wrench(datum, force, center):
    v=np.array(datum)-center
    return np.cross(v,force)


def evaluate(centers, rails, application, force, strength):
    # The planar, equal-stiffness group is an analysis assumption. Preload
    # friction is not credited. Through-wall sharing is evaluated two ways.
    mid=np.mean(centers,axis=0)
    center=np.array([*mid,69.2333628767])
    moment=wrench(application,force,center)
    arm=centers-mid
    perp=np.column_stack((-arm[:,1],arm[:,0]))
    shear=np.array(force[:2])/len(centers)+moment[2]*perp/np.sum(arm*arm)
    normal=np.vstack((np.ones(len(centers)),arm[:,1],-arm[:,0]))
    demanded=np.array([force[2],moment[0],moment[1]])
    axial=np.linalg.lstsq(normal,demanded,rcond=None)[0]
    residual=normal@axial-demanded
    reactions=[]
    for i,(q,r) in enumerate(zip(shear,rails)):
        j=r['lengthAxis']
        # Determine correct material in the direction of bolt travel. The far
        # end is irrelevant to a load pointing into the middle of the tube.
        e=(r['maximumMm'][j]-centers[i,j]) if q[j]>=0 else (centers[i,j]-r['minimumMm'][j])
        ligament=e-2.65-.25-.15-.05
        walls=[]
        for n in (1,2):
            t=3.0*n
            # Derived conservative first-yield and simple projected area screens.
            capacity={'axialTearoutN':2*t*ligament*strength/math.sqrt(3)/2,
                      'holeBearingN':4.826*t*strength/2,
                      'transverseNetSectionN':(25.4-5.40)*t*strength/2}
            util=max(abs(q[j])/capacity['axialTearoutN'],np.linalg.norm(q)/capacity['holeBearingN'],abs(q[j])/capacity['transverseNetSectionN'])
            walls.append(dict(engagedWalls=n,capacity=capacity,maxUtilization=util))
        reactions.append(dict(rail=r['name'],axisXYmm=centers[i].tolist(),forceOnRailXYN=q.tolist(),verticalOnRailN=axial[i],
                              effectiveDirectionalEdgeMm=ligament,wallCases=walls))
    return dict(applicationMm=application,forceN=force,momentAboutGroupNmm=moment.tolist(),residualFzMxMy=residual.tolist(),rows=reactions,
                worstOneWall=max(v['wallCases'][0]['maxUtilization'] for v in reactions),
                worstTwoWalls=max(v['wallCases'][1]['maxUtilization'] for v in reactions))


def run():
    native=read(D/'cad/fusion-authority.json');receipt=read(D/'cad/issue-244.json')
    if native['source']['archive']!=receipt['master'] or sha(D.parents[2]/receipt['master']['file'])!=receipt['master']['sha256']:raise ValueError('Current file is not the assessed native')
    frame=read(D/'chassis/socket-holes.json')['rails']
    budgets=read(D/'strategy/drive-envelope.json')['cases']
    csvfile=D/'strategy/timed-bus.csv'
    with csvfile.open() as f:drive=max(csv.DictReader(f),key=lambda x:float(x['wheelN']))
    points=[];rails=[]
    for r in frame.values():
        for p,s in zip(r['boltCentersMm'],r['sources']):
            if s['station']=='FL':
                points.append(p);rails.append(r)
    xy=np.array(points)
    if len(xy)!=6:raise ValueError('Missing real corner bolt axes')
    cases={};wheel=float(drive['wheelN']);normal=max(b['maxLoadedNormalN'] for b in budgets.values())
    for angle in range(0,360,5):
        v=math.radians(angle);cases['wheel-'+str(angle)]=evaluate(xy,rails,[282,277,0],[wheel*math.cos(v),wheel*math.sin(v),normal],240.)
    # A conservative corner-level proposed connection. Off-axis point uses the front/side
    # outer plan intersection. Projection to any purely normal outer bumper face
    # does not change its yaw moment. Peak load assumes a linear force/stroke law.
    impact={}
    for condition,speed,mass in [('nominal',1.8,65),('adverse',1.7,67)]:
        for stroke in (.018,.040,.070):
            peak=mass*speed*speed/stroke
            for face,(fx,fy) in {'front':(-peak,0.),'side':(0.,-peak)}.items():
                for z in (114.3,150.0):
                    for sign in (1,-1):
                        impact[f'{condition}/{stroke}/{face}/z{z}/sign{sign}']=evaluate(xy,rails,[370,365,z],[sign*fx,sign*fy,0.],240.)
    rail=next(r for r in frame.values() if r['name']=='Front transverse socket rail')
    points=[p for p,s in zip(rail['boltCentersMm'],rail['sources']) if s['station']=='FR']
    partial=evaluate(np.array(points),[rail]*3,[282,-277,0],[0,0,normal],240.)
    worst=lambda cases,mode:max(cases,key=lambda k:cases[k][mode])
    return dict(authority=native['source']['archive'],synchronizedDutySha256=sha(csvfile),loadFactor=2.,
      material={'stock':'Specified 6061-T6 extrusion, minimum 240 MPa tensile yield, prewelded material. Requalify any heat affected joint. No HAZ strength credited in the reference case.',
      'publication':'Hydro 6061, 2019/01, p.2, standard T6 hollow extrusions','source':'https://www.hydro.com/globalassets/01-products--services/extruded-profiles/americas/ena-resources/alloy-data-sheets/hydro_2019_data_sheet_6061.pdf'},
      manufacturing={'nominalTubeWallMm':3.175,'minimumWallScreenMm':3.0,'sawToDrillMm':.25,'holeLocationMm':.15,'radialOversizeMm':.05,'nominalSmallestHoleLigamentMm':2.975,'adverseSmallestMm':2.525},
      maximumWheelDutyRow=drive,maximumLoadedNormalN=normal,wheelAngularCases=cases,impactCases=impact,
      peakWheel=worst(cases,'worstOneWall'),peakImpact=worst(impact,'worstOneWall'),actualFRCollinearPartial=partial,
      limits='Wheel/corner envelopes with planar rigid equal-stiffness screw group. Captured wheel peak combined with conservative peak normal; directions explored individually. Corner impact assumes the entire proposed joint takes the pointed bumper load; actual opposite-side transfer, wall contact, tire/battery relative drop, foam stroke, bumper bracket stiffness, aluminum bearing, joint slip, screw retention, vertical clamp and local bending need separate installed checks. Signed opposite force is a reverse direction control. A collinear group without the second rail cannot support arbitrary overturning.')
if __name__=='__main__':
    result=run();(D/'chassis/corner-wrench.json').write_text(json.dumps(result,indent=2)+'\n')
    print(result['peakWheel'],result['wheelAngularCases'][result['peakWheel']]['worstOneWall'])
    for key in ['nominal/0.04/front/z150.0/sign1','adverse/0.04/front/z150.0/sign1',result['peakImpact']]:
        a=result['impactCases'][key];print(key,a['worstOneWall'],a['worstTwoWalls'], max(abs(x['verticalOnRailN']) for x in a['rows']))
    print('residual',result['actualFRCollinearPartial']['residualFzMxMy'])
