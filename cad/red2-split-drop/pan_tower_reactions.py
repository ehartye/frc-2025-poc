"""Carry the simultaneous source-time pipe reactions into the R2 starboard tower's actual seat."""
import json
from pathlib import Path

import numpy as np

from output_mount import ROOT, digest, read


def study():
    d=ROOT/'artifacts/red2-integrated'
    source=d/'single-pivot-spatial-rim-installed-reactions.json'
    stock=d/'pan-shop-plan.json'
    first=d/'pan-native/foot-patches-r1.json'
    current=d/'pan-native/first-party-stock-r2.json'
    registered=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    if digest(ROOT/registered['file'])!=registered['sha256']:
        raise ValueError('Canonical assembly changed')
    r=read(source)['runs']
    patch=read(first)['patches']['Starboard bearing tower foot']
    if abs(patch['areaMm2']-1620.)>.1:raise ValueError('Changed tower support')
    footprint=patch['pressureCells'][0]['footprintBoundsMm']
    # The R2 stock is independently compared to the same original unchanged
    # foot profile. The 1620 mm² support cell screen is from R1.
    plan=read(stock)
    foot=next(v for v in plan['items'] if v['id']=='Starboard bearing tower foot')
    support=read(current)
    if not any('Starboard bearing tower foot' in str(v) for v in support.values()):
        raise ValueError('Unresolved unchanged foot stock')
    center=np.array(patch['centroidMm'])
    axis=np.array([205.,-195.775,580.])
    arm=(axis-center)/1000.
    edgeLever=(footprint[4]-footprint[1])/2000.
    rows={}
    for name,run in r.items():
        for demand,P in run['loadPeaks'].items():
            rx,rz=P['starboardRadialXZ_N']
            fy=P['pipeForceN'][1]
            force=np.array([-rx,fy,-rz])
            moment=np.cross(arm,force)
            # This bound permits any compression distribution over the full
            # seat, which is more optimistic than finite pressure cells.
            normal=max(0.,-force[2])
            maxSeatRoll=normal*edgeLever
            rows[name+' / '+demand]=dict(timeS=P['timeS'],role=run['id'],pipeDemandField=demand,phase=P['phase'],angleDeg=P['shopPhaseDeg'],
                forceToTowerN=force.tolist(),simultaneousFootMomentNm=moment.tolist(),upwardNetN=max(0.,force[2]),
                maximumCompressionOnlyRollNm=maxSeatRoll,rollBeyondSeatNm=max(0.,abs(moment[0])-maxSeatRoll),
                sourcePipeForceN=P['pipeForceN'],sourcePipeMomentNm=P['pipeMomentNm'])
    worst=lambda f:max(rows.items(),key=lambda p:f(p[1]))
    result=dict(authority=registered,sourceSha256=digest(source),sourceFootPatchSha256=digest(first),independentFootStockSha256=digest(current),
       capturedSourceTimes=len(rows),footCentroidMm=center.tolist(),shaftAxisAtStarboardRadialBearingMm=axis.tolist(),footprintBoundsMm=footprint,
       maximumTowerUplift=worst(lambda x:x['upwardNetN']),maximumShaftRoll=worst(lambda x:abs(x['simultaneousFootMomentNm'][0])),
       maximumUnreactedRoll=worst(lambda x:x['rollBeyondSeatNm']),
       extremePositiveAxial=worst(lambda x:x['forceToTowerN'][1]),extremeNegativeAxial=worst(lambda x:-x['forceToTowerN'][1]),
       analysis='Pipe-imposed load only at the recorded peak sample of each of the five source channels in every run. Allocate all axial reaction to the new idler thrust pair, radial force to the radial bearing. The drive side takes pure shaft torque. Exact robot mass, foot weld/fastener preload, cross-tie stiffness, clamp friction, and dynamic deflections are additional load cases. No full trace maximum is asserted beyond these 200 correlated records.',
       findings='A single compression-only 90 x 18 mm pad does not carry the signed torsional moment or net uplift. Qualify positive tower-to-frame ties and the actual host rail before claiming the thrust duty.')
    destination=d/'pan-thrust/r3-tower-foot-reactions-radial-center.json'
    destination.write_text(json.dumps(result,indent=2)+'\n')
    for key in ['maximumTowerUplift','maximumUnreactedRoll','extremePositiveAxial','extremeNegativeAxial']:
        i,row=result[key]
        print(key,i,round(row['upwardNetN'],2),round(row['simultaneousFootMomentNm'][0],3),round(row['maximumCompressionOnlyRollNm'],3))

if __name__=='__main__':
    study()
