"""Screen the corrected bridge against the signed local field and parked footprint."""
import itertools
import json
import math
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/split-drop-native'
target=json.loads((ROOT/'experiments/six-wheel/mission/target.json').read_bytes())
windows=json.loads((ROOT/'experiments/swerve-drive/low-placement/clearance.json').read_bytes())['cases']
native=json.loads((OUT/'pose-01.json').read_bytes())['native']['states']
issues=[]
for key,signed in windows.items():
    for dx,dy,yaw in itertools.product(*zip(*signed['enclosure'])):
        for pitch,zoff in itertools.product((-.04,.04),(-5,5)):
            for side in [0.,360.]:
                # R2 front crossbar geometry, at full extension. Include 1 mm
                # fixture machining/registration allowance on each box face.
                corners=[]
                for x,y,z in itertools.product((681,711),(-243,243),(509,519)):
                    offset=x*math.cos(pitch)-z*math.sin(pitch)
                    corners.append([dx-490+offset*math.cos(yaw)-y*math.sin(yaw),dy+side+offset*math.sin(yaw)+y*math.cos(yaw),x*math.sin(pitch)+z*math.cos(pitch)+zoff])
                pts=np.asarray(corners);low=pts.min(axis=0);hi=pts.max(axis=0)
                if hi[1]<-470 or low[1]>470:continue
                # The infinite target slope is conservative across the seam and holes.
                floor=low[2]-target['floor_intercept_m']*1000-target['floor_slope']*hi[0]
                lip=max(low[0]-target['lip_depth_m']*1000,low[2]-target['lip_top_m']*1000)
                riser=target['riser_bounds_m'][0][0]*1000-hi[0]
                issues.append(dict(key=key,sector=side,pose=[dx,dy,yaw,pitch,zoff],floorGapMm=float(floor),lipSeparationMm=float(lip),riserSeparationMm=float(riser)))
stowed=next(x for x in native if x['pose']=='home')['bodies'];max_x=max(v[3] for v in stowed.values())
report=dict(poses=len(issues),bounds={k:min(issues,key=lambda r:r[k]) for k in ['floorGapMm','lipSeparationMm','riserSeparationMm']},
            oldShopFixtureHomeFrontMm=max_x,newBridgeHomeFrontMm=306,otherHomeFrontMm=330,referenceFrameFrontMm=350,
            fixtureHomeFrontGapMm=20,fullBlockoutForwardGapMm=1,
            scope='New bridge vs official field floor, lip and inner riser at signed corner poses in both sectors. Conditional until the correct physical field solids and fixture reopen in Fusion. The separately tested original straight gate continues to hit the panel.')
(OUT/'r2-signed-field-screen.json').write_text(json.dumps(report,indent=2))
print({k:v[k] for k,v in report['bounds'].items()},report['poses'])
