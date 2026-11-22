"""Intersect the moving gate's centre plane with the two extracted field panels."""
import itertools
import math
from pathlib import Path
import json
import numpy as np

def in_panel(x,y,target):
    xy=np.array([x,y])/1000
    for panel in target['panels']:
        poly=np.array(panel['bounds_xy'])
        edge=np.roll(poly,-1,axis=0)-poly
        step=xy-poly
        cross=edge[:,0]*step[:,1]-edge[:,1]*step[:,0]
        if not (np.all(cross>.000001) or np.all(cross<-.000001)):continue
        if min(np.linalg.norm(xy-[hx,hy])-r for hx,hy,r in panel['holes_xy_r'])>.005:return True
    return False

def check(root):
    target=json.loads((root/'experiments/six-wheel/mission/target.json').read_bytes())
    cases=json.loads((root/'experiments/swerve-drive/low-placement/clearance.json').read_bytes())['cases']
    hits=[]
    total=0
    for key,v in cases.items():
        for dx,dy,yaw in itertools.product(*zip(*v['enclosure'])):
            for pitch,heave in itertools.product((-.04,.04),(-5,5)):
                total+=1
                a=math.radians(12)-pitch
                def point(h):
                    # The 8 mm gate sits immediately ahead of the seated tube.
                    along=370-55+57.15+4
                    rx=205*math.cos(pitch)-580*math.sin(pitch)+along*math.cos(a)+h*math.sin(a)
                    return np.array([dx-490+rx*math.cos(yaw),dy+rx*math.sin(yaw),205*math.sin(pitch)+580*math.cos(pitch)-along*math.sin(a)+h*math.cos(a)+heave])
                low,high=point(-106),point(-14)
                def d(p):return p[2]-1000*target['floor_intercept_m']-p[0]*target['floor_slope']
                dl,dh=d(low),d(high)
                if dl<=0<=dh:
                    touch=low-dl/(dh-dl)*(high-low)
                    if in_panel(*touch[:2],target):hits.append(dict(calibration=key,dock=[dx,dy,yaw],pitch=pitch,heaveMm=heave,pointMm=touch.tolist(),verticalBreachMm=-dl))
    return dict(total=total,intersectionCount=len(hits),worst=max(hits,key=lambda r:r['verticalBreachMm']) if hits else None)
