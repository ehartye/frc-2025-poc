"""Synthetic patch check for a round bright bolt-face approximation. No real camera frame."""
import math
import numpy as np
from pose import EYE,SECOND_EYE,POINTS,FOCAL,WIDTH,HEIGHT,image,local,axes


def sample(eye,point,shift,radius,adverse,subsamples=16):
    center=np.array(image(point,(0,0,0),eye))+shift
    ys,xs=np.mgrid[math.floor(center[1])-23:math.ceil(center[1])+24,math.floor(center[0])-23:math.ceil(center[0])+24]
    fx,fy,fz=local(point)
    right,down,forward=np.array(axes(eye))
    # Subpixel integration of a station-plane disk; radial boundary projects to a conic.
    intensity=np.zeros(xs.shape)
    s=(np.arange(subsamples)+.5)/subsamples-.5
    for sx in s:
        for sy in s:
            u=(xs+sx-shift[0]-WIDTH/2)/FOCAL
            v=(ys+sy-shift[1]-HEIGHT/2)/FOCAL
            depth=(fx-eye[0])/(forward[0]+u*right[0]+v*down[0])
            a=eye[1]+depth*(forward[1]+u*right[1]+v*down[1])-fy
            b=eye[2]+depth*(forward[2]+u*right[2]+v*down[2])-fz
            weight=(a*a+b*b<=radius*radius).astype(float)
            if adverse: weight*=1+.15*a/radius
            intensity+=weight
    return np.array([(xs*intensity).sum(),(ys*intensity).sum()])/intensity.sum()-center


def run():
    radius=5.55625  # Approximate disk from the measured front face span, omitting engravings.
    result={}
    for name,eye in (("right",EYE),("left",SECOND_EYE)):
        raw=corrected=shadow=0.
        for p in POINTS:
            # Geometry-based conic-center correction from a separate nominal ideal patch.
            bias=sample(eye,p,np.zeros(2),radius,False)
            for shift in ([-.35,.2],[.15,.4],[.4,-.3]):
                v=sample(eye,p,shift,radius,False)
                w=sample(eye,p,shift,radius,True)
                raw=max(raw,float(max(abs(v))))
                corrected=max(corrected,float(max(abs(v-bias))))
                shadow=max(shadow,float(max(abs(w-bias))))
        result[name]={"uncorrectedUniformMaxPx":raw,"correctedUniformMaxPx":corrected,
                      "corrected15PercentGradientMaxPx":shadow}
    return result


if __name__=="__main__":
    import json
    print(json.dumps(run(),indent=2))
