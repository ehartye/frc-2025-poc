"""Project the machined candidate onto the original plate and corrected feed models."""
import sys
from loaded_rack import HERE,SIX,digest,read
import numpy as np

sys.path.insert(0,str(SIX/"shelf-structure"))
import response as plate


def activate():
    cam=read(HERE/"cam.json")
    if digest(HERE/"cam-lands.step")!=cam["candidateStepSha256"]:
        raise ValueError("Different continuous-land assembly")
    source_sections=plate.sections
    d=cam["dimensionsMm"]
    def sections(x,y,hybrid,installed=False):
        t,h=source_sections(x,y,hybrid,installed)
        if not (hybrid and installed):
            return t,h
        # Physical section is measured down from metal Z496 / polymer Z500.
        rise=d["rise"]*np.clip((d["transitionX"]-x)/(d["transitionX"]-d["fullDepthX"]),0.,1.)
        lanes=(np.abs(y-d["axesY"][0])<d["landWidth"]/2)|(np.abs(y-d["axesY"][1])<d["landWidth"]/2)
        joined=lanes&(x>=d["fullDepthX"])&(x<=118)
        h[joined]=(11-rise[joined])/1000
        relief=lanes&(x<d["transitionX"])
        h[relief]=np.minimum(h[relief],(11-rise[relief])/1000)
        t[relief]=np.minimum(t[relief],(15-rise[relief])/1000)
        return t,h
    plate.sections=sections
    return cam


def candidate_gaps(s):
    # Minimum height under the whole 6.4 mm cap for a rising monotone track;
    # beyond the continuous land the same height conservatively fills open pockets.
    s=np.asarray(s)
    x=118-1000*s+3.2
    rise=3*np.clip((115-x)/114,0.,1.)
    gap=np.where(s>.2212,np.inf,(.1+rise)/1000)
    return np.column_stack((gap,gap))
