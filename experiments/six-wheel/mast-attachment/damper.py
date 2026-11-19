"""Route the signed stowed damper/rail resultants, including the original two plunger reactions."""
import argparse
import hashlib
from pathlib import Path

import numpy as np
from loads import HERE,REFERENCE,j,report


def run(folder,source):
    sections=j.read(HERE/"load.json")["sections"]
    results={}
    for case in j.read(source):
        csv=folder/(case["id"]+"-support.csv")
        if hashlib.sha256(csv.read_bytes()).hexdigest()!=case["supportHistorySha256"]:raise ValueError("Different damper support trace")
        d=np.loadtxt(csv,delimiter=",",skiprows=1)
        t=d[:,0]
        force=d[:,1:4]
        result={}
        for yaw in (-50.,50.):
            # Yaw is absent from this CSV. Screen a declared symmetric bounding exercise.
            moment=np.column_stack([d[:,4:6],np.full(len(t),yaw)])
            for s in (1,-1):
                p=np.array([-45,s*222.5,154])
                side="L" if s==1 else "R"
                for kind in ("verticalCouple","sharedRoll","single"):
                    f=force.copy() if kind=="single" else force*.5
                    m=moment.copy() if kind=="single" else moment*.5
                    if kind=="verticalCouple":
                        f[:,2]+=s*moment[:,0]/(.445)
                        m[:,0]=0.
                    q=report(t,f,m,REFERENCE if kind=="single" else p,s,sections[str(s)])
                    key=kind+side
                    if key not in result: result[key]={"maxGrossMPa":0,"spanGrossUpperMPa":0,"maxFastenerMPa":0}
                    r=result[key]
                    r["spanGrossUpperMPa"]=max(r["spanGrossUpperMPa"],q["spanGrossUpperMPa"])
                    # The two maxima may be on different clocks and are recorded separately.
                    if q["sectionMPa"]>r["maxGrossMPa"]:
                        r.update(maxGrossMPa=q["sectionMPa"],criticalTimeS=q["witnessTimeS"],
                                 forceAtSectionPeakN=q["coincidentForceN"],momentAtSectionPeakNm=q["coincidentSectionNm"])
                    r["maxFastenerMPa"]=max(r["maxFastenerMPa"],max(v["threadVonMisesMPa"] for g in q["group"].values() for v in g.values()))
        results[case["case"]]={"sampleCount":len(t),"supportSha256":case["supportHistorySha256"],
                               "minimumChassisFzN":float(force[:,2].min()),
                               "maximumDownwardPerDamperN":np.max(d[:,6:],axis=0).tolist(),"bounds":result}
    j.write(HERE/"damper.json",{"sourcesSha256":j.digest(source),"includesBothPlungersOnce":True,
            "assumedYawNm":[-50,50],"measuredLoadPath":None,"cases":results})


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder",type=Path)
    p.add_argument("source",type=Path)
    a=p.parse_args()
    run(a.folder,a.source)
