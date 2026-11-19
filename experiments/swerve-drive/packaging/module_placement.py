"""Shared front-left/front-right/rear-left/rear-right geometry in robot millimetres."""
import json
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]/"six-wheel"
SITES={"FL":(282,277), "FR":(282,-277), "RL":(-282,277), "RR":(-282,-277)}
def basis(site):
    x,y=SITES[site]
    # Source STEP vertical is +Y. Both +X and +Z point into this robot.
    tx=np.array([-np.sign(x),0.,0]);ty=np.array([0.,-np.sign(y),0]);z=np.array([0.,0.,1.])
    for a,b in ((tx,ty),(ty,tx)):
        m=np.column_stack((a,z,b))
        if np.linalg.det(m)>0:return m
    raise ValueError(site)
def place(p,site):
    return np.asarray(p)@basis(site).T+[*SITES[site],38.1]
def rotated_box(box,site):
    import itertools
    pts=place(list(itertools.product(*[(box[i],box[i+3]) for i in range(3)])),site)
    return np.r_[pts.min(axis=0),pts.max(axis=0)]
def write(path,data):path.write_text(json.dumps(data,indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")
def read(path):return json.loads(path.read_bytes())
def contract():
    return {"assembly":"REV-21-3005 V1.1 with REV-21-2986, NEO V1.1, NEO 550 and V2 plastic wheel",
            "role":"fixed-low station to reef and recovery","frame":"robot right handed, +X forward, +Y left, +Z up",
            "order":list(SITES),"modules":{k:{"xM":x/1000,"yM":y/1000,"groundZMm":0.0,"sourceXyzToRobot":basis(k).tolist()}
                for k,(x,y) in SITES.items()},"wheelDiameterM":.0762,"driveMotorTurnsPerWheelTurn":5.5,
            "driveStages":{"pinion":12,"spur":22,"bevelPinion":15,"wheelBevel":45},"azimuthMotorTurnsPerModuleTurn":9424/203,
            "sourceWheelAxis":"supplier +X; wheel center [0,0,0], azimuth +Y, shaft located on azimuth center",
            "driveMotor":"REV-21-1650","steerMotor":"REV-21-1651","controllerPerCorner":["SPARK MAX","SPARK MAX"],
            "absoluteEncoder":"REV-11-1271 at azimuth; 10 bit",
            "geometryStatus":"initial native layout; inspect signed fit and revision scope before clearance transfer",
            "sourceDrawing":"https://www.revrobotics.com/content/docs/REV-21-3005-DR.pdf","sourceRevision":"supplier assembly STEP 2022-11-14; separate V1.1 change inputs",
            "sourceStepSha256":"11f8381810846a2e52a23dbbe07f8271b314967624eee53f56e59a3ece05c75e"}
if __name__=="__main__":write(HERE/"module.json",contract())
