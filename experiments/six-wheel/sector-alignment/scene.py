"""Open the registered image data; measured primitives refer to the front of the red reef."""
import hashlib
import json
from pathlib import Path
import numpy as np
import cv2
from vision import corners

HERE=Path(__file__).resolve().parent

def context(cache):
    rec=json.loads((HERE/"scene.json").read_bytes())
    data=dict(np.load(cache/"fixed-stowed.npz"))
    for key,sha in rec["imageArraysSHA256"].items():
        if hashlib.sha256(np.ascontiguousarray(data[key]).tobytes()).hexdigest()!=sha:
            raise ValueError("Unregistered optical background")
    faces=[];centers=[]
    for _,vertices in sorted(rec["boltFacesMm"].items()):
        v=np.asarray(vertices);h=cv2.convexHull(v[:,1:].astype("float32")).reshape(-1,2)
        moments=cv2.moments(h)
        center=[v[:,0].min(),moments["m10"]/moments["m00"],moments["m01"]/moments["m00"]]
        faces.append(np.column_stack((np.full(len(h),center[0]),h)));centers.append(center)
    data.update(points=corners(np.asarray(rec["targetFrameCenterMm"])),faces=faces,bolts=np.asarray(centers),front=[])
    return data
