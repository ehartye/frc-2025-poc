"""Associate visible native fasteners and reject cropped faces before estimating a planar pose."""
import numpy as np
import vision


class VisibleFasteners:
    def __init__(self, scene):
        self.scene = scene
        self.points = scene["bolts"]
        self.audit = []

    def identify(self, detections, prediction):
        rays = []
        ambiguous = False
        for index, (eye, name, detected) in enumerate(zip(vision.EYES, ("left", "right"), detections)):
            centers, _ = vision.project(self.points, prediction, eye)
            for target, center in enumerate(centers):
                nearby = [p for p in detected if np.linalg.norm(p-center) < 13]
                if len(nearby) != 1:
                    ambiguous |= len(nearby) > 1
                    continue
                pixel = nearby[0]
                if np.linalg.norm(pixel-center)>=8:
                    continue
                polygon, depth = vision.project(self.scene["faces"][target], prediction, eye)
                # Register the expected physical face at the measured center. Any possible
                # foreground pixel in the padded face window can bias its centroid.
                polygon += pixel-center
                x0,y0 = np.floor(polygon.min(axis=0)).astype(int)-1
                x1,y1 = np.ceil(polygon.max(axis=0)).astype(int)+2
                if x0 < 0 or y0 < 0 or x1 > 640 or y1 > 480:
                    continue
                raster = self.scene[name+"_z"][y0:y1,x0:x1]
                if np.min(raster) > depth.max()+12:
                    rays.append((index,target,pixel))
        if ambiguous or len({target for _,target,_ in rays}) != 2:
            return None
        return rays

    def pose(self, rays, prediction):
        def project(p):
            return np.concatenate([vision.project(self.points[j:j+1],p,vision.EYES[e])[0][0]
                                   for e,j,_ in rays])
        measured=np.concatenate([pixel for _,_,pixel in rays])
        pose=prediction.copy()
        steps=np.array([.1,.1,.0001])
        for _ in range(3):
            expected=project(pose)
            jacobian=np.stack([(project(pose+delta)-expected)/scale
                               for delta,scale in zip(np.diag(steps),steps)],axis=1)
            pose+=np.linalg.lstsq(jacobian,measured-expected,rcond=None)[0]
        return pose

    def bounds(self,rays,goal):
        # Only a complete two-feature eye can produce the final analytic outer bound.
        # A second physical pupil must also corroborate at least one head.
        if len(rays)<3:
            return None
        pairs=[[pixel for e,_,pixel in rays if e==eye] for eye in range(2)]
        if all(len(eye)==2 for eye in pairs):
            return vision.interval(pairs,self.points,goal,epsilon=.18)
        selected=0 if len(pairs[0])==2 else 1
        included=[pairs[0]] if selected==0 else [[],pairs[1]]
        return vision.interval(included,self.points,goal,epsilon=.18)

    def error(self,rays,truth):
        value=max(float(np.max(np.abs(pixel-truth[e][j]))) for e,j,pixel in rays)
        self.audit.append([len(rays),value])
        return value
