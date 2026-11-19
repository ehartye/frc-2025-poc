"""Synthetic unmarked pipe images in front of registered, depth-tested robot geometry."""
import base64
import gzip
import json
import math
from pathlib import Path
import re
import sys
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0,str(ROOT/"release-observer"))
import stereo
W,H,F = stereo.WIDTH, stereo.HEIGHT, stereo.FOCAL
C,S = stereo.C,stereo.S
ORIGIN = np.array(stereo.ORIGIN)
EYES = (stereo.BASELINE/2, -stereo.BASELINE/2)
R,HALF = stereo.RADIUS, stereo.LENGTH/2
RED = ROOT/"design/competition-package/shop-duplex/red"
V,U = np.mgrid[:H,:W] + .5
Q = (H/2 - V)/F
DX,DY,DZ = C+Q*S, -(U-W/2)/F, -S+Q*C


def read_mesh():
    text=(RED/"robot.html").read_text(encoding="utf-8")
    contents=re.search(r'<script[^>]*id=.mesh.[^>]*>(.*?)</script>',text,re.S).group(1)
    return json.loads(gzip.decompress(base64.b64decode(contents)))


def images_for_pose(pose):
    import vtk
    from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy
    vertices, indices, shades = [], [], []
    offset=0
    for key,group,color,points,faces in pose["parts"]:
        if key=="observer-camera-proxy": continue # These rays originate in its two surveyed optical windows.
        points=np.asarray(points,dtype=float).reshape(-1,3)/10
        faces=np.asarray(faces,dtype=np.int64).reshape(-1,3)
        vertices.append(points)
        indices.append(faces+offset)
        offset+=len(points)
        rgb=bytes.fromhex(color[1:])
        gray=int(40 + .50*sum(rgb)/3)
        shades.append(np.full((len(faces),3), gray,dtype=np.uint8))
    pts=vtk.vtkPoints()
    pts.SetData(numpy_to_vtk(np.vstack(vertices),deep=True))
    triangles=np.vstack(indices).reshape(-1)
    cells=vtk.vtkCellArray()
    cells.SetData(numpy_to_vtkIdTypeArray(np.arange(0,len(triangles)+1,3,dtype=np.int64),deep=True),
                  numpy_to_vtkIdTypeArray(triangles,deep=True))
    poly=vtk.vtkPolyData()
    poly.SetPoints(pts)
    poly.SetPolys(cells)
    poly.GetCellData().SetScalars(numpy_to_vtk(np.vstack(shades),deep=True))
    mapper=vtk.vtkPolyDataMapper()
    mapper.SetInputData(poly)
    actor=vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().LightingOff()
    renderer=vtk.vtkRenderer()
    renderer.AddActor(actor)
    renderer.SetBackground(.12,.12,.12)
    win=vtk.vtkRenderWindow()
    win.SetShowWindow(False)
    win.SetOffScreenRendering(1)
    win.SetMultiSamples(0)
    win.SetSize(W,H)
    win.AddRenderer(renderer)
    result=[]
    try:
        for eye in EYES:
            o=ORIGIN+[0,eye,0]
            cam=renderer.GetActiveCamera()
            cam.SetPosition(*o)
            cam.SetFocalPoint(*(o+np.array([C,0,-S])))
            cam.SetViewUp(S,0,C)
            cam.SetViewAngle(math.degrees(2*math.atan(H/(2*F))))
            cam.SetClippingRange(10,3500)
            win.Render()
            planes=[]
            for depth in (False,True):
                capture=vtk.vtkWindowToImageFilter()
                capture.SetInput(win)
                if depth: capture.SetInputBufferTypeToZBuffer()
                capture.ReadFrontBufferOff()
                capture.Update()
                array=vtk_to_numpy(capture.GetOutput().GetPointData().GetScalars())
                if depth:
                    value=array.reshape(H,W)[::-1].astype(float)
                    planes.append(3500*10/(3500-value*(3500-10)))
                else:
                    planes.append(array.reshape(H,W,3)[::-1,:,0].copy())
            result.append(tuple(planes))
    finally:
        win.Finalize()
    return result


def exposure(background, center, yaw=0., intensity=220):
    """Nearest intersection with the finite cylindrical sidewall and annular end caps."""
    pair=[]
    axis=np.array([math.sin(math.radians(yaw)),math.cos(math.radians(yaw)),0.])
    dot=DX*axis[0]+DY*axis[1]
    length2=DX**2+DY**2+DZ**2
    for eye,(gray,depth) in zip(EYES,background):
        pixels=gray.copy()
        if center is not None:
            origin=ORIGIN+np.array([0,eye,0])-np.asarray(center)
            along=origin@axis
            cross=DX*origin[0]+DY*origin[1]+DZ*origin[2]-dot*along
            denom=length2-dot**2
            discr=cross**2-denom*(origin@origin-along**2-R**2)
            distance=(-cross-np.sqrt(np.maximum(discr,0)))/denom
            inside=(discr>=0)&(distance>0)&(abs(along+distance*dot)<=HALF)
            best=np.where(inside,distance,np.inf)
            for face in (-HALF,HALF):
                with np.errstate(divide="ignore",invalid="ignore"):
                    end=(face-along)/dot
                    rho=(origin@origin + 2*end*(cross+dot*along) + end**2*length2)-HALF**2
                cap=(end>0)&(rho>=50.8**2)&(rho<=R**2)
                best=np.minimum(best,np.where(cap,end,np.inf))
            visible=best<depth
            pixels[visible]=intensity
        pair.append(pixels)
    return pair
