"""Perturb actual lens origins and ray directions, retaining the nominal registered backgrounds and decoder."""
import itertools
import json
import math
from unittest.mock import patch
import numpy as np
from design import HERE,ROOT,SHIFT,mount,optics,parts
from bridge_fit import fit,registration_scores
from PIL import Image

def matrix(axis,angle):
    a=np.array(axis,dtype=float)
    a/=np.linalg.norm(a)
    x,y,z=a
    skew=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    t=math.radians(angle)
    return np.eye(3)+math.sin(t)*skew+(1-math.cos(t))*skew@skew

def scenario(mesh,position,rotation,reference,points):
    im=optics.camera
    o=optics.ORIGIN
    ri=[]
    for k,g,c,vs,fs in mesh["parts"]:
        v=np.asarray(vs,dtype=float).reshape(-1,3)/10
        mapped=o+(v-o-position)@rotation
        ri.append([k,g,c,(mapped*10).reshape(-1),fs])
    scene=im.images_for_pose({"parts":ri})
    local=np.stack(np.broadcast_arrays(im.DX,im.DY,im.DZ))
    world=(rotation@local.reshape(3,-1)).reshape(local.shape)
    results=[]
    for x,y,z,a in points:
        pair=[]
        for eye,bg in zip(im.EYES,scene):
            pupil=o+position+rotation@np.array([0,eye,0])
            with patch.multiple(im,ORIGIN=pupil,EYES=(0.,),DX=world[0],DY=world[1],DZ=world[2]):
                pair+=im.exposure([bg],[x,y,z],a)
        answer=fit(pair,reference)
        fields=("x_mm","y_mm","bottom_mm")
        truths=(x,y,z-500-im.R)
        error=max((max(answer[k][0]-v,v-answer[k][1],0) for k,v in zip(fields,truths) if k in answer),default=None)
        results.append({"state":[x,y,z,a],"status":answer["status"],"truth_outside_mm":error,
                        "background_patch_fraction":registration_scores(pair,reference)})
    return results

def run():
    o=optics
    o.ORIGIN=np.array([98+SHIFT,0.,842.]);o.configure()
    with patch.object(mount,"parts",return_value=parts()):mesh=o.moving_mesh()
    bg=o.camera.images_for_pose(mesh)
    candidates=[]
    flex=json.loads((ROOT/"moving-observer/checks.json").read_bytes())["two_g_ideal_post_tip_mm"]
    slope=math.degrees(1.5*flex/359.5)
    points=[(68.58,0.,557.15,-1.15),(80.,0.,557.15,0.),(91.42,0.,557.15,1.15)]
    for axis,unit in (("fore-aft",(1,0,0)),("lateral",(0,1,0))):
        u=np.asarray(unit)
        bend=(0,1,0) if axis=="fore-aft" else (-1,0,0)
        for sign in (-1,1):
            offset=sign*(flex+.25)*u
            candidates.append({"case":f"{axis}/{sign}","translation":offset.tolist(),"axis":bend,"degrees":sign*slope})
    for sign in (-1,1):
        candidates.append({"case":f"vertical/{sign}","translation":[0,0,sign*1.],"axis":[0,0,1],"degrees":sign*.1})
    for sign in (-1,1):
        candidates.append({"case":f"vertical-refined/{sign}","translation":[0,0,sign*.5],"axis":[0,0,1],"degrees":sign*.05})
        for dx,dy in itertools.product((-1,1),repeat=2):
            direction=np.array([dx,dy,0])/math.sqrt(2)
            candidates.append({"case":f"diagonal/{dx}/{dy}/{sign}",
                "translation":((flex+.25)*direction+np.array([0,0,.5*sign])).tolist(),
                "axis":[-direction[1],direction[0],0],"degrees":slope})
    candidates.append({"case":"misregistered-5mm-2deg","translation":[5,0,0],"axis":[0,1,0],"degrees":2.})
    for v in candidates:
        v["results"]=scenario(mesh,np.array(v["translation"]),matrix(v["axis"],v["degrees"]),bg,points)
    controls={}
    for name,p in zip(("rearward","centered","forward"),points):
        raw=o.camera.exposure(bg,p[:3],p[3])
        controls[name]={"issued_decoder":o.fit(raw,bg)["status"],"aft_decoder":fit(raw,bg)["status"]}
    dim=o.camera.exposure(bg,points[1][:3],0.,intensity=174)
    controls["uncalibrated-low-contrast"]={"aft_decoder":fit(dim,bg)["status"]}
    frame=o.camera.exposure(bg,points[-1][:3],points[-1][3])[0]
    left=Image.fromarray(bg[0][0]);right=Image.fromarray(frame)
    from PIL import ImageDraw
    img=Image.new("RGB",(960,412),"#142533")
    d=ImageDraw.Draw(img)
    img.paste(left.resize((465,349)),(10,34));img.paste(right.resize((465,349)),(485,34))
    d.text((10,9),"Empty registered shelf",fill="white");d.text((485,9),"Pipe at the farthest loaded excursion",fill="white")
    d.text((10,390),"The forward outline crosses the shelf edge. The two curved ends and the rear tangent remain visible.",fill="white")
    # Encode an original rendered image as SVG scanlines for text-only public source.
    data=np.asarray(img.convert("RGB"))
    svg=['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 960 412"><title>Aft camera and pipe at the travel boundary</title>']
    colors={}
    for y,row in enumerate(data):
        edges=np.r_[0,np.flatnonzero(np.any(np.diff(row.astype(int),axis=0)!=0,axis=1))+1,len(row)]
        for x,last in zip(edges,edges[1:]):
            color=tuple(int(v) for v in row[x])
            colors.setdefault(color,[]).append(f"M{x} {y+.5}h{last-x}")
    for (r,g,b),paths in colors.items():svg.append(f'<path stroke="rgb({r},{g},{b})" d="{"".join(paths)}"/>')
    (HERE/"view.svg").write_text("".join(svg)+"</svg>\n",encoding="utf8",newline="\n")
    (HERE/"calibration.json").write_text(json.dumps({"cantilever_displacement_mm":flex,"cantilever_slope_deg":slope,
        "sampled_states_per_case":len(points),"fit_controls":controls,"cases":candidates},indent=2)+"\n",encoding="utf8",newline="\n")
if __name__=="__main__":run()
