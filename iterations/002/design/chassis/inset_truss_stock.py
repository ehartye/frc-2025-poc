"""Define trimmed, weldable straight tube members by their actual two oblique sections."""
import json
import numpy as np
from inset_chassis_shop import HERE,tube,op,polygon


def add(article):
    model=json.loads((HERE/"inset-pan-truss.json").read_bytes());by_side=model["layout"]
    low,high=193.675,470.475
    for s,rec in by_side.items():
        sign=int(s);nodes=np.array(rec["nodes"]);prior=[]
        top_y=242*sign
        article("Upper pan stringer "+s,tube("Saw removable side stringer",[100.475,top_y-9.525,high],[259.525,top_y+9.525,high+19.05],0,1.5875),"pan strut")
        def prism(verts,p,q,z1,z2,h):
            d=q-p
            v=verts*h
            a=p+v
            pts=a[:,:2]+(z1-a[:,2,None])*d[:2]/d[2]
            return polygon(pts),((z2-z1)*d[:2]/d[2]).tolist()
        for index,(aa,bb) in enumerate(rec["edges"]):
            if aa<2 and bb<2:continue
            a,b=nodes[aa],nodes[bb];d=(b-a)/np.linalg.norm(b-a)
            # Both the saw planes and the tube are defined from the frame datum.
            v=np.cross(d,[0.,1.,0.]);v/=np.linalg.norm(v);u=np.cross(d,v)
            verts=np.array([-u-v,u-v,u+v,-u+v])
            steps=[]
            for label,optype,z1,z2,h in (("Cut to upper and lower bearing planes","new",low,high,9.525),("Open stock bore","cut",low-1.,high+1.,9.525-1.5875)):
                poly,xy=prism(verts,a,b,z1,z2,h);step=op(label,optype,2,z1,z2-z1,poly);step["offsetXY"]=xy;steps.append(step)
            for other,oa,ob,ov,charge in prior:
                if aa not in (oa,ob) and bb not in (oa,ob):continue
                poly,xy=prism(ov,nodes[oa],nodes[ob],low-.1,high+.1,9.525)
                step=op("Cope to adjoining tube "+str(other),"cut",2,low-.1,high-low+.2,poly);step["offsetXY"]=xy;steps.append(step)
            article(f"Pan truss {s} member {index}",steps,"pan strut")
            prior.append((index,aa,bb,verts,0.))
