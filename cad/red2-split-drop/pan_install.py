"""Dimension the selected pan, its fixed inlet and their mounts on the saved R2 frame."""
import math
import json
from pathlib import Path
from output_mount import ROOT, digest, read

DIR=ROOT/'artifacts/red2-integrated'
A='2025 6061-T6'
B='2025 5052-H32 aluminum'

def build():
    foundation=read(ROOT/'cad/red2-split-drop/chassis-foundation-r2.json')
    authority=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    assert digest(ROOT/authority['file'])==authority['sha256']==foundation['authority']['archiveSha256']
    assert digest(DIR/'red2-integrated-foundation-r2.f3d')==foundation['nativeArchiveSha256']
    items=[]
    def add(name,moving,grade,sections,axis=0,bores=None,hexes=None,void=None,process='Saw, mill and fit stock to the named faces.'):
        items.append(dict(id=name,moving=moving,material=grade,axis=axis,sections=sections,bores=bores or [],hexes=hexes or [],void=void,process=process))
    def yz(x,y0,y1,z0,z1):
        return [[x,y0,z0],[x,y1,z0],[x,y1,z1],[x,y0,z1]]
    def xz(y,x0,x1,z0,z1):
        return [[x0,y,z0],[x1,y,z0],[x1,y,z1],[x0,y,z1]]
    def box(name,move,g,x0,x1,y0,y1,z0,z1,**kw):
        add(name,move,g,[yz(x0,y0,y1,z0,z1),yz(x1,y0,y1,z0,z1)],**kw)
    def yface(y,coords):return [[x,y,z] for x,z in coords]
    def across(name,move,g,y0,y1,coords,**kw):
        add(name,move,g,[yface(y0,coords),yface(y1,coords)],1,**kw)
    def side(sign,a,b):return sorted([sign*a,sign*b])
    def bport(x,y,z,s,r,dep):return dict(start=[x,y,z],direction=[0,s,0],radius=r,depth=dep)
    # Root/world coordinates at zero pitch: vertex +340 is the duty's physical tip.
    box('430 mm pan floor',True,B,115,545,-161,161,578,580)
    for sign,word in [(1,'Port'),(-1,'Starboard')]:
        lo,hi=side(sign,161,166)
        across(word+' structural pan cheek',True,A,lo,hi,[(115,578),(545,578),(545,700),(115,700)])
        # A closed perimeter section supplies the long top edge struck by the pipe.
        # Two separate 20x20x2 structural tubes terminate clear of the hub.
        for level,label in [(680,'upper'),(580,'lower')]:
            y0,y1=side(sign,166,186)
            spans=[(115,545)] if label=='upper' else [(115,190),(220,545)]
            for num,(start,stop) in enumerate(spans,1):
                cavity=[yz(x,y0+2,y1-2,level+2,level+18) for x in (start,stop)]
                box(word+' cheek '+label+' closed stringer '+str(num),True,A,start,stop,y0,y1,level,level+20,void=cavity)
        # Webs bridge the two spines. Butt joints stop at both tube faces.
        for j,x in enumerate([115,160,205,250,295,340,385,430,475,520,541]):
            yy=side(sign,166,186)
            box(word+' cheek web '+str(j+1),True,A,x,x+4,*yy,600,680)
        y0,y1=side(sign,96,161)
        across(word+' slotted rear lip',True,B,y0,y1,[(105,690),(115,580),(115,578),(102.99,689.82)])
        # Place the hub on the cheek exterior below the lower stiffener.
        # A bored socket is machined in the cheek and lower stringer as one assembly.
        cheek=next(row for row in items if row['id']==word+' structural pan cheek')
        cheek['bores']=[bport(205,sign*160,580,sign,8,7)]
        y0,y1=side(sign,166,188)
        across(word+' driven hex socket',True,A,y0,y1,[(190,565),(220,565),(220,600),(190,600)],hexes=[dict(y0=y0,y1=y1,center=[205,580],flats=12.72)],process='Machine one solid socket block, broach the through hex, and join both interrupted lower stringers and the side wall. Fixture coaxially to both bearing bores.')
        if sign==-1:
            # Shop-turned headed shaft. Across flats are clocked using the same
            # datum as the driven socket. The pilot for an axial retainer is editable.
            radius=12.7/math.sqrt(3)
            hexagon=[(205+radius*math.cos(math.radians(60*i+30)),580+radius*math.sin(math.radians(60*i+30))) for i in range(6)]
            across('Idler headed stub hex shank',True,A,-202,-166,hexagon,process='Machine a one-piece headed stub in traceable 6061-T6 hex stock. Finish both shoulders; final internal axial retainer and material fatigue are open.')
        # Inboard bearing center y=195. OD shank 1.125, outside flange 1.225.
        y0,y1=side(sign,191.85,198.15)
        across(word+' bearing tower',False,A,y0,y1,[(171,487),(239,487),(239,605),(171,605)],bores=[bport(205,y0,580,1,14.2875,y1-y0)])
        y0,y1=side(sign,191.85,251)
        box(word+' bearing tower foot',False,A,160,250,y0,y1,480,487)
        # Frame sill belongs to the frozen R2 assembly from |Y|233 to 251.
        for j,x in enumerate((172,233)):
            ys=side(sign,198.15,240)
            c0=yz(x,ys[0],ys[1],487,488)
            zhinge=560
            reach=212 if sign==1 else 240
            tri=[[x,sign*198.15,487],[x,sign*reach,487],[x,sign*198.15,zhinge]]
            add(word+' tower knee '+str(j+1),False,A,[tri,[[v[0]+4,*v[1:]] for v in tri]])
    mmholes=[bport(205,212,580,1,12.2,4)]
    for hx,hz in [(13.856406,8),(0,16),(-13.856406,8),(-13.856406,-8),(0,-16),(13.856406,-8)]:
        mmholes.append(bport(205+hx,212,580-hz,1,1.65,4))
    across('Six-bolt gearbox reaction plate',False,A,212,216,[(170,487),(240,487),(240,606),(170,606)],bores=mmholes,process='Mill the centre opening and six actual M3 front mount clearances on the 32 mm circle. Deburr both sides. Weld foot and knees in a shaft alignment fixture.')
    for j,x in enumerate((174,231)):
        tri=[[x,216,487],[x,238,487],[x,216,554]]
        add('Gearbox reaction knee '+str(j+1),False,A,[tri,[[a+4,b,c] for a,b,c in tri]])
    # Fixed feed bed upper contact faces match the full physical trial.
    profile=[(-340,795,192),(-255,685,192),(-35,623,82),(75,592,82)]
    for i,((x0,z0,w0),(x1,z1,w1)) in enumerate(zip(profile,profile[1:])):
        thick=2*math.hypot(x1-x0,z1-z0)/(x1-x0)
        add('Fixed feed panel '+str(i+1),False,B,[yz(x0,-w0,w0,z0-thick,z0),yz(x1,-w1,w1,z1-thick,z1)])
    def edge(x):return 192-34*(x+340)/305
    for sign,word in [(1,'Port'),(-1,'Starboard')]:
        for i,((x0,z0,_),(x1,z1,_)) in enumerate(zip(profile[:2],profile[1:3])):
            def end(x,z,offset=0):
                ys=side(sign,edge(x)+offset,edge(x)+offset+3)
                return yz(x,*ys,z,z+120)
            add(word+' fixed inlet guide '+str(i+1),False,B,[end(x0,z0),end(x1,z1)])
            def rail(x,z):
                return yz(x,*side(sign,edge(x)+3,edge(x)+23),z+100,z+120)
            def inside(x,z):
                return yz(x,*side(sign,edge(x)+5,edge(x)+21),z+102,z+118)
            add(word+' fixed upper guide spine '+str(i+1),False,A,[rail(x0,z0),rail(x1,z1)],void=[inside(x0,z0),inside(x1,z1)])
        for j,(x,z) in enumerate([(-255,685),(-45,625.82)]):
            # Single flat structural web cut to reach the validated side sill
            # and the outer face of the upper fixed guide spine.
            def post(xi,inset=0):
                frame=242;head=edge(xi)+23;deck=685-(xi+255)*62/220
                # Fabricate the post as a hollow plate weldment. Both its
                # top shoe and its foot have finite full-width contact.
                return [[xi,sign*yy,zz] for yy,zz in [(frame-12.5+inset,484),(frame+12.5-inset,484),
                        (head+25-inset,deck+120),(head+inset,deck+120),(head+inset,deck+100)]]
            # Avoid projecting the forward post beyond the physical guide end.
            x=min(x,-60)
            add(word+' fixed inlet post '+str(j+1),False,A,[post(x),post(x+25)],
                void=[post(x+2.5,2.5),post(x+22.5,2.5)],process='Fabricate the inclined 25 mm box from 2.5 mm plates with an open-end inner profile; machine the upper saddle to the outside of the chute spine.')
            box(word+' fixed inlet mounting foot '+str(j+1),False,A,x-4,x+29,*side(sign,229.5,254.5),480,484)
    for i,(x,z,w) in enumerate(profile[1:3]):
        q0,q1=profile[i],profile[i+2]
        m0=(z-q0[1])/(x-q0[0]);m1=(q1[1]-z)/(q1[0]-x)
        d0=2*math.hypot(1,m0);d1=2*math.hypot(1,m1)
        coords=[(x-3,z-24),(x+3,z-24),(x+3,z+3*m1-d1),(x,z-d1)]
        if abs(d0-d1)>.001:coords.append((x,z-d0))
        coords.append((x-3,z-3*m0-d0))
        across('Fixed chute crossmember '+str(i+1),False,A,-edge(x)-3,edge(x)+3,coords)
    evidence=read(DIR/'single-pivot-spatial-loads.json')
    if len(evidence['runs'])!=40:raise ValueError('Incomplete full match replay')
    impact={}
    for label in ['guideImpact','panImpact']:
        r=max((v[label] for v in evidence['runs'].values() if v[label]),key=lambda v:v['normalN'])
        impact[label]=r
    out=dict(authority=authority,foundationSha256=foundation['nativeArchiveSha256'],shaftCenterMm=[205,0,580],driveDatumYmm=216,items=items,impacts=impact)
    (DIR/'pan-shop-plan.json').write_text(json.dumps(out,indent=2)+'\n')
    print(len(items),'editable upper parts',impact)

if __name__=='__main__':build()
