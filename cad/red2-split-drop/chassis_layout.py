"""Define a new six-faced chassis, structural miters and removable bumper modules."""
from pathlib import Path
import hashlib,json,math
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/red2-integrated'

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):return json.loads(path.read_bytes())

def layout():
    master=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    assert digest(ROOT/master['file'])==master['sha256']
    precedent=read(ROOT/'cad/rear-frame-facets/construction.json')
    assert precedent['frameSetbackMm']==19.05 and precedent['backerNormalMm']==4 and precedent['foamNormalMm']==69.5
    # Counterclockwise, counted from the front-left corner. Distinct stock
    # depths make the corner join planes different from a blanket 22.5° cut.
    h,c=350.,precedent['frameSetbackMm']
    v=np.array([[h,h],[-h+c,h],[-h,h-c],[-h,-h+c],[-h+c,-h],[h,-h]])
    names=['left','rear left miter','rear','rear right miter','right','front']
    width=np.array([25.,20.,20.,20.,25.,20.])
    tang=np.roll(v,-1,axis=0)-v;tang/=np.linalg.norm(tang,axis=1)[:,None]
    norm=np.column_stack((-tang[:,1],tang[:,0]))
    ds=np.sum(norm*v,axis=1)
    def ring(offset):
        distances=ds+offset
        return np.array([np.linalg.solve(norm[[i-1,i]],distances[[i-1,i]]) for i in range(6)])
    interior=ring(width)
    def face(i,outside,inside):
        return [outside[i],outside[(i+1)%6],inside[(i+1)%6],inside[i]]
    def quad(i,start,stop):
        a=ring(start);b=ring(stop)
        return face(i,a,b)
    def record(name,poly,z,h,material,process):
        return dict(name=name,planMm=np.round(poly,7).tolist(),zMinMm=z,heightMm=h,material=material,process=process)
    frame=[];layers=[];stock=[]
    for i,name in enumerate(names):
        poly=face(i,v,interior)
        tube=record(name+' frame tube',poly,50.,100.,'6061-T6 aluminum rectangular hollow stock',
                    'Cut both ends to the adjacent miter planes, deburr and fixture at the measured kingpin datums. Derate weld heat-affected zones.')
        w=width[i]
        corea=v+(interior-v)*(2/w)
        coreb=v+(interior-v)*((w-2)/w)
        tube.update(openCavityPlanMm=np.round(face(i,corea,coreb),7).tolist(),cavityZMm=[52.,148.],
                    sectionMm=[100.,w,2.],outsideLengthMm=float(np.linalg.norm(poly[1]-poly[0])))
        frame.append(tube)
        for layer,di,do,z0,z1,mat,process in [
            ('contact shim',0.,1.,57.,143.,'1 mm aluminum shim stock, supplier alloy and temper unresolved','Form metal to actual rail flatness at fixed mounting points.'),
            ('backing',1.,5.,43.,181.,'5052-H32 aluminum sheet, 4 mm incoming; weld heat-affected properties open','Cut and independently miter 4 mm stock; fixture and weld each removable U.'),
            ('padding',5.,74.5,43.,181.,'Sealed Air Ethafoam 220 closed-cell polyethylene, nominal 37 kg/m3; installed glued seams and recovery unqualified','Cut each face to the backing; retain full stock height through each corner.'),
            ('cover',74.5,75.,42.5,181.5,'AndyMark am-2675 red or am-2676 blue 1000-denier nylon Cordura; areal density and sewn thickness unverified','Sew fitted returns around the padded U, including both short rear faces.'),
            ('top return',1.,74.5,181.,181.5,'AndyMark am-2675 red or am-2676 blue 1000-denier nylon Cordura; areal density and sewn thickness unverified','Sew top return to the face and attach on the backing side.'),
            ('bottom return',1.,74.5,42.5,43.,'AndyMark am-2675 red or am-2676 blue 1000-denier nylon Cordura; areal density and sewn thickness unverified','Sew bottom return to the face and attach on the backing side.')]:
            p=np.array(quad(i,-di,-do))
            # The two U modules split at x=±1 on both straight side faces.
            # The skins on all other segments meet adjoining miter planes.
            def clip(at,positive):
                output=[]
                for a,b in zip(p,np.roll(p,-1,axis=0)):
                    ia=(a[0]>=at)==positive;ib=(b[0]>=at)==positive
                    if ia:output.append(a)
                    if ia!=ib:output.append(a+(b-a)*(at-a[0])/(b[0]-a[0]))
                return output
            if i in (0,4):sections=[('front',clip(1.,True)),('rear',clip(-1.,False))]
            else:sections=[('front' if i==5 else 'rear',p)]
            for module,f in sections:
                layers.append(record(module+' bumper '+name+' '+layer,f,z0,z1-z0,mat,process))
        stock.append(dict(face=name,outerStartMm=v[i].tolist(),outerEndMm=v[(i+1)%6].tolist(),
                          innerStartMm=interior[i].tolist(),innerEndMm=interior[(i+1)%6].tolist()))
    assert len(frame)==6 and len(layers)==48
    upper=[]
    def rectangle(x0,x1,y0,y1):
        return [[x0,y0],[x1,y0],[x1,y1],[x0,y1]]
    for name,sign in [('left',1),('right',-1)]:
        lo,hi=sorted([sign*233.,sign*251.])
        sill=record(name+' upper carriage sill',rectangle(-268,330,lo,hi),450.,30.,
                    '6061-T6 aluminum rectangular hollow stock',
                    'Saw square ends. Weld to the head of each inclined tower and the mating 6 mm rail shoe before installing the wear strips.')
        sill.update(openCavityPlanMm=rectangle(-268,330,lo+2,hi-2),cavityZMm=[452.,478.],
                    seatAgainst=name.title()+' machined anchor shoe')
        upper.append(sill)
        for fore,x in [('aft',-144.),('forward',148.)]:
            # The cut ends sit on horizontal seating planes. The full width
            # at the side tube and the full width at the upper sill are real
            # face contacts. An 18 mm horizontal chord is 17.15 mm normal
            # to the inclination. Make these as welded four-sheet box towers.
            y0,y1=sorted([sign*328.5,sign*346.5])
            post=record(name+' '+fore+' inclined tower',rectangle(x-15,x+15,y0,y1),150.,300.,
                        '6061-T6 aluminum sheet, 2 mm nominal',
                        'Fabricate a closed four-sheet box over a jig, miter both ends flat, inspect the root welds before the upper sill is fitted.')
            post.update(sectionTranslationMm=[0.,-sign*95.5,300.],openCavityPlanMm=rectangle(x-13,x+13,y0+2.099,y1-2.099),
                        cavityZMm=[150.,450.],seatAgainst=[name+' frame tube',sill['name']])
            upper.append(post)
    assert len(upper)==6
    return dict(authority=master,historicalFacetControlSha256=digest(ROOT/'cad/rear-frame-facets/fit.json'),
        physicalConfiguration='Separate successor frame and two detachable bumper sets. Red installed; interchangeable blue cover shares the same datums.',
        frame=frame,bumper=layers,upperSupport=upper,frameEndpoints=stock,
        exclusions=['No original bench, baseplate stand-in, public robot, or historical received-part geometry is inserted.',
                    'Attachment grooves, weld preparations, actual cover seams, access holes and mounting hardware must be detailed on the installed geometry.',
                    'Shop stock, exact rail corner radii/material condition, impact strength, competition inspection, wire clearances and whole-robot pose tests are open.'])

if __name__=='__main__':
    OUT.mkdir(parents=True,exist_ok=True);d=layout()
    (OUT/'chassis-stock.json').write_text(json.dumps(d,indent=2)+'\n')
    print([(s['name'],round(s['outsideLengthMm'],3)) for s in d['frame']],len(d['bumper']))
