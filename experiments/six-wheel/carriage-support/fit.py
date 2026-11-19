"""Screen the static braces, the whole extension, attachment gauges and forward withdrawal."""
import argparse
from pathlib import Path
import sys
import numpy as np
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Vec
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from support import HERE,ROOT,j,make,BOLTS,Y_FRONT,FRONT,REAR,Y_REAR,xcyl,zcyl,compound,OUTLINE,prism
from mast import deck

def bbox(b):return j.box(b[0],b[3],b[1],b[4],b[2],b[5])

def selected(folder,file,source,pin,names):
    meta=j.read(folder/source)
    path=folder/file
    if j.digest(path)!=meta[pin]:raise ValueError('Changed '+file)
    return j.cad.read_named(path,[{'id':k} for k in meta[names]])

def extraction_envelopes(s):
    # Each prism bounds a manufactured feature throughout a 535 mm axial pull.
    # The pins, cutouts and lightening windows are deliberately filled here.
    rows=[('root-lower',-63,-25,205,210,342,358),
          ('root-upper',-63,-25,205,210,391,407),
          ('rear-pad',-100,-77,187,205,405,412),('forward-pad',154,174,174,205,426,433),
          ('forward-rib',144,174,190,205,417,426)]
    out={k:j.box(x,X+535,s*y,s*Y,z,Z) for k,x,X,y,Y,z,Z in rows}
    for name,loop,y,Y in [('web',OUTLINE,201,205),('rib',[(-31,341),(167,417),(159,420),(-34,347)],205,210)]:
        out[name+'-start']=prism(s,loop,y,Y)
        for i,(p,q) in enumerate(zip(loop,loop[1:]+loop[:1])):
            if abs(p[1]-q[1])>.001:
                out[name+str(i)]=prism(s,[p,q,(q[0]+535,q[1]),(p[0]+535,p[1])],y,Y)
    return out

def scene(reg,hoist):
    if j.digest(reg)!="291d68c43e21f20c33885064e6bbe8c0e47b684d13a8a06e8af61f9fd9db5886":raise ValueError("Different robot")
    src=j.read(reg)
    for file,digest in src['inputs'].items():
        if j.digest(ROOT/file)!=digest:raise ValueError(file)
    plugs=j.read(hoist/'fit.json')
    sys.path.insert(0,str(hoist))
    import importlib.util
    spec=importlib.util.spec_from_file_location('stationary_hoist_fit',hoist/'fit.py')
    fixed=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fixed)
    retired=fixed.retired()|{'long-common-shaft','slide-base-bridge','lift-crossbar-drilled','backup-L-drilled-mast','backup-R-drilled-mast'}
    obstacles={}
    # Use original full-stroke occupancy; replace its two whole-cable rectangles with
    # individually bounded line/arc sections from the issued source.
    for k,b in src['bounds'].items():
        if k not in retired|{'wire/power-feed','wire/data-feed'}:obstacles[k]=bbox(b)
    duplex=ROOT/'design/receiver-lift'
    names=j.read(duplex/'duplex-inventory.json')['poses']['place']
    defs=j.cad.read_named(duplex/'duplex-place.step',[{'id':k} for k in names])
    for k,v in defs.items():
        if k in obstacles and np.allclose(j.cad.bounds(v),src['bounds'][k],rtol=0,atol=.001):obstacles[k]=v
    # The issued harness replaces the original roof USB route. Retain its own
    # guides and the remaining power run. Carry actual tangent axes with the
    # same radial tolerance used in the integrated mast splice.
    sys.path.insert(0,str(ROOT/'fixed-camera-route'))
    import harness_fit
    from routing import power_feed
    for k in list(obstacles):
        if k in harness_fit.RETIRED or k.startswith('jacket/'):del obstacles[k]
    lines=harness_fit.jacket_and_form()[0]
    power=[]
    for length,pts in power_feed(-329):
        straight=abs(length-np.linalg.norm(pts[-1]-pts[0]))<1e-6
        power.append((length,pts,None if straight else 1.))
    lines['previous-roof-power']=(power,np.zeros(3),4.35)
    edges=[]
    for key,(path,offset,radius) in lines.items():
        for edge in harness_fit.edges(path,offset):edges.append((key,edge,j.cad.bounds(edge),radius+.5))
    for folder,file,meta,pin,ids in (
        (hoist,'stationary-shaft.step','fit.json','stepSha256','volumeMm3'),
        (ROOT/'mast-attachment','mast-joints.step','fit.json','stepSha256','volumeMm3'),
        (ROOT/'cover-retention','supported-covers.step','fit.json','stepSha256','volumeMm3'),
        (ROOT/'fixed-camera-route','harness-guides.step','guides.json','stepSha256','volumeMm3')):
        hardware=selected(folder,file,meta,pin,ids)
        for k,v in hardware.items():
            if k not in {'mast-base-L-mast','mast-base-R-mast'}:obstacles[k]=v
    for y in (-66,42):
        obstacles['damper-body-and-adjustment-'+str(y)]=zcyl(118,y,363.5,5.8,121.4)
    return obstacles,edges,src

def run(reg,hoist,out):
    out.mkdir(parents=True,exist_ok=True)
    old,modified,fresh=make()
    parts={**{'support-machined-'+k:v for k,v in modified.items()},**fresh}
    path=out/'carriage-support.step'
    j.write_step(path,parts);j.finalize_step(path)
    reopened=j.cad.read_named(path,[{'id':k} for k in parts])
    for k,v in reopened.items():
        if not BRepCheck_Analyzer(v).IsValid() or j.cad.solid_count(v)!=1 or abs(j.cad.volume(v)[0]-j.cad.volume(parts[k])[0])>.002:raise ValueError(k)
    obstacles,edges,registered=scene(reg,hoist)
    obstacles.update(reopened)
    bounds={k:j.cad.bounds(v) for k,v in obstacles.items()}
    # A retracted rack is a physical service prerequisite for upper driver access.
    service_obstacles=dict(obstacles)
    for k in registered["stowed"]:
        if k in obstacles and np.max(abs(np.asarray(registered["bounds"][k])-registered["stowed"][k]))>.001:
            service_obstacles[k]=bbox(registered["stowed"][k])
    stowed_bounds={k:j.cad.bounds(v) for k,v in service_obstacles.items()}
    def check(v,omit,stowed=False):
        a=j.cad.bounds(v)
        candidates=service_obstacles if stowed else obstacles
        hits={k:round(t,3) for k,b in (stowed_bounds if stowed else bounds).items() if k not in omit and j.lower_bound(a,b)<.001
              and (t:=j.cad.intersection(v,candidates[k]))>.001}
        for key,edge,b,r in edges:
            if key not in omit and j.lower_bound(a,b)<r and (d:=j.distance(v,edge)-r)<-.001:
                hits[key+'/intrusionMm']=min(hits.get(key+'/intrusionMm',0.),round(d,4))
        return hits
    fits={k:check(reopened[k],{k}) for k in fresh}
    tools={};service={}
    for s,side in ((1,'L'),(-1,'R')):
        pre='support-'+side+'-'
        for i,(x,z) in enumerate(BOLTS):
            name=pre+'mast-'+str(i)+'-'
            own={k for k in fresh if k.startswith(name)}
            for tag,gauge in [('inboardSocket',j.rod(x,s,175,201,z,7)),('nutSocket',j.rod(x,s,236.6,266,z,7)),('screwPull',compound([j.rod(x,s,151,201,z,5.8),j.rod(x,s,153,244,z,3)])),('sleevePull',j.rod(x,s,167,235,z,5))]:
                tools[side+'/'+str(i)+'/'+tag]=check(gauge,own|({pre+'ribbed-bracket'} if tag=='sleevePull' else set()))
        for i,y in enumerate(Y_FRONT):
            own={k for k in fresh if k.startswith(pre+'front-'+str(i)+'-')}
            tools[side+'/front'+str(i)+'Nut']=check(zcyl(FRONT,s*y,397,6.4,28),own)
            # Below the flush head its swept radius is bounded by the seated
            # monotone cone and shank. Beyond Z438 the full head translates.
            withdrawal=j.fuse(fresh[pre+'front-'+str(i)+'-flush-screw'],zcyl(FRONT,s*y,438,5.5,27))
            tools[side+'/front'+str(i)+'TopExtraction']=check(withdrawal,own,True)
            tools[side+'/front'+str(i)+'HexKey']=check(compound([zcyl(FRONT,s*y,438,2.8,24),xcyl(FRONT,s*y,460,3.2,50)]),own,True)
        rear_own={pre+'rear-washer',pre+'rear-screw'}
        tools[side+'/rearScrewWithdrawal']=check(compound([
            zcyl(REAR,s*Y_REAR,366,5.78,37.4),zcyl(REAR,s*Y_REAR,371,3,57.4)]),rear_own)
        tools[side+'/rearSocket']=check(zcyl(REAR,s*Y_REAR,376,7,27.4),{pre+'rear-washer',pre+'rear-screw'})
        br=fresh[pre+'ribbed-bracket']
        own={k for k in fresh if k.startswith(pre)}
        for tag,gauge in extraction_envelopes(s).items():
            service[side+'/forward/'+tag]=check(gauge,own)
    witness={}
    for side,s in (('L',1),('R',-1)):
        pre='support-'+side+'-';b=fresh[pre+'ribbed-bracket']
        def press(a,delta,c):return round(j.cad.intersection(j.cad.translated(a,delta),c),5)
        front=fresh[pre+'front-0-flush-screw'];rear=fresh[pre+'rear-screw']
        witness[side]={'frontDownSeatAt0p10mm':press(modified['bridge'],[0,0,-.1],b),
            'rearDownSeatAt0p10mm':press(modified['crossbar'],[0,0,-.1],b),
            'frontHeadCatchesAt0p10mm':press(modified['bridge'],[0,0,.1],front),
            'frontWasherToBracketAt0p10mm':press(fresh[pre+'front-0-washer'],[0,0,.1],b),
            'rearWasherToBracketAt0p10mm':press(fresh[pre+'rear-washer'],[0,0,.1],b),
            'mastBoltToSleeveAt0p5mm':press(fresh[pre+'mast-0-screw'],[0,0,.5],fresh[pre+'mast-0-sleeve']),
            'sleeveToMastAt0p5mm':press(fresh[pre+'mast-0-sleeve'],[0,0,.5],modified['mast-'+side])}
        if min(witness[side].values())<=0:raise ValueError(witness)
    result_controls=check(j.rod(-53,-1,151,201,335,5.8),set())
    damped=selected(ROOT/'shelf-structure','stow-receiver.step','installed.json','manufacturingSha256','volumeMm3')
    shelf_floor=min(j.cad.bounds(v)[2] for k,v in damped.items() if k!='slide-base-bridge')
    if shelf_floor<465:raise ValueError("Different overhead shelf")
    results={'minimumOverheadShelfMarginMm':shelf_floor-465,'bearingMm3':witness,'old335mmScrewPowerIntrusionMm':result_controls.get('previous-roof-power/intrusionMm'),
             'sourceSha256':{'registry':j.digest(reg),'stationaryShaft':j.digest(hoist/'stationary-shaft.step'),
              'mast':j.digest(ROOT/'mast-attachment/mast-joints.step'),'shelf':j.digest(ROOT/'shelf-structure/stow-receiver.step')},
             'stepSha256':j.digest(path),'registeredObstacles':len(obstacles),'volumesMm3':{k:round(j.cad.volume(v)[0],4) for k,v in reopened.items()},
             'installed':fits,'tools':tools,'service':service,'physicalFit':None}
    j.write(out/'fit.json',results)
    print({name:{k:v for k,v in rows.items() if v} for name,rows in [('installed',fits),('tools',tools),('service',service)]})
    if any(v for rows in (fits,tools,service) for v in rows.values()):raise ValueError('Unresolved interference, see fit.json')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('registry',type=Path);p.add_argument('hoist',type=Path);p.add_argument('--out',type=Path,default=HERE)
    a=p.parse_args();run(a.registry,a.hoist,a.out)
