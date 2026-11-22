"""Native opposed rack cassette; positions in the historical robot datum, millimetres."""
import math
from adsk import core,fusion

def mm(x):return core.ValueInput.createByString(str(x)+' mm')
def point(x,y,z):return core.Point3D.create(x/10,y/10,z/10)
def placement(deg,x,y,z,flip=False):
    c=math.cos(math.radians(deg));s=math.sin(math.radians(deg));k=-1 if flip else 1
    return [c,-k*s,0,x/10,s,k*c,0,y/10,0,0,k,z/10,0,0,0,1]

def cut_holes(comp,name,points,base_z,length,radius):
    anchor=comp.constructionPlanes.createInput();anchor.setByOffset(comp.xYConstructionPlane,mm(base_z))
    plane=comp.constructionPlanes.add(anchor);sk=comp.sketches.add(plane)
    sk.name=name+' locations'
    for x,y in points:sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(point(x,y,base_z)),radius/10)
    profiles=core.ObjectCollection.create()
    for p in sk.profiles:profiles.add(p)
    result=comp.features.extrudeFeatures.addSimple(profiles,mm(length),fusion.FeatureOperations.CutFeatureOperation)
    result.name=name;sk.isLightBulbOn=False;plane.isLightBulbOn=False

def make(des,member,component,profile,bar,vendor,rev):
    cx=-147
    finish=57 if rev=='r4' else 49
    fastener_z=(555,570) if rev=='r4' else (550,563)
    height=579 if rev=='r4' else 570
    if rev=='r4':
        carriage=member['Longitudinal carriage'].component
        # The six interior welds join real crossmembers to longitudinal plates.
        # Leave the outer guide, its 0.6mm top gap and the lower wear face clear.
        for word,x0,x1 in [('Rear tie',-240,-226),('Aft track',-62,-24),('Front track',273,311)]:
            for sign in (-1,1):
                profile(carriage,word+' inboard 3 mm fillet '+str(sign),[(sign*230,510),(sign*233,507),(sign*233,510)],
                        'x',x0+2,str(x1-x0-4)+' mm')
        bodies=list(carriage.bRepBodies)
        target=next(b for b in bodies if b.name=='Left moving guide plate')
        tools=core.ObjectCollection.create()
        for body in bodies:
            if body!=target:tools.add(body)
        joins=carriage.features.combineFeatures;args=joins.createInput(target,tools);args.operation=fusion.FeatureOperations.JoinFeatureOperation
        result=joins.add(args)
        if carriage.bRepBodies.count!=1:raise ValueError('Weldment did not join every stock member')
        result.bodies.item(0).name='Welded carriage with two captive guide tracks'
        for cx_stop in (-43,292):
            xp=carriage.yZConstructionPlane;nx=xp.geometry.normal.x
            ci=carriage.constructionPlanes.createInput();ci.setByOffset(xp,mm((cx_stop-21)/nx));plane=carriage.constructionPlanes.add(ci)
            sk=carriage.sketches.add(plane)
            for y in (-226,-100,100,226):
                sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(point(cx_stop-21,y,524)),.26)
            bores=core.ObjectCollection.create()
            for profile_ in sk.profiles:bores.add(profile_)
            carriage.features.extrudeFeatures.addSimple(bores,mm(42/nx),fusion.FeatureOperations.CutFeatureOperation)
            sk.isLightBulbOn=False;plane.isLightBulbOn=False
            for y in (-226,-100,100,226):
                label=str(cx_stop)+' '+str(y)
                vendor(carriage,'REV-29-2959','Captive jaw travel stop '+label,
                       [0,0,-1,(cx_stop+5.1173)/10,0,1,0,y/10,1,0,0,52.4,0,0,0,1],
                       'a3dad6cabe6b27a066bdcdc80610e3bd4d7c75edaa71650ce3751b845a78704e')
                vendor(carriage,'REV-29-1016','Jaw stop retaining nut '+label,
                       [0,1,0,(cx_stop+19)/10,-1,0,0,y/10,0,0,1,52.4,0,0,0,1],
                       'bc0a4ee3749b4d93d5e313a6d328297825c5117828396b8593b274760afbf38d')
    for side,sign in [('Left',1),('Right',-1)]:
        jaw=member[side+' splitting support'].component
        root_x=cx+sign*23.9125
        rack=vendor(jaw,'WCP-0291',side+' '+str(125+finish)+' mm cut rack',placement(90*sign,root_x,0,548),
                    '2519d52681cb423a48a7f50a7b2af96a6194ce930f621d26c98c61a2558ae3d7','.step')
        target=rack.component
        if not target.bRepBodies.count:
            children=[o.component for o in target.allOccurrences if o.component.bRepBodies.count]
            if len(children)!=1:raise ValueError('Expected one original solid inside rack stock')
            target=children[0]
        # A native intersection is an editable two-end saw operation. Retain the
        # distributor stock as the imported base feature with its original holes.
        anchor=target.constructionPlanes.createInput();anchor.setByOffset(target.xYConstructionPlane,mm(-3))
        plane=target.constructionPlanes.add(anchor);sk=target.sketches.add(plane)
        sk.sketchCurves.sketchLines.addTwoPointRectangle(sk.modelToSketchSpace(point(-125,-10,-3)),sk.modelToSketchSpace(point(finish,16,-3)))
        ext=target.features.extrudeFeatures.addSimple(sk.profiles.item(0),mm(6),fusion.FeatureOperations.IntersectFeatureOperation)
        ext.name='Saw the two registered end faces to '+str(finish+125)+' mm finished length'
        sk.isLightBulbOn=False;plane.isLightBulbOn=False
        hole_x=root_x+sign*2.2225
        lo,hi=hole_x-7,hole_x+7
        # Machine the integral L-section pull bracket as one shop part. Finish
        # the two jaw-side holes through the clamped parts from a common datum.
        path=[(lo,6),(hi,6),(hi,95),(-34,95),(-34,110),(lo,110)]
        tab=profile(jaw,side+' rack pull bracket',[(x,sign*y) for x,y in path],'z',540.4125,'6 mm')
        flange=profile(jaw,side+' rack bracket upright',[(-54,540.4125),(-34,540.4125),(-34,height),(-54,height)],
                       'y',104 if sign==1 else -110,'6 mm')
        combine=jaw.features.combineFeatures;to_merge=core.ObjectCollection.create();to_merge.add(flange)
        union=combine.createInput(tab,to_merge)
        union.operation=fusion.FeatureOperations.JoinFeatureOperation
        combine.add(union)
        start=sign*102
        xp=jaw.xZConstructionPlane;ny=xp.geometry.normal.y
        plane_in=jaw.constructionPlanes.createInput();plane_in.setByOffset(xp,mm(start/ny))
        plane=jaw.constructionPlanes.add(plane_in);sk=jaw.sketches.add(plane)
        for z in fastener_z:
            sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(point(-44,start,z)),.26)
        holes=core.ObjectCollection.create()
        for p in sk.profiles:holes.add(p)
        jaw.features.extrudeFeatures.addSimple(holes,mm(sign*22/ny),fusion.FeatureOperations.CutFeatureOperation)
        sk.isLightBulbOn=False;plane.isLightBulbOn=False
        for i,z in enumerate(fastener_z):
            # Local +Z is toward the screw head. The nut's +Y points away
            # from its seating plane. All dimensions here come from the SKU.
            vendor(jaw,'REV-29-2951',side+' jaw screw '+str(i+1),
                   [1,0,0,-4.4,0,0,sign,sign*(122-11.4173)/10,0,-sign,0,z/10,0,0,0,1],
                   '89e2b6ad909e8bdcb66fb86a7948298c159ffb0fd8c6b00bac022a7e54550ea2')
            vendor(jaw,'REV-29-1016',side+' jaw locknut '+str(i+1),
                   [1,0,0,-4.4,0,-sign,0,sign*10.4,0,0,-sign,z/10,0,0,0,1],
                   'bc0a4ee3749b4d93d5e313a6d328297825c5117828396b8593b274760afbf38d')
        # Actual supplier through-hole row is at -438.15 + n*25.4 mm.
        hole_positions=[(hole_x,sign*y) for y in ([19.05,44.45] if rev=='r4' else [12.7,38.1])]
        cut_holes(jaw,side+' rack screw clearance',hole_positions,539,8,2.60)
        for i,(x,y) in enumerate(hole_positions):
            # Authentic 3/4 in stock across the 3.175 rack plus 6mm bracket.
            vendor(jaw,'REV-29-2947',side+' rack screw '+str(i+1),placement(0,x,y,549.5875-8.2423),
                   'a54f396b0c1695ec7cc77275b526e7f365f9e83466be3b2eaca014d3312de052')
            vendor(jaw,'REV-29-1016',side+' rack locknut '+str(i+1),[1,0,0,x/10,0,0,1,y/10,0,-1,0,54.04125,0,0,0,1],
                   'bc0a4ee3749b4d93d5e313a6d328297825c5117828396b8593b274760afbf38d')
    rotor=component('Synchronized jaw drive')
    vendor(rotor,'WCP-0290','Coupled sixteen tooth pinion',placement(101.25,cx,0,548),
           '27edad00569d465247adfdada71fa5600a4325953abe2456027aade27af5d06d','.step')
    vendor(rotor,'REV-41-1620','Output hex adapter',placement(11.25,cx,0,521,True),
           '21bbba896df087dd8fca0d60c950fd09de54d88dd7f9a1cca1dcff767b6045cb')

def link(des,member,left,right):
    root=des.rootComponent
    sketch=root.sketches.add(root.xYConstructionPlane)
    pt=sketch.sketchPoints.add(point(-147,0,0));sketch.isLightBulbOn=False
    axis=fusion.JointGeometry.createByPoint(pt)
    ji=root.asBuiltJoints.createInput(member['Synchronized jaw drive'],member['Longitudinal carriage'],axis)
    ji.setAsRevoluteJointMotion(fusion.JointDirections.ZAxisJointDirection)
    gear=root.asBuiltJoints.add(ji);gear.name='Jaw pinion rotation'
    links=root.motionLinks
    m=fusion.JointMotionTypes
    def add(name,j1,j2,one,two,v1,v2,reverse):
        data=links.createInput(j1,j2);data.motionOne=one;data.motionTwo=two
        data.valueOne=v1;data.valueTwo=v2;data.isReversed=reverse
        actual=links.add(data);actual.name=name
    add('Equal and opposite rack travel',left,right,m.SliderJointSlideMotionType,m.SliderJointSlideMotionType,mm(10),mm(10),True)
    add('Rack pitch sets pinion rotation',left,gear,m.SliderJointSlideMotionType,m.RevoluteJointRotateMotionType,
        mm(10.16),core.ValueInput.createByString('1 rad'),False)

def jam(des):
    """Bound the first fraction of a millimetre using real installed tooth solids."""
    app=core.Application.get();manager=fusion.TemporaryBRepManager.get()
    tokens={'left':' mm cut rack WCP-0291','right':' mm cut rack WCP-0291','pinion':'Coupled sixteen tooth pinion'}
    frozen={};names={}
    for occurrence in des.rootComponent.allOccurrences:
        for k,t in tokens.items():
            if t not in occurrence.fullPathName:continue
            if k in ('left','right') and not occurrence.fullPathName.startswith(k.title()+' splitting support'):continue
            for b in occurrence.component.bRepBodies:
                if k in frozen:raise ValueError('Ambiguous installed drive '+k)
                frozen[k]=manager.copy(b)
                if not manager.transform(frozen[k],occurrence.transform2):raise ValueError('Could not place actual rack')
                names[k]=occurrence.fullPathName+'/'+b.name
    if set(frozen)!=set(tokens):raise ValueError('Missing installed teeth')
    doc=app.documents.add(core.DocumentTypes.FusionDesignDocumentType)
    try:
        scratch=fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        scratch.designIntent=fusion.DesignIntentTypes.HybridDesignIntentType
        scratch.designType=fusion.DesignTypes.DirectDesignType
        ro=scratch.rootComponent
        rows=[]
        for displacement,fraction in [(0,0),(.49873,0),(.49873,.25),(.49873,.5),(.49873,.75),(.49873,1)]:
            objects={}
            for k in tokens:
                shape=manager.copy(frozen[k]);mat=core.Matrix3D.create()
                if k=='left':mat.translation=core.Vector3D.create(0,displacement/10,0)
                if k=='pinion':mat.setToRotation(fraction*displacement/10.16,core.Vector3D.create(0,0,1),point(-147,0,548))
                if not manager.transform(shape,mat):raise ValueError('Failed perturbation')
                placed=ro.bRepBodies.add(shape);placed.name=k;objects[k]=placed
            items={}
            for k in ['left','right']:
                pair=core.ObjectCollection.create();pair.add(objects[k]);pair.add(objects['pinion'])
                setting=scratch.createInterferenceInput(pair);setting.areCoincidentFacesIncluded=False
                query=scratch.analyzeInterference(setting)
                a=manager.copy(objects[k]);b=manager.copy(objects['pinion'])
                if query is None or not manager.booleanOperation(a,b,fusion.BooleanTypes.IntersectionBooleanType):
                    raise ValueError('Unresolved native tooth interference')
                items[k]=dict(nativeMm3=sum(r.interferenceBody.volume*1000 for r in query),
                              transientMm3=1000*a.volume if a.isValid and a.lumps.count else 0)
            rows.append(dict(leftTravelMm=displacement,rightTravelMm=0,freePinionFraction=fraction,contacts=items))
            for body in objects.values():body.deleteMe()
        return dict(installed=names,rows=rows,scope='Indexed Native tooth contacts. Compliance and fracture still require material and loading evidence.')
    finally:doc.close(False)
