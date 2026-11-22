"""Editable first-article pan, tapered walls and side-fenced Red 2 jaw weldments."""
from adsk import core,fusion

def mm(x): return core.ValueInput.createByString(f'{x} mm')
def point(x,y,z): return core.Point3D.create(x/10,y/10,z/10)

def make(des,member,profile,bar):
    fixed=member['Fixture bench rails'].component
    pan=[(-340,795),(-255,685),(-65,602)]
    jaw=[(-65,602),(205,550),(290,595),(315,698)]
    fixed_pan=profile(fixed,'Bent receiving pan 4 mm 6061',pan+[(x,z-4) for x,z in reversed(pan)],'y',-180,'360 mm')
    def height(points,x):
        for (x0,z0),(x1,z1) in zip(points,points[1:]):
            if x0<=x<=x1: return z0+(z1-z0)*(x-x0)/(x1-x0)
        raise ValueError('Outside sheet')
    def weld(comp,target,parts,name):
        stock=core.ObjectCollection.create()
        for body in parts:stock.add(body)
        combine=comp.features.combineFeatures
        value=combine.createInput(target,stock);value.operation=fusion.FeatureOperations.JoinFeatureOperation
        joined=combine.add(value)
        if joined.bodies.count!=1:raise ValueError(name+' contains detached stock')
        joined.bodies.item(0).name=name
    def access(comp,side):
        plane=comp.xZConstructionPlane
        ny=plane.geometry.normal.y
        inp=comp.constructionPlanes.createInput();inp.setByOffset(plane,mm(side*157/ny))
        surface=comp.constructionPlanes.add(inp);sk=comp.sketches.add(surface)
        for z in (555,570):
            sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(point(-44,side*157,z)),.60)
        selected=core.ObjectCollection.create()
        for prof in sk.profiles:selected.add(prof)
        cut=comp.features.extrudeFeatures.addSimple(selected,mm(side*49/ny),fusion.FeatureOperations.CutFeatureOperation)
        cut.name='Two 12 mm cross-bores for jaw screws and hex driver'
        sk.isLightBulbOn=False;surface.isLightBulbOn=False
    # Tapered side stock uses real YZ profiles at the fold and physical ends.
    # Each pair of parallel edges is separated by 4 mm in lateral projection.
    # The bottom lip straddles the pan edge and welds it at the adjacent bosses.
    fixed_parts=[]
    for word,side in [('Left',1),('Right',-1)]:
        lofts=fixed.features.loftFeatures
        sections=[]
        for x in (-340,-255,-68):
            y=side*(192-(x+360)*34/325)
            z=height(pan,x)
            ci=fixed.constructionPlanes.createInput()
            ci.setByOffset(fixed.yZConstructionPlane,mm(x/fixed.yZConstructionPlane.geometry.normal.x))
            plane=fixed.constructionPlanes.add(ci);sk=fixed.sketches.add(plane)
            sk.sketchCurves.sketchLines.addTwoPointRectangle(
                sk.modelToSketchSpace(point(x,y,z-6)),sk.modelToSketchSpace(point(x,y+4*side,z+120)))
            sections.append(sk.profiles.item(0))
            sk.isLightBulbOn=False;plane.isLightBulbOn=False
        for a,b in zip(sections,sections[1:]):
            li=lofts.createInput(fusion.FeatureOperations.NewBodyFeatureOperation)
            li.loftSections.add(a);li.loftSections.add(b)
            body=lofts.add(li).bodies.item(0);body.name=word+' tapered station side';fixed_parts.append(body)
            body.material=fixed_pan.material
        # Rear posts transfer to the stationary crossbar. Forward posts stand
        # outboard of the moving guide tracks, on added base feet at y=265.
        for x in (-282,-85):
            z=height(pan,x)-4
            if x==-282:
                plate=profile(fixed,word+' rear receiver upright',[(side*150,492),(side*176,492),(side*176,z+1),(side*150,z+1)],'x',x-3,'6 mm')
                fixed_parts.append(plate)
            else:
                y0,y1=sorted([side*252,side*292])
                fixed_parts.append(profile(fixed,word+' outboard receiver foot',
                    [(x-13,y0),(x+13,y0),(x+13,y1),(x-13,y1)],'z',474,'6 mm'))
                fixed_parts.append(profile(fixed,word+' forward outboard receiver bracket',
                    [(side*277,480),(side*289,480),(side*289,z+1),(side*174,z+1),(side*174,z-6),(side*277,z-6)],'x',x-3,'6 mm'))
        c=member[word+' splitting support'].component
        target=next(b for b in c.bRepBodies if b.name==word+' pipe rail')
        parts=[]
        ys=lambda a,b:min(side*a,side*b)
        parts.append(profile(c,word+' 5 mm outer cheek',
            [(x,z+150) for x,z in jaw]+[(315,542),(-65,542)],'y',ys(160,165),'5 mm'))
        for tag,z,h in [('lower skin',542,3),('upper skin',569,3)]:
            y0,y1=sorted((side*165,side*202))
            parts.append(profile(c,word+' boxed spine '+tag,[(-65,y0),(315,y0),(315,y1),(-65,y1)],'z',z,f'{h} mm'))
        parts.append(profile(c,word+' boxed spine outside skin',[(-65,545),(315,545),(315,569),(-65,569)],'y',ys(199,202),'3 mm'))
        for x in (-61,4,70,136,202,253,286,309):
            h=height(jaw,x)+147
            parts.append(profile(c,word+' outer cheek rib '+str(x),
                [(side*165,572),(side*199,572),(side*165,h)],'x',x-2,'4 mm'))
        # Cross-web roots attach only to the two original tall supports. The
        # 12 mm access windows preserve the screw tool line between the aft pair.
        for x,h in [(-54,582),(-38,582),(282,582),(296,582)]:
            parts.append(profile(c,word+' deep root '+str(x),
                [(side*122,541),(side*160,541),(side*160,h),(side*122,h)],'x',x,'4 mm'))
        weld(c,target,parts,word+' pipe rail')
        access(c,side)
    # Retain independent bench anchors as discrete parts. The pan, sides, both
    # uprights and their outboard feet form one weldment on those fixed surfaces.
    weld(fixed,fixed_pan,fixed_parts,'Station receiving pan and supports')

def assign_materials(des,library):
    """Use the issuer's grade where published and an explicit alloy family elsewhere."""
    grade={}
    catalog=list(library.materials)
    def select(key,includes):
        matches=[x for x in catalog if all(q in x.name.lower() for q in includes)]
        if matches:grade[key]=min(matches,key=lambda x:len(x.name))
    select('rack',['6061','t6']);select('pinion',['steel, alloy']);select('adapter',['7075']);select('nylon',['nylon 6'])
    select('alloy screws',['steel, alloy']);select('shell',['steel'])
    labels={'rack':'Supplier 6061-T6 aluminum','pinion':'Supplier 4140 steel, temper unspecified',
            'adapter':'Supplier 7075 aluminum, temper unspecified','nylon':'Supplier nylon insert, grade unspecified',
            'alloy screws':'Supplier alloy steel, black oxide, grade unspecified','shell':'Supplier zinc plated steel shell, grade unspecified'}
    if set(grade)!=set(labels):raise ValueError('Missing required stock families '+repr(set(labels)-set(grade)))
    for key,base in list(grade.items()):
        material=des.materials.itemByName(labels[key])
        if not material:material=des.materials.addByCopy(base,labels[key])
        grade[key]=material
    counts={}
    for occ in des.rootComponent.allOccurrences:
        p=occ.fullPathName
        for body in occ.component.bRepBodies:
            kind=None
            if 'REV-29-1016' in p: kind='nylon' if body.name=='Body1' else 'shell'
            if any(q in p for q in ('REV-29-2947','REV-29-2951','REV-29-2959')):kind='alloy screws'
            if 'WCP-0290' in p:kind='pinion'
            if 'WCP-0291' in p:kind='rack'
            if 'REV-41-1620' in p:kind='adapter'
            if kind:
                if kind not in grade:raise ValueError('Native material family missing '+kind+': '+', '.join(x.name for x in catalog if any(t in x.name.lower() for t in ['nylon','6061','4140','7075','steel'])))
                body.material=grade[kind]
                counts[kind]=counts.get(kind,0)+1
                if kind=='pinion':
                    # Fusion's physical mass (kg) and solid volume (cm³) give
                    # a unit-independent scale for its editable density field.
                    # Industeel's 09/2022 DS-MOLD-4140 gives 7.85 g/cm³.
                    # No WCP heat treatment or yield allowable is inferred.
                    prior=body.physicalProperties.mass/body.volume
                    ids=set()
                    for p in grade[kind].materialProperties:
                        if p.id not in ('structural_Density','thermal_Density'):continue
                        density=core.FloatProperty.cast(p)
                        if density is None:raise ValueError('Density property is not scalar')
                        if density.units=='KilogramPerCubicMeter':density.value=7850.
                        else:density.value*=.00785/prior
                        ids.add(p.id)
                    if 'structural_Density' not in ids:raise ValueError('Native structural density absent')
                    mass=body.physicalProperties.mass
                    if abs(mass/body.volume-.00785)>.0000002:raise ValueError('Native 4140 mass check did not reproduce 7.85 g/cm3')
                    occ.component.attributes.add('Supplier material','engineering scope',
                       'WCP-0290 is 4140. Density 7.85 g/cm3 from Industeel DS-MOLD-4140 09/2022. Other mechanical properties and heat treatment unqualified.')
    expected={'rack':2,'pinion':1,'adapter':1,'nylon':24,'alloy screws':24,'shell':24}
    if counts!=expected:raise ValueError('Authentic supplier body register differs: '+repr(counts))
