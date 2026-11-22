"""Create editable pan stock, limiting every cut to the named physical blank."""
import math

from adsk import core, fusion


def extrusion(comp, profile, distance_mm, operation, geom, owner):
    features = comp.features.extrudeFeatures
    if owner is None:
        return features.addSimple(profile, geom.mm(distance_mm), operation)
    args = features.createInput(profile, operation)
    length = fusion.DistanceExtentDefinition.create(geom.mm(abs(distance_mm)))
    direction = (fusion.ExtentDirections.PositiveExtentDirection if distance_mm > 0
                 else fusion.ExtentDirections.NegativeExtentDirection)
    args.setOneSideExtent(length, direction)
    args.participantBodies = [owner]
    return features.add(args)


def machine(comp, part, geom, scoped=True):
    new = fusion.FeatureOperations.NewBodyFeatureOperation
    cut = fusion.FeatureOperations.CutFeatureOperation
    def sections(axis, profiles, name, operation, owner=None):
        first, sign = geom.profile(comp, axis, profiles[0], name + ' start')
        if all(abs(a[k]-b[k]) < 1e-8 for a,b in zip(*profiles) for k in range(3) if k != axis):
            distance = (profiles[1][0][axis] - profiles[0][0][axis]) / sign
            feature = extrusion(comp, first, distance, operation, geom, owner)
        else:
            last, _ = geom.profile(comp, axis, profiles[1], name + ' end')
            args = comp.features.loftFeatures.createInput(operation)
            args.loftSections.add(first)
            args.loftSections.add(last)
            if owner is not None:
                args.participantBodies = [owner]
            feature = comp.features.loftFeatures.add(args)
        feature.name = name
        return feature
    sections(part['axis'], part['sections'], 'Fabricate '+part['id'], new)
    if comp.bRepBodies.count != 1:
        raise ValueError('One new blank required '+part['id'])
    def owner():
        if comp.bRepBodies.count != 1:
            raise ValueError('Stock split before machining '+part['id'])
        return comp.bRepBodies.item(0) if scoped else None
    if part['void']:
        sections(part['axis'],part['void'],'Hollow '+part['id'],cut,owner())
    for i, hole in enumerate(part['bores']):
        start, vector = hole['start'], hole['direction']
        axis = next(k for k,q in enumerate(vector) if abs(q) > .9999)
        plane, sign = geom.plane(comp,axis,start[axis])
        sketch = comp.sketches.add(plane)
        sketch.name = 'Bore circle '+part['id']+' '+str(i+1)
        sketch.sketchCurves.sketchCircles.addByCenterRadius(sketch.modelToSketchSpace(geom.point(start)),hole['radius']/10)
        feature = extrusion(comp, sketch.profiles.item(0), hole['depth']*vector[axis]/sign,cut,geom,owner())
        feature.name = 'Bore '+part['id']+' '+str(i+1)
        sketch.isLightBulbOn = False
    for i, hole in enumerate(part['hexes']):
        x,z = hole['center']
        r = hole['flats']/math.sqrt(3)
        outlines = [[[x+r*math.cos(math.radians(60*j+30)),y,z+r*math.sin(math.radians(60*j+30))]
                     for j in range(6)] for y in (hole['y0'],hole['y1'])]
        sections(1,outlines,'Broach '+part['id']+' '+str(i+1),cut,owner())
    if comp.bRepBodies.count != 1:
        raise ValueError('Unexpected machined stock count '+part['id'])
    comp.bRepBodies.item(0).name = part['id']
