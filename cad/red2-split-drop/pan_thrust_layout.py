"""Screen a bilateral idler thrust cartridge against the cold R2 pan and foundation.

The cartridge is a machining study. Purchased shapes retain their source profiles.
Thread minimum engagement and the strength of all constituents are separate duties.
"""
import itertools
import json
import sys
from pathlib import Path

import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

from output_mount import ROOT, digest, place, read
from pan_mechanics import rotation

DATA = ROOT / 'artifacts/red2-integrated'
AXIS_X, AXIS_Z = 205., 580.


def cylinder(point, direction, radius, length):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*point), gp_Dir(*direction)), radius, length).Shape()


def along_axis(u, r, length):
    return cylinder((AXIS_X, -u, AXIS_Z), (0, -1, 0), r, length)


def transform(R, t):
    return np.column_stack((np.asarray(R), np.asarray(t)))


def translate(body, vector):
    return place(body, transform(np.eye(3), vector))


def boolean(op, first, second):
    alg = op(first, second)
    alg.Build()
    if not alg.IsDone():
        raise ValueError('Stock construction failed')
    return alg.Shape()


def subtract(body, tool):
    return boolean(BRepAlgoAPI_Cut, body, tool)


def fuse(first, second):
    return boolean(BRepAlgoAPI_Fuse, first, second)


def study(plugin, rated, serviced, piloted, prepare=False):
    sys.path.insert(0, str(plugin / 'scripts'))
    from mechanical_motion import bounds, common, distance
    from showcase_contacts import box_gap, load_solids

    output = DATA / 'pan-thrust'
    old = DATA / 'pan-native'
    receipt = read(old / 'pan-r2-foundation-install-02.json')
    master = read(ROOT / 'cad/fusion-authority.json')['source']['archive']
    archive = old / 'integrated-340mm-pan-r2.f3d'
    step = archive.with_suffix('.step')
    if not (digest(ROOT / master['file']) == receipt['authority']['archiveSha256']
            and digest(archive) == receipt[archive.stem] and digest(step) == receipt['derivativeSha256']):
        raise ValueError('Parent issue changed')
    labels = {}
    existing, other = load_solids(step, labels)
    if len(existing) != 294 or other:
        raise ValueError('Incomplete parent assembly')
    previous = read(old / 'critical-interfaces-r2.json')
    if previous['idlerAxialFreedom']['maximumCommonMm3'] != 0 or previous['idlerAxialFreedom']['sweepMm'] != [-3,3]:
        raise ValueError('Lost the original free-shaft control')
    owner = lambda name: next(k for k in existing if name in labels[k]['occurrencePath'])
    shaft_name = owner('Idler headed stub hex shank')
    socket_name = owner('Starboard driven hex socket')
    tower_name = owner('Starboard bearing tower')
    # Work in flat pan coordinates so radial machining and screws rotate together.
    stub = place(existing[shaft_name], rotation(-70))
    socket = place(existing[socket_name], rotation(-70))
    tower = existing[tower_name]

    source = output / ('6655k17 assembly.STEP' if rated else '1613-0514-0006 assembly.STEP')
    bearing, extras = load_solids(source)
    if len(bearing) != 11 or extras:
        raise ValueError('Lost a supplier bearing constituent')
    races = sorted([(bounds(s)[1],k,s) for k,s in bearing.items() if 'washer' in k])
    if len(races) != 2:
        raise ValueError('Missing authentic washer pair')
    (_, loose_label, loose), (_, _, tight) = races
    top = bounds(tight)
    bottom = bounds(loose)
    center = np.array([(top[0]+top[3])/2, top[4], (top[2]+top[5])/2])
    height = top[4] - bottom[1]
    # Native races differ from nominal catalog stack. The use of cold-source
    # planes defines a geometry trial, not production preload.
    if abs(height - (6.2992 if rated else 4.84)) > .02:
        raise ValueError('Unexpected as-drawn stack')
    gap = .10
    shoulder = 205. if rated else 202.
    race_opening = 7.15 if rated else 4.5
    shaft_radius = 6.33 if rated else 2.98
    abutment_radius = 11.35 if rated else 6.2
    plate0, plate1 = shoulder + height + gap/2, shoulder + height + gap/2 + 4.
    cap = plate1 + height + gap/2
    fixed, moving, groups = {}, {}, {}
    for prefix, start, R in [('inboard',shoulder,np.eye(3)),('outboard',cap,np.diag([1.,-1.,-1.]))]:
        offset = np.array([AXIS_X,-start,AXIS_Z]) - R @ center
        for k, raw in bearing.items():
            # A cage/ball can orbit. For installed clearance we keep the
            # supplier phase; complete loaded rolling kinematics are open.
            fixed[prefix+' '+k] = place(raw, transform(R,offset))
            groups[prefix+' '+k] = prefix
    # Fix the opposing clearance races against a stationary plate supported at four ears.
    plate = BRepPrimAPI_MakeBox(gp_Pnt(178,-plate1,563),gp_Pnt(232,-plate0,597)).Shape()
    plate = subtract(plate, along_axis(plate0-.5,race_opening,5))
    seat_relief = 3. if serviced else 0.
    seat_u = plate1-seat_relief
    if piloted:
        # Register each loose housing washer on its OD, allowing axial travel
        # up to its real contact plane without riding on the ball cage.
        lip_depth = .60
        for u0 in (plate0-lip_depth,plate1):
            lip = subtract(along_axis(u0,13.6,lip_depth),along_axis(u0-.05,11.95,lip_depth+.1))
            plate = fuse(plate,lip)
    fixed_screws = []
    screwfiles = {}
    def screw(length):
        f = DATA / 'alternative-supplier' / ('M3X'+str(length)+'.STEP')
        screwfiles[length] = digest(f)
        purchase, incomplete = load_solids(f)
        (s,) = purchase.values()
        if incomplete:
            raise ValueError('Invalid screw geometry')
        return s
    mount_length = 20
    through = screw(mount_length)
    holes = list(itertools.product([184.,226.],[570.,590.]))
    for i,(x,z) in enumerate(holes,1):
        point = (x,-198.15,z)
        boss = cylinder(point,(0,-1,0),4.5,plate0-198.15)
        plate = fuse(plate,boss)
        plate = subtract(plate,cylinder((x,-198.,z),(0,-1,0),1.7,plate1-198.+.5))
        if seat_relief:
            plate = subtract(plate,cylinder((x,-seat_u,z),(0,-1,0),3.1,seat_relief+.1))
        tower = subtract(tower,cylinder((x,-190.,z),(0,-1,0),1.5,10))
        # Source screw head seats at z=-20, tip at z=0. +z becomes +Y.
        R = np.array([[1,0,0],[0,0,1],[0,-1,0]])
        key = 'Bracket screw '+str(i)
        fixed[key] = place(through,transform(R,[x,-seat_u+20,z]))
        fixed_screws.append(key)
    fixed['Stationary diaphragm with four bosses'] = plate
    fixed['Drilled starboard bearing tower'] = tower
    # Hard shoulder and an axial cap on a turned journal capture both thrust packs.
    # A rigid shaft end supports the cap. Tightening the axial screw cannot
    # reduce clearance by freely drawing the cap through the bearing races.
    shim_thickness = .10 if serviced else 0.
    journal_end = cap - shim_thickness
    journal = along_axis(shoulder,shaft_radius,journal_end-shoulder)
    shank = fuse(stub,along_axis(202.,abutment_radius,shoulder-202.)) if rated else stub
    shank = fuse(shank,journal)
    shank = subtract(shank,along_axis(journal_end-10.,1.5,11))
    retainer = subtract(along_axis(cap,abutment_radius,3.),along_axis(cap-.1,1.65,3.2))
    cap_screw = screw(10)
    R = np.array([[1,0,0],[0,0,1],[0,-1,0]])
    moving['End cap screw'] = place(cap_screw,transform(R,[AXIS_X,-cap-3+10,AXIS_Z]))
    moving['Hardened removable end cap'] = retainer
    if shim_thickness:
        moving['Match-ground axial shim'] = subtract(along_axis(journal_end,5.5,shim_thickness),along_axis(journal_end-.1,1.7,shim_thickness+.2))
    # A removable radial screw attaches moving socket to shaft, in double shear.
    cutter = cylinder((205,-177,564),(0,0,1),1.65,24.)
    shank = subtract(shank,cutter)
    socket = subtract(socket,cutter)
    socket = subtract(socket,cylinder((205,-177,588),(0,0,1),1.5,10))
    moving['Cross screw'] = translate(screw(30),[205,-177,595])
    moving['One-piece stepped shaft'] = shank
    moving['Cross-drilled moving socket'] = socket

    original = {k:v for k,v in existing.items() if k not in [tower_name,shaft_name,socket_name]}
    mobile_old = {k for k in original if '340 mm pan rotating installation' in labels[k]['occurrencePath']}
    out = dict(authority=master,parentArchive=digest(archive),parentStep=digest(step),sourceBearing=digest(source),
        vendor=('ServoCity/McMaster 6655K17' if rated else 'goBILDA 1613-0514-0006'),sourceDate=('2020-06-15' if rated else '2020-09-22'),vendorShapes=len(bearing),
        source=('https://www.servocity.com/content/step_files/6655k17%20assembly.zip' if rated else 'https://www.gobilda.com/content/step_files/1613-0514-0006.zip'),
        sourceRaces=dict(asDrawnStackMm=height,catalogStackMm=(.249*25.4 if rated else 5.)),
        importedCoordinateSystem='mm. u = negative robot Y. This scoped geometry uses the R2 shaft axis x205 z580.',
        stackPlanesUmm=[shoulder,plate0,plate1,cap],candidateTotalEndFloatMm=gap,
        rigidCapSeatUmm=journal_end, shaftDiameterMm=2*shaft_radius, abutmentDiameterMm=2*abutment_radius,
        capScrewEngagementMm=7., mountingTowerEngagementMm=max(0.,min(6.3,20.-(seat_u-198.15))),towerScrewSeatUmm=seat_u,selectedShimMm=shim_thickness,
        screwSources=screwfiles,cases={})
    if prepare:
        if not (piloted and serviced and rated):
            raise ValueError('Prepare only the source-qualified mechanical variant')
        from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
        from OCP.IFSelect import IFSelect_RetDone
        from OCP.BRepGProp import BRepGProp
        from OCP.GProp import GProp_GProps
        case=read(output/'r3-retainer-6655k17-pilots.json')
        if any(row['positiveCommonMm3'] for row in case['cases'].values()) or any(t['hitsMm3'] for row in case['cases'].values() for t in row['straightToolApproach'].values()):
            raise ValueError('Address installed or access interference before preparing stock')
        folder=output/'r3-native-inputs'
        folder.mkdir(exist_ok=True)
        def save(shape,filename):
            file=folder/filename
            if not file.exists():
                writer=STEPControl_Writer()
                writer.Transfer(shape,STEPControl_AsIs)
                if writer.Write(str(file))!=IFSelect_RetDone:raise ValueError(filename)
            (reopened,),extras=tuple(load_solids(file)[0].values()),load_solids(file)[1]
            props=GProp_GProps();BRepGProp.VolumeProperties_s(shape,props)
            other=GProp_GProps();BRepGProp.VolumeProperties_s(reopened,other)
            if extras or abs(props.Mass()-other.Mass())>max(.005,abs(props.Mass())*1e-6):
                raise ValueError('Stale or altered neutral '+filename)
            return dict(file=file.relative_to(ROOT).as_posix(),sha256=digest(file),volumeMm3=props.Mass(),boundsMm=bounds(shape))
        shop_names=['One-piece stepped shaft','Cross-drilled moving socket','Drilled starboard bearing tower','Stationary diaphragm with four bosses','Hardened removable end cap','Match-ground axial shim']
        out['stock']={name:save((moving|fixed)[name],f'stock-{n}.step') for n,name in enumerate(shop_names)}
        out['bearingPlacements']={}
        for prefix,start,R in [('inboard',shoulder,np.eye(3)),('outboard',cap,np.diag([1.,-1.,-1.]))]:
            offset=np.array([AXIS_X,-start,AXIS_Z])-R@center
            for n,(key,supplier) in enumerate(bearing.items()):
                mask=(prefix+' '+key)
                group='fixed' if key==loose_label else ('moving' if 'washer' in key else 'cage')
                out['bearingPlacements'][mask]=dict(sourceBody=key,sourceStepSha256=digest(source),originalTransfer=save(supplier,f'source-6655K17-{n:02}.step'),matrixMm=transform(R,offset).tolist(),group=group,material='nylon' if 'housing' in key else 'steel')
        out['screwPlacements']={}
        positions={**{k:(20,transform([[1,0,0],[0,0,1],[0,-1,0]],[x,-seat_u+20,z]),'fixed') for k,(x,z) in zip(fixed_screws,holes)},'End cap screw':(10,transform([[1,0,0],[0,0,1],[0,-1,0]],[AXIS_X,-cap-3+10,AXIS_Z]),'moving'),'Cross screw':(30,transform(np.eye(3),[205,-177,595]),'moving')}
        for k,(length,M,group) in positions.items():
            f=DATA/'alternative-supplier'/f'M3X{length}.STEP'
            out['screwPlacements'][k]=dict(source=f.relative_to(ROOT).as_posix(),sourceSha256=digest(f),matrixMm=M.tolist(),group=group,material='steel alloy not graded in supplier kit')
        out['layoutProofSha256']=digest(output/'r3-retainer-6655k17-pilots.json')
        out['fixedRacePilot']=case['fixedRacePilot']
        out['widths']=case['widths']
        out['expectedSolids']=len(original)+len(shop_names)+len(out['bearingPlacements'])+len(out['screwPlacements'])
        (output/'r3-native-plan.json').write_text(json.dumps(out,indent=2)+'\n')
        return
    clearance_tools = {}
    if serviced:
        # The supplier model's screw hex AF is 2.5 mm. Two bounded access gauges:
        # full circumscribed key steel, plus a wider straight socket handle.
        def gauges(origin,axis):
            O=np.array(origin);d=np.array(axis)
            bit=cylinder(O,d,1.45,65.)
            sleeve=cylinder(O+55*d,d,10.,45.)
            return fuse(bit,sleeve)
        for name,(x,z) in zip(fixed_screws,holes):
            clearance_tools[name] = (gauges((x,-seat_u-3,z),(0,-1,0)),False)
        clearance_tools['End cap screw'] = (gauges((AXIS_X,-cap-6,AXIS_Z),(0,-1,0)),True)
        clearance_tools['Cross screw'] = (gauges((205,-177,562),(0,0,-1)),True)
    cache = {}
    moving_keys = set(moving) | mobile_old
    for state in receipt['cold']['poses']:
        name = state['pose']; angle = state['angleDeg']
        model = {k: place(v,rotation(angle-70)) if k in mobile_old else v for k,v in original.items()}
        mounts = dict(fixed)
        for k,v in moving.items():
            mounts[k] = place(v, rotation(angle))
        shapes = model | mounts
        boxes = {k:bounds(v) for k,v in shapes.items()}
        hits={}; candidates=0
        for a,b in itertools.combinations(shapes,2):
            if (a not in mounts and b not in mounts) or (a in groups and groups.get(a)==groups.get(b)):
                continue
            if box_gap(boxes[a],boxes[b])>.001:
                continue
            candidates += 1
            # Intersections are invariant to a shared rigid transform. Reuse
            # only an identical relative pose, including the actual stow reset.
            pose_key = angle if ((a in moving_keys) != (b in moving_keys)) else 'same-transform'
            key = (a,b,pose_key)
            if key not in cache:
                cache[key] = common(shapes[a],shapes[b])
            vol = cache[key]
            if vol > .001:
                hits[a+' :: '+b] = vol
        out['cases'][name] = dict(neighbors=len(model),candidatePairs=candidates,positiveCommonMm3=hits)
        if serviced:
            access={}
            for target,(gauge,mobile) in clearance_tools.items():
                tool=place(gauge,rotation(angle)) if mobile else gauge
                b=bounds(tool)
                near={k:v for k,v in shapes.items() if k!=target and box_gap(b,boxes[k])<.1}
                overlaps={k:common(tool,v) for k,v in near.items()}
                access[target]=dict(checkedOther=len(shapes)-1,near=len(near),hitsMm3={k:v for k,v in overlaps.items() if v>.001})
            out['cases'][name]['straightToolApproach']=access
        print(name, candidates, hits, flush=True)
    # Bearing faces closest to the diaphragm form positive axial stops at either sign.
    for pref, sign in [('inboard',1),('outboard',-1)]:
        k = next(key for key in fixed if key.startswith(pref) and key.endswith(loose_label))
        race = fixed[k]
        out.setdefault('stops',{})[pref] = [
            dict(moveUmm=sign*d, gapMm=distance(translate(race,[0,-sign*d,0]),plate),
                 commonMm3=common(translate(race,[0,-sign*d,0]),plate))
            for d in (0.,gap/2,gap/2+.25)]
    out['limitations']=['Preliminary engineering geometry, unsaved in Fusion.',
        ('The 6655K17 dynamic catalog limit is 200 lbf, 889.64 N. Shock, indentation, wear, duty and combined radial/axial qualification are open.' if rated else 'No published rating for this exact bearing. Do not transfer another article rating.'),
        'The source exchange differs from the catalog thickness. Selective shimming or finish-facing the positive shaft end, permissible end float and measured assembled drag require verification.',
        'Threaded holes use the outer thread envelope. Bolt preload, root engagement, pullout, fatigue and combined shaft bending/torsion remain unqualified.',
        'Ball orbit, complete installation/service path and transferring the stationary tower load into the chassis require separate verification.']
    if piloted:
        out['fixedRacePilot']=dict(sourceWasherDiameterMm=top[3]-top[0],pilotDiameterMm=23.9,lipDepthMm=.6,sourceClearanceRadialMm=(23.9-top[3]+top[0])/2)
        out['widths']=dict(shaftPilotDiameterMm=2*shaft_radius,sourceWasherIdMm=12.827,journalDiametralGapMm=12.827-2*shaft_radius)
    if serviced:
        out['toleranceProcess']='Measure actual installed race stack and parallel faces first. Match the removable stepped shaft-end shim to the measured assembly. A dial-indicator reversal and loaded drag trial are required; the catalog-minus-source mismatch for two packs is 0.0508 mm.'
    filename='r3-retainer-6655k17-pilots.json' if piloted else 'r3-retainer-6655k17-service.json' if serviced else ('r3-retainer-6655k17.json' if rated else 'r3-retainer-layout.json')
    (output/filename).write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps(dict(stack=out['sourceRaces'],stops=out['stops'],cases=out['cases']),indent=2))

if __name__=='__main__':
    study(Path(sys.argv[1]), len(sys.argv)>2 and sys.argv[2]=='6655K17', '--service' in sys.argv[3:], '--pilots' in sys.argv[3:], '--prepare' in sys.argv[3:])
