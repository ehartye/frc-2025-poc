"""Screen the deployed opposed folding stops against the common physical trough and brake load."""
import argparse
import hashlib
import itertools
import json
import math
import sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/red2-integrated'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def corners(low,high):
    return np.array(list(itertools.product(*zip(low,high))))


def screen(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from showcase_contacts import load_solids
    from mechanical_motion import bounds
    master_file=ROOT/'cad/fusion-authority.json'
    master=json.loads(master_file.read_bytes())['source']['archive']
    assert digest(ROOT/master['file'])==master['sha256']
    candidate=json.loads((ROOT/'experiments/red2-successor/blockouts.json').read_bytes())['layouts']['tip-tray']
    target_file=ROOT/'experiments/six-wheel/mission/target.json'
    target=json.loads(target_file.read_bytes())
    docks_file=ROOT/'experiments/swerve-drive/low-placement/clearance.json'
    docks=json.loads(docks_file.read_bytes())['cases']
    sources={}
    rosters={}
    for sku,count in [('REV-41-1097',5),('REV-41-1363',1),('REV-11-1144',21)]:
        path=OUT/'alternative-supplier'/(sku+'.step')
        bodies,surfaces=load_solids(path)
        assert len(bodies)==count and not surfaces
        rosters[sku]=[bounds(s) for s in bodies.values()]
        sources[sku]=dict(file=path.relative_to(ROOT).as_posix(),sha256=digest(path),solids=count,
                          supplierUrl='https://www.revrobotics.com/content/cad/'+sku+'.STEP')
    shaft_y=-10.05
    station=370-55+57.15+4
    pipe_half=301.625/2
    left_axis_y=208.
    tongue_to_axis=103.
    # Z is perpendicular to the deployed tray's support surface. The horn's
    # flat Y0 face is at +76 mm, the raised Y6 boss looks toward the servo.
    # +Y in the vendor horn maps to +tray Z. The provisional axial seat
    # puts the supplier spline tip at the shoulder of the horn counterbore.
    # The supplier model omits actual female spline teeth.
    def solids(angle,side):
        ca,sa=math.cos(angle),math.sin(angle)
        for typ,vertices in [('stop',corners([-4.,-tongue_to_axis,32.],[4.,0.,76.])),
                             ('horn',corners([-19.,-19.,76.],[19.,19.,82.]))]:
            x,y,z=vertices.T
            yield typ,np.column_stack((station+ca*x-sa*y,side*(left_axis_y+sa*x+ca*y),z))
        for index,bb in enumerate(rosters['REV-41-1097']):
            v=corners(bb[:3],bb[3:])
            # Servo faces down. Source Z is already directed toward the
            # housing from its spline. Rotate the in-plane package outboard.
            xx=v[:,0]+station; yy=left_axis_y+v[:,1]-shaft_y
            yield 'servo-body-'+str(index),np.column_stack((xx,side*yy,76.+2.5+43.55+v[:,2]))
    slope=target['floor_slope']
    norm=math.hypot(1.,slope)
    riser_x=min(target['riser_bounds_m'][0][0],target['riser_bounds_m'][1][0])*1000
    records=[]
    step=math.pi/180
    # Expand by maximum swept radius across half a one-degree interval.
    # Rays/panels terminate, so positive infinite-plane gap or forward X
    # separation are conservative bounds even above holes and beyond edges.
    sweep_loss=math.hypot(tongue_to_axis,4.)*step/2
    for sign,case in docks.items():
        for dx,dy,yaw in itertools.product(*zip(*case['enclosure'])):
            for pitch,dz,sector in itertools.product((-.04,.04),(-5.,5.),(0.,360.)):
                cp,sp=math.cos(pitch),math.sin(pitch)
                r=math.radians(12)-pitch
                ca,sa,cy,sy=math.cos(r),math.sin(r),math.cos(yaw),math.sin(yaw)
                lim_floor=float('inf');lim_riser=float('inf');lim_name=None
                for angle_i in range(91):
                    angle=angle_i*step
                    for side in (-1,1):
                        for name,pts in solids(angle,side):
                            x,y,z=pts.T;rx=205*cp-580*sp+ca*x+sa*z
                            xx=dx-490+cy*rx-sy*y
                            zz=dz+205*sp+580*cp-sa*x+ca*z
                            min_normal=np.min(zz-target['floor_intercept_m']*1000-slope*xx)/norm
                            if min_normal<lim_floor:
                                lim_floor=float(min_normal);lim_name=(name,angle_i,side)
                            lim_riser=min(lim_riser,riser_x-max(xx))
                records.append(dict(dock=[sign,dx,dy,yaw,pitch,dz,sector],lowerFloorGapMm=lim_floor-sweep_loss,
                                    lowerRiserForeAftGapMm=lim_riser-sweep_loss,worstFloorBody=lim_name))
    floor=min(records,key=lambda v:v['lowerFloorGapMm']);riser=min(records,key=lambda v:v['lowerRiserForeAftGapMm'])
    # A one-sided restrained load on a specified admitted pipe is a direct
    # wrench at the servo spline. Two equal reactions would halve each demand.
    a=2*9.80665
    force=.816466266*(a+9.80665*math.sin(math.radians(12)))
    stall=13.5*.0980665
    load=dict(projectBrakeBoundMps2=a,oneSideWorstContactYmm=left_axis_y-tongue_to_axis,
              minimumPipeYedgeToHingeMm=left_axis_y-pipe_half,oneSideForceN=force,
              nearEndMomentNm=force*(left_axis_y-pipe_half)/1000,tipMomentNm=force*tongue_to_axis/1000,
              equalSidesAtTipNm=force*tongue_to_axis/2000,manufacturerStallTorqueNm=stall,
              torqueReserveAtEndNm=stall-force*tongue_to_axis/1000)
    result=dict(authority=master,scope='Concept-only deployed gate, field analytic bounds and finite first brake-load case. No native or acquisition-to-score acceptance.',
                sources=sources,target=dict(file=target_file.relative_to(ROOT).as_posix(),sha256=digest(target_file)),
                docks=dict(file=docks_file.relative_to(ROOT).as_posix(),sha256=digest(docks_file)),
                basis=candidate['release'],servoSpline=[0,shaft_y,-43.55],hornShoulderMm=2.5,
                physicalLimits='The REV horn exchange replaces the 25T bore with a smooth cylinder. Spline indexing, real end-screw, joint capacity, mounted supports, servo wire and high-angle intake duty remain open.',
                closedWidthCoverageMm=[left_axis_y-tongue_to_axis,pipe_half],angularStepDegree=1,sweepInflationMm=sweep_loss,
                dockCases=len(records),worstInfinitePlane=floor,worstRiser=riser,loads=load,cases=records)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('plugin',type=Path)
    result=screen(parser.parse_args().plugin)
    (OUT/'folding-end-stop-screen.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    print(result['dockCases'],result['worstInfinitePlane'],result['worstRiser'],result['loads'])
