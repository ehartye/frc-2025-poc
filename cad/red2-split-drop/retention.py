"""Static and upset challenges for the distal transverse shoes and coupled drive."""
from pathlib import Path
import hashlib
import json
import math

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/split-drop-native'
def read(file):return json.loads(file.read_bytes())
def sha(file):return hashlib.sha256(file.read_bytes()).hexdigest()
def beam(parts,F,a,b):
    E=69000  # N/mm²; provisional elastic 6061-T6 value
    area=sum(width*height for width,height,z in parts)
    center=sum(width*height*z for width,height,z in parts)/area
    inertia=sum(width*height**3/12+width*height*(z-center)**2 for width,height,z in parts)
    top=max(z+height/2 for width,height,z in parts)
    return dict(areaMm2=area,inertiaMm4=inertia,loadN=F,stressMPa=F*a*b/(a+b)*max(center,top-center)/inertia,
                loadPointDeflectionMm=F*a*a*b*b/(3*E*inertia*(a+b)))
def main():
    spec=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    if sha(ROOT/spec['file'])!=spec['sha256']:raise ValueError('Changed full source')
    receipt=read(OUT/'r2-load-02.json')
    if sha(OUT/'red2-split-drop-fixture-r2.f3d')!=receipt['sha256']:raise ValueError('Changed upper fixture')
    r=read(OUT/'r1-gamepiece-control.json')['reactions']
    F=r['oneSideJamFrontN']
    loads={}
    # Full lateral force on ONE rail at its parked Y=116 within the 480 mm span.
    for kind,parts in [('R2-flat',[(28,8,4)]),('R3-12mm-lips',[(38,4,2),(6.8,4.6,6.3),(13,12,14.6)]),
                       ('R3-20mm-lips',[(38,4,2),(6.8,4.6,6.3),(13,20,18.6)])]:
        loads[kind]={'jamAtClosed':beam(parts,F,356,124),'jamNearRelease':beam(parts,F,396.8125,83.1875),
                     'conservativeCenter':beam(parts,F,240,240)}
    rpin=10.16/1000
    # REV's 3:1 cartridge drawings give 2.89:1, 4:1 gives 3.61:1.
    gearing=2.89*3.61
    mass=.816466266;G=9.80665;normal=mass*G
    # One stalled pinion with escaped teeth must be reported as unsafe.
    remainingLever=.110
    output=dict(authority=spec['sha256'],fixture=receipt['sha256'],concentratedFrontJawN=F,beams=loads,
        mechanism=dict(pressureAngleDeg=14.5,diametralPitch=20,teeth=16,pitchRadiusMm=10.16,oneSideReleaseTravelMm=40.8125,
            fullTravelMm=90,freeAirTravelAfterContactMm=49.1875,driverRpmAt600MmS=.6/rpin*60/(2*math.pi)*gearing,
            gearRatio=gearing,assumedGearboxEfficiency=.75,assumedCurrentLimitA=20,
            assumedMotorTorqueNmPerA=.97/100,
            tangentialForceAtAssumedCurrentLimitN=20*(.97/100)*gearing*.75/rpin,
            loadChallenges=[dict(mu=mu,shockNormalN=r['peakCaptureN'],breakawayN=mu*r['peakCaptureN'],
                neededTorqueNm=mu*r['peakCaptureN']*rpin,estimatedCurrentA=mu*r['peakCaptureN']*rpin/(gearing*.75*.0097)) for mu in [.15,.35]],
            singleJawStockGravityRollNm=normal*remainingLever,singleJawLateralComOutsideSupportMm=remainingLever*1000,
            sensorDistanceDuring20msAt600MmS=12,
            normalLoadPath='Closed shoes carry vertical and pullout loads. Pinion sees rack drag, output radial force, process friction and unexpected blockage. Hardware must inhibit powered opening during impact.'),
        scope='Elastic simply supported beam and motor allocation only. Dynamic capture is not a measured shock on a real installed robot. Shear and local bearing, retaining lip, slot tool/machining tolerance, datum misalignment, bolts, gearbox tooth strength and bracket twist require independent closure.')
    (OUT/'r3-retention-loads.json').write_text(json.dumps(output,indent=2))
    print(output['beams'])
if __name__=='__main__':main()
