"""Screen the six-point joint group, sleeve, end-wall webs and local flange in SI/mm units."""
from itertools import product
import math
import numpy as np
from kit import AXES, HERE, ROOT, j

G=9.80665
# Whole-machine envelope includes bumpers and cargo. It is intentionally much heavier than
# the fraction carried directly by the deck. These loading directions are design exercises.
MASS=67.
EAL=68300.
ESTEEL=200000.
PRELOAD=850.
PROOF=580.  # MPa, project trial minimum: verify fastener certificate and actual nut
THREAD_AREA=8.78


def joint_group(axes,force,position):
    at=np.array(axes,dtype=float)
    center=at.mean(axis=0)
    xy=at-center
    position=np.array(position)-[center[0],center[1],150.]
    moment=np.cross(position,force)
    system=np.stack([np.ones(len(at)),xy[:,1],-xy[:,0]])
    normal=np.linalg.lstsq(system,[force[2],moment[0],moment[1]],rcond=None)[0]
    shear=force[:2]/len(at)+np.stack([-xy[:,1],xy[:,0]],axis=1)*moment[2]/np.sum(xy**2)
    return {"normalMaxN":float(max(normal)),"normalMinN":float(min(normal)),
            "shearMaxN":float(max(np.linalg.norm(shear,axis=1))),"momentNm":(moment/1000).tolist()}


def envelope(omit):
    values=[]
    for i in ([None] if not omit else range(6)):
        active=[p for n,p in enumerate(AXES) if n!=i]
        for sx,sy,az in product((-1,1),(-1,1),(-3,3)):
            for angle in range(0,360,2):
                theta=math.radians(angle)
                row=joint_group(active,np.array([1000*math.cos(theta),1000*math.sin(theta),az*MASS*G]),[100*sx,100*sy,500])
                values.append((i,row))
    n=max(values,key=lambda item:item[1]["normalMaxN"])
    # Linear responses to (cos,sin) have a peak at most sec(1 degree) times
    # the largest two-degree sample. Use both vertical signs to bound the offset term.
    angular_bound=1/math.cos(math.radians(1))
    return {"maximumTieTensionN":n[1]["normalMaxN"]*angular_bound,"missingAtTensionPeak":n[0],
            "maximumTieShearN":max(r["shearMaxN"] for _,r in values)*angular_bound,
            "peakCompressionN":max(-r["normalMinN"] for _,r in values)}


def section(case):
    span=16.
    area=math.pi/4*(8**2-4.4**2)
    inertia=math.pi/64*(8**4-4.4**4)
    sleeve_k=EAL*area/97.95
    bolt_k=ESTEEL*THREAD_AREA/117.5
    tension=case["maximumTieTensionN"]
    shear=case["maximumTieShearN"]
    f=PRELOAD+tension
    # Treat flange as the sole beam spanning both 2 mm sidewall webs. Conservative
    # center point load, net width across bore; omit parallel 2 mm tube-floor stiffness.
    width=22-4.4
    local_i=width*4**3/12
    # Under clamp preload the sleeve returns load directly to the upper wall.
    # Bound lower foot and tube sides separately using the entire preload + external load.
    return {"sleeveAxialStiffnessNPerMm":sleeve_k,"screwAxialStiffnessNPerMm":bolt_k,
            "sleeveElasticShorteningMmAtPreload":PRELOAD/sleeve_k,
            "sleeveEulerLoadN":math.pi**2*EAL*inertia/97.95**2,"sleeveCompressionMPa":f/area,
            "footStripStressMPa":f*span/4*2/local_i,"footDeflectionMm":f*span**3/(48*EAL*local_i),
            "footBearingOnTwoWebsMPa":f/(2*1.75*22),"singleWebAdverseBearingMPa":f/(1.75*22),
            "screwVonMisesMajorThreadMPa":math.hypot(f/THREAD_AREA,math.sqrt(3)*shear/THREAD_AREA),
            "screwTrialProofMPa":PROOF,"thinUpperWallBearingMPa":shear/(4*2),
            "deckNetEdgeShearOutMPa":shear/(2*(7-2.25)*4),
            "jointSeparationNAt850NModelPreload":PRELOAD/(1-bolt_k/(bolt_k+sleeve_k)),
            "thinRoundFootControlMPa":tension*span/4/(12*2**2/6)}


def run():
    full=envelope(False)
    failed=envelope(True)
    impacts={}
    source=ROOT/"shelf-contact/load.json"
    for record in j.read(source):
        key=record["case"]
        force=record["peakRobotReactionXYZn"]
        position=record["peakNormalCenterRobotMm"]
        impulse=abs(record["pulseImpulseRobotXYZns"][2])
        r=joint_group(AXES,np.array(force),position)
        projection=[position[i]+(150-position[2])*force[i]/force[2] for i in (0,1)]
        impacts[key]={**r,"forceN":force,"appliedAtMm":position,"lineAtDeckMm":projection,
                      "pulseSeconds":record["endFromLipS"]-record["startFromLipS"],
                      "modelVerticalImpulseNs":impulse,"wholeRobotRigidDeltaVMps":impulse/MASS}
    def integral(case):
        shear=case["maximumTieShearN"]
        force=PRELOAD+case["maximumTieTensionN"]
        root=math.pi*3.95**2/4
        ring=math.pi*(8**2-5.4**2)/4
        web_i=(22-5.4)*4**3/12
        stiffness=1/(90/(71000*ring)+12/(71000*math.pi*8**2/4)+16/(71000*root))
        return {"netRootMm2":root,"maximumPreloadPlusTensionN":force,
                "nominalCombinedAtRootMPa":math.hypot(force/root,math.sqrt(3)*shear/root),
                "stressAtTrialNotchFactor1_6MPa":1.6*math.hypot(force/root,math.sqrt(3)*shear/root),
                "minimumLotYieldMPa":400,"threadFlankShearMPaAt4mmMetal":force/(.5*math.pi*3.95*4),
                "footBendingMPa":force*16/4*2/web_i,"footDeflectionMm":force*16**3/(48*71000*web_i),
                "upperWallBearingMPa":shear/(5*2),"deckEdgeShearOutMPa":shear/(2*(7-2.75)*4),
                "anchorAxialNPerMm":stiffness,"extraUpliftTravelAtZeroLowerPreloadMm":.10+case["maximumTieTensionN"]/stiffness,
                "physicalYieldFatigueAndSeparation":None}
    answer={"unmeasuredTrialPreloadN":PRELOAD,"fullGroup":full,"fullSection":section(full),
            "oneTieUnavailable":failed,"adverseSection":section(failed),"contact":impacts,
            "contactSourceSha256":j.digest(source),"integralAllSix":integral(full),"integralMissingOne":integral(failed),
            "calibratedImpulseOrAllowable":None}
    j.write(HERE/"load.json",answer)
    return answer


if __name__=="__main__": run()
