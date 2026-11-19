"""Falsifiable bounds for the spring-to-lock, air-to-release winch ratchet."""
from pathlib import Path
import json
import math
HERE=Path(__file__).resolve().parent


def screen():
    lift=json.loads((HERE.parent/"receiver-lift/sizing.json").read_text())
    mass=lift["loaded_design_kg"]
    g=9.80665
    r=(.787*.0254+.003175)/2
    reduction=16
    # Four asymmetric external lands in am-4398 rev 3. No jump across a second tooth allowed.
    pitch=2*math.pi*r/(4*reduction)
    down=.253125
    proof=lift["holding"]["two_times_static_design_Nm"]
    cases=[]
    for ms in [0,10,25,50,100]:
        seconds=ms/1000
        distance=down*seconds+g*seconds**2/2+pitch
        speed=math.sqrt(down**2+2*g*distance)
        energy=mass*speed**2/2
        # Additional 1 mm stopping compliance is an assumption. Peak can exceed the mean.
        stop=.001
        mean_torque=(mass*g+energy/stop)*r
        cases.append({"releaseToPawlMs":ms,"dropBeforeContactMm":distance*1000,"speedMps":speed,
                      "translationalEnergyJ":energy,"meanDrumArrestNmAt1mm":mean_torque,
                      "meanIdealInputNm":mean_torque/reduction})
    return {"loadedMassKg":mass,"proposedLock":"+Y shaft yaw (CCW looking from the +Y output side)",
            "drumRadiusM":r,"inputToDrumRatio":reduction,"externalCatchLands":4,"maxOnePitchMm":1000*pitch,
            "staticProofDrumNm":proof,"staticProofIdealInputNm":proof/reduction,
            "minimumTangentialAtOutsideRadiusN":proof/(reduction*.031/2),
            "idealBoreForceAt50psiN":50*6894.75729*math.pi*.012**2/4,
            "cylinderSweptMl":math.pi*.012**2/4*.010*1e6,
            "cases":cases,"assemblyHoldingRatingNm":None,"measuredCaptureMs":None,
            "physicalPowerLossCatchVerified":False}


if __name__=="__main__":
    (HERE/"pawl.json").write_text(json.dumps(screen(),indent=2)+"\n")
