"""Screen parked shaft loads, free play and radial overhang with explicit assumed inputs."""
from math import pi
from fit import HERE, SHAFT, j


def run():
    fit = j.read(HERE/"fit.json")
    # Whole retained shaft plus two drum groups and pulleys, capped by a proposed weighing gate.
    limit_kg = 2.
    axial = limit_kg*6*9.80665
    minor_diameter = 3.0  # minimum admissible diameter at the weakest actually engaged screw section
    area = pi*minor_diameter**2/4
    # Use the hex's circumscribed diameter when bounding the remaining aluminum cheeks.
    cheek = 10.-12.93/(3**.5)
    arm_area = min(k["spacerToArmMm3"] for k in fit["axialContactsAt0p6mm"].values())/(.60-.15)
    # Inward/outward chain gaps, plus play on each side of the pin and the block bore.
    nominal_gap = .15 + (236.538-236.40) + (267.65-267.462) + 2*(2.10-2.)
    result = {
        "shaftNominalKg":fit["volumeMm3"][SHAFT]*7.85e-6,
        "hypotheticalTotalMovingAxialKg":limit_kg,"assumedLateralAccelerationG":6,
        "oneSideAxialForceN":axial,"singlePlaneScrewShearMPa":axial/area,
        "oneAluminumCheekBearingMPa":axial/(minor_diameter*cheek),
        "nativeArmShoulderSectionMm2":arm_area,"nominalArmBearingMPa":axial/arm_area,
        "nominalPackedChainSlackMm":nominal_gap,"fieldShockMeasured":False,
        "shaftRadialComparison":{},"ratedCarryOrActuationLoadN":None}
    # 32 mm from original bearing center to primary drum center, end bores outside load span.
    for force in (100,300,900):
        moment = force*32.
        result["shaftRadialComparison"][str(force)] = {"lineForceN":force,"momentNmm":moment,
            "nominalInscribedRoundStressMPa":32*moment/(pi*12.7**3)}
    j.write(HERE/"loads.json",result)


if __name__=="__main__":
    run()
