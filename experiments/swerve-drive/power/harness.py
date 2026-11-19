"""Receiving mass limit for additional conductors, linked to the electrical lead budgets."""

import math
from electrical import HERE
from screen import write

# Geometry and density are incoming build limits, not vendor properties or a measured installation.
COPPER_MM2 = {10: 5.26, 12: 3.31, 14: 2.08, 16: 1.31, 22: 0.326}
MAX_OD_MM = {10: 5.5, 12: 4.7, 14: 4.0, 16: 3.3, 22: 2.1}


def kg_per_m(awg):
    metal = COPPER_MM2[awg]
    jacket = math.pi * (MAX_OD_MM[awg] / 2) ** 2 - metal
    return 0.001 * (metal * 8.96 + jacket * 1.50)


def calculate():
    # Only extension wire is added here. Controller and motor pigtails are already
    # in the article mass. Keep old wire budget in full as an adverse no-credit case.
    runs = {
        "driveSupply": (4 * 2, 1.55, 10),
        "steerSupply": (4 * 2, 1.65, 14),
        "driveThreePhaseExtension": (4 * 3, 0.40, 12),
        "steerThreePhaseExtension": (4 * 3, 0.15, 16),
        "CANPairExtensions": (2, 3.2, 22),
    }
    terms = {
        k: dict(conductors=count, metresEach=l, awg=g, maximumKg=count * l * kg_per_m(g))
        for k, (count, l, g) in runs.items()
    }
    # Four relocated motor/absolute adapter sets and cable retainers require a
    # weighed aggregate. Gearbox-local encoder adaptors are inside complete modules.
    reserve = {
        "cutEndsTerminationsAndProtectionKg": 0.20,
        "sensorExtensionsAndCANAdaptersKg": 0.12,
        "sleeveStrainReliefTiesKg": 0.14,
    }
    upper = sum(r["maximumKg"] for r in terms.values()) + sum(reserve.values())
    return dict(
        leadAllowances=terms,
        accessoryUpperLimits=reserve,
        totalNoWireRemovalCreditUpperKg=upper,
        copperAreaMm2=COPPER_MM2,
        maxInsulatedDiameterMm=MAX_OD_MM,
        maximumJacketDensityGPerCm3=1.50,
        installGeometryQualification=None,
        assumptions="No deduction for original drive wiring; separately weighed incoming cable/wiring targets, no bracket stock.",
    )


if __name__ == "__main__":
    write(HERE / "harness.json", calculate())
