"""Reconcile this one installed polymer body against the numbered R4 bill."""

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
MASS = HERE.parent / "mass-closure"


def account():
    receiving = json.loads((MASS / "r4-receiving.json").read_bytes())
    recipe = json.loads((MASS / "recipe.json").read_bytes())
    shelf = recipe["originalMaterial"]
    volume = receiving["installedShelfMm3"]["r6-polymer-facing"]
    if volume != shelf["retainedFaceVolumeMm3"]:
        raise ValueError("The R4 registration and enrolled polymer stock differ")
    original = shelf["materialDensityKgM3"]["polymer"]
    selected = [950, 960]  # Röchling 717982, ASTM D792 guideline (kg/m³).
    if not original[0] <= selected[0] <= selected[1] <= original[1]:
        raise ValueError("Candidate is outside enrolled polymer stock")
    kg_per_density = volume * 1e-9
    mass_range = [value * kg_per_density for value in selected]
    issued = receiving["case"]["issued"]
    # Never book extra credit from an individual guideline property.
    delta = max(0, (selected[1] - original[0]) * kg_per_density)
    return {
        "selectedBody": "r6-polymer-facing",
        "sourceProductId": 717982,
        "nativeVolumeMm3": volume,
        "supplierGuidelineKgM3": selected,
        "previousEnrolledKgM3": original,
        "candidateBodyKg": mass_range,
        "issuedBudgetHeadroomKg": issued["r103MarginKg"],
        "upperIncrementIfBaselineFacingWasLightestKg": delta,
        "headroomUnderExtraIndependentChargeKg": issued["r103MarginKg"] - delta,
        "headroomWithoutRefundOfEnrolledRangeKg": issued["r103MarginKg"],
        "scaleMassKg": None,
    }


if __name__ == "__main__":
    print(json.dumps(account(), indent=2))
