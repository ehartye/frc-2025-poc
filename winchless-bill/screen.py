"""Screen a proposed one-row-per-site remnant schedule against the issued mass envelope."""

import argparse
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent


def blank(stock):
    return {
        "allOtherPrimaryHardware": "omitted",
        "actualScaleKg": None,
        "sites": {
            name: {
                "residualSizeMm": [0, 0, 0],
                "maxDensityKgM3": stock["permittedMaxDensityKgM3"],
                "throughSets": 0,
            }
            for name in stock["retainedStock"]
        },
    }


def evaluate(schedule, stock, margin_before_residual):
    if set(schedule) != {"allOtherPrimaryHardware", "actualScaleKg", "sites"}:
        raise ValueError("Unknown or missing schedule field; account for every item")
    if schedule.get("allOtherPrimaryHardware") != "omitted" or set(
        schedule.get("sites", {})
    ) != set(stock["retainedStock"]):
        raise ValueError("Declare every stock site and all other primary hardware")
    if schedule.get("actualScaleKg") is not None:
        raise ValueError("This prebuild screen accepts modeled dimensions only")
    total = 0.0
    for name, row in schedule["sites"].items():
        if set(row) != {"residualSizeMm", "maxDensityKgM3", "throughSets"}:
            raise ValueError("Unsupported hardware field at " + name)
        dims = row.get("residualSizeMm")
        if (
            not isinstance(dims, list)
            or len(dims) != 3
            or any(
                type(x) not in (int, float) or not math.isfinite(x) or x < 0
                for x in dims
            )
        ):
            raise ValueError("Three finite nonnegative dimensions required: " + name)
        if any(x == 0 for x in dims) and any(x != 0 for x in dims):
            raise ValueError("Use three zeroes for an omitted stock: " + name)
        if any(d > m for d, m in zip(dims, stock["retainedStock"][name]["maxSizeMm"])):
            raise ValueError("Stock exceeds this site's registered envelope: " + name)
        density = row.get("maxDensityKgM3")
        if (
            type(density) not in (int, float)
            or not math.isfinite(density)
            or not 0 < density <= stock["permittedMaxDensityKgM3"]
        ):
            raise ValueError("Material outside the selected density bounds: " + name)
        sets = row.get("throughSets")
        if type(sets) is not int or not 0 <= sets <= stock["throughSetsPerSite"]:
            raise ValueError("Too many or unidentified mounting sets: " + name)
        total += math.prod(dims) * density / 1e9 + sets * stock["throughSetUpperKg"]
    return {
        "proposedResidualUpperKg": total,
        "estimatedR103UpperMarginKg": margin_before_residual - total,
        "physicalScaleMassKg": None,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("schedule", type=Path)
    p.add_argument(
        "--example", action="store_true", help="Create an all-omitted starting fixture"
    )
    p.add_argument("--matched-shelf", action="store_true")
    a = p.parse_args()
    stock = json.loads((HERE / "stock.json").read_bytes())
    if a.example:
        with a.schedule.open("x", encoding="utf8") as f:
            json.dump(blank(stock), f, indent=2)
    else:
        key = "densityMatchedRemovalKg" if a.matched_shelf else "noCutCreditKg"
        case = json.loads((HERE / "straight-guide.json").read_bytes())[key]
        print(
            json.dumps(
                evaluate(
                    json.loads(a.schedule.read_bytes()),
                    stock,
                    case["upperResidualBudgetKg"],
                ),
                indent=2,
            )
        )
