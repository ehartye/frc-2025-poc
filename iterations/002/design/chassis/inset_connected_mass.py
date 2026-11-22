"""Reconcile named stock against the frozen route budget without charging a shared part twice."""
import collections
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def run(plugin):
    sys.path.insert(0, str(plugin / "scripts"))
    from fusion_authority import register_path, verify
    root = HERE.parents[3]
    register = register_path(root)
    authority = verify(root, register, register.parent / "issue-005/lineage.json")
    inputs = {}

    def read(file):
        data = (HERE / file).read_bytes()
        inputs[file] = hashlib.sha256(data).hexdigest()
        return json.loads(data)

    budget = read("inset-structure-clock.json")
    stock = read("inset-stock-check.json")
    shop = read("inset-chassis-shop.json")
    original = read("inset-mount-purchases.json")
    pan = read("inset-upper-named-mass.json")
    if any(source["authority"] != authority for source in (budget, stock, shop, original)):
        raise ValueError("Changed 552 source")
    from component_shape import volume
    from showcase_contacts import load_solids
    bodies, sheets = load_solids(register.parent / "issue-005/assembly.step")
    if len(bodies) != 552 or sheets:
        raise ValueError("Incomplete source")
    groups = stock["massByGroup"]
    density = 7.86e-6
    counts = collections.Counter(row["sku"] for row in shop["purchases"])
    if counts != {"91251A539": 68, "91251A351": 16, "91251A354": 32}:
        raise ValueError(counts)
    screw = {sku: volume(bodies[original["originals"][sku]["source"]]) * density for sku in counts}
    charge = {}
    def debit(account, groups_used, purchases):
        charge[account] = dict(stockGroups={g: groups[g] for g in groups_used},
                               screws=purchases, kg=sum(groups[g] for g in groups_used)
                               + sum(screw[sku] * n for sku, n in purchases.items()))
        return charge[account]["kg"]

    primary = debit("primary frame and wheel seats", ["perimeter","socket","crossmember","sill",
                          "sill seat","wheel saddle","wheel and sill sleeve"], {"91251A354":32,"91251A351":16})
    decks = debit("electronics sheet stock in 7 kg electrical budget", ["panel"], {})
    backers = debit("fixed connection and fitment allowance", ["cassette","panel backer","panel pedestal",
                    "panel retainer","truss feet","truss backing"], {"91251A539":68})
    tubes = debit("upper structure in 8.2 kg pan budget", ["pan strut"], {})
    assigned = [g for a in charge.values() for g in a["stockGroups"]]
    if sorted(assigned) != sorted(groups) or sum(sum(a["screws"].values()) for a in charge.values()) != 116:
        raise ValueError("Stock or purchase omitted or duplicated")
    previous = sum(budget[k] for k in ("ringGrossCatalogKg","newFourLowerRailsKg","newSillsAndCrossbarsKg",
                                       "eightCompleteSaddleGroupsKg","innerScrewContingencySleevesKg"))
    retired = sum(r["kg"] for r in pan["historicStock"] if r["disposition"].startswith("retired support;"))
    upper_named = pan["stockPlusNamedPurchasedKg"] - retired + tubes
    # Commissioned new steel spacer volume per module, from the actual placed 127 (volume in mm^3).
    measured = json.loads((HERE.parent/"private-inputs/matched-sds/cold.json").read_bytes())
    inputs["../private-inputs/matched-sds/cold.json"] = hashlib.sha256((HERE.parent/"private-inputs/matched-sds/cold.json").read_bytes()).hexdigest()
    labels = [r for r in measured["roster"] if "relieved" in r["path"].lower() and "spacer" in r["path"].lower()]
    if len(labels) != 6:
        raise ValueError([r["path"] for r in measured["roster"] if "spacer" in r["path"].lower()])
    new_spacers = 4 * sum(r["volumeMm3"] * density for r in labels)
    # Axle refacing removes stock; retain the old stock plus a conservative 1 g for the small barrel length change.
    local_change = primary - previous + new_spacers + .001
    masses = {}
    for label, case in budget["cases"].items():
        masses[label] = dict(previousBareKg=case["fixedRobotKg"],bareKg=case["fixedRobotKg"]+local_change,
            R103MarginKg=case["R103MarginKg"]-local_change,
            fieldWithLargestBatteryKg=case["fieldKg"]+local_change+(6.479-case["excludedBatteryAndLimitedBatterySideLeadsKg"]))
    return dict(authority=authority,inputs=inputs,actualStockCount=len(stock["parts"]),plannedScrewCount=sum(counts.values()),
        sourceScrewKg=screw,sourceCorrectionSteelKg=new_spacers,allocation=charge,previousPrimaryKg=previous,
        previousFixedJoiningAllowanceKg=budget["fullRetainedJoiningAndBracketsAllowanceKg"],
        remainingFixedJoiningAllowanceKg=budget["fullRetainedJoiningAndBracketsAllowanceKg"]-backers,
        electricalAllowanceAfterSheetsKg=7-decks,retiredIndependentTowerSupportKg=retired,
        panNamedInstalledOrTransferredBudgetKg=upper_named,panRemainingForUnweighedArticlesKg=pan["allocationKg"]-upper_named,
        previousToCurrentBareKg=local_change,inspectionAndField=masses,panOutstanding=pan["openArticles"],
        scope="Nominal geometry/density. Keep the full remaining fixed, electrical and pan allowances as mass in both cases, the 2 kg high case, and 1 g for the slightly extended axle spacer. The 116 screws are positioned in the shop plan, pending native installation/fit. The stock profile, seams, electrical harness, service travel and upper center of gravity require installed verification. Measured mass and route replay remain separate.")

if __name__=="__main__":
    result=run(Path(sys.argv[1]))
    (HERE/"inset-connected-mass.json").write_text(json.dumps(result,indent=2)+"\n")
