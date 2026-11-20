"""Allocate the installed camera assembly once across purchased units and fit allowances."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MOVING = "experiments/six-wheel/camera-disconnects/mass-conditions.json"
FIXED = "experiments/six-wheel/fixed-camera-route/mass-input.json"
CONVERTER = "https://www.meanwell.com/Upload/PDF/DDR-30/DDR-30-spec.pdf"


def allocate(accounts):
    stages = json.loads((ROOT / MOVING).read_bytes())["groups_kg"]
    receipts = json.loads((ROOT / FIXED).read_bytes())["preservedReceivingKg"]
    starting = float(accounts["unitemized-receiving"]["upperKg"])
    if starting != 5.0 or float(accounts["camera-leads"]["upperKg"]) != 0.275:
        raise ValueError("Changed predecessor receiving account")
    purchased = {}

    def line(name, description, mass, source):
        if name in accounts:
            raise ValueError(("Duplicate receiving line", name))
        accounts[name] = dict(
            instance=name, description=description, upperKg=mass, source=source
        )
        purchased[name] = mass

    line(
        "camera-carrier-links",
        "23 installed 30.5 mm pitch igus links; 12.2 g per link, charged as a complete bought unit without a native chain proxy",
        stages["23_carrier_links"][1],
        MOVING,
    )
    line(
        "camera-carrier-guides",
        "Allowance for carrier guide, end brackets and parking fittings additional to the drawn cable support",
        stages["carrier_guides_supports_and_park_hardware"][1],
        MOVING,
    )
    # Moving tails, plug and the Y-case display currently share one owner in the native archive.
    wire = {
        "fixedAdapterAndBothLeads": receipts["fixedAdapterAndBothLeads"],
        "roofFactoryDataLead": receipts["roofFactoryDataLead"],
        **{
            k: stages[k][1]
            for k in (
                "cut_LAPP_bulk_power",
                "DM6010",
                "short_power_pigtail_less_than_complete_donor",
            )
        },
    }
    if wire["roofFactoryDataLead"] != stages["complete_Newnex_including_plugs"][1]:
        raise ValueError("Fixed/roof data lead duplicated or unsupported")
    lead_delta = sum(wire.values()) - float(accounts["camera-leads"]["upperKg"])
    accounts["camera-leads"].update(
        description="Fixed adapter and leads, factory Newnex moving data cable, LAPP moving power cut, DM6010 moving Y and short power donor; see camera-receiving.json for the five caps",
        upperKg=round(sum(wire.values()), 6),
        source=MOVING,
    )
    line(
        "camera-moving-finish",
        "Moving liner, solder, retention and finish allowance; separate from the fixed transition/closures line",
        stages["local_liners_solder_retainers_and_finish"][1],
        MOVING,
    )
    converters = [k for k in accounts if k.startswith("wire--converter--")]
    if len(converters) != 3:
        raise ValueError("Wrong number of independent camera/host converters")
    old_panel = sum(float(accounts[k]["upperKg"]) for k in converters)
    for k in converters:
        accounts[k].update(
            description="One MEAN WELL DDR-30G-5, 0.120 kg manufacturer packing specification, 0.130 kg receiving allocation within the fixed power panel",
            source=CONVERTER,
            upperKg=0.130,
        )
    new_converters = sum(float(accounts[k]["upperKg"]) for k in converters)
    panel = receipts["panelAndFixedWiring"]
    if panel != stages["fixed_complete_panel_with_fixed_wiring"][1]:
        raise ValueError("Changed panel account")
    line(
        "camera-panel-fittings",
        "DIN rail, positive retention, fuses, fixed panel wiring and terminations; balance within the 750 g complete panel including its three converters",
        round(panel - new_converters, 6),
        FIXED,
    )
    allocated = round(
        sum(purchased.values()) + lead_delta + new_converters - old_panel, 8
    )
    residual = round(starting - allocated, 8)
    accounts["unitemized-receiving"].update(
        description="Unassigned remainder of the original 5 kg combined installation allowance after registering the camera carrier, moving leads, panel and fittings",
        upperKg=residual,
    )
    return dict(
        historicalCombinedAllowanceKg=starting,
        reassignedToNamedArticlesKg=allocated,
        unassignedRemainderKg=residual,
        newLinesKg=purchased,
        cableGroupCapsKg=wire,
        oldConverterGroupKg=old_panel,
        newConverterGroupKg=new_converters,
        completeFixedPanelKg=panel,
        countedNativeSupportBodies=25,
        unchangedFixedClosuresKg=float(accounts["camera-closures"]["upperKg"]),
        scope="The 23-link carrier is counted at the source configuration. The original 5 kg remains the aggregate install-and-receiving allowance. The unassigned remainder is a build acceptance cap. No mass credit for the proposed 22-link successor or for the represented cable surfaces.",
    )
