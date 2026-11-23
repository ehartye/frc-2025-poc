# Count the installed articles and compare mass alternatives

<!-- cad-still -->
[![Fixed pocket 1.0, preliminary design](../../../../docs/robot/fixed-pocket.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Project CAD · Fixed pocket 1.0, preliminary design · in-progress.** Whole-robot blockout on four-module swerve: fixed pocket, held two-roller nip and deploy-once exit lip. Parked pose at the end of the withdraw-park case, lip deployed and pocket empty; field parts, the delivered pipe and the two service hand gauges are hidden. It is not detailed CAD. [Open the design review](https://ehartye.github.io/frc-2025-poc/preliminary-design/)
<!-- /cad-still -->

Each alliance map represents one 545-body robot in four poses. The baseline contains **520 modeled physical instances** and **36 additional wheel-to-hub fasteners**. Of those 556 instances, 263 are designated for fabrication, 223 are purchased articles or commodity items, and 70 still have an open selection. The count covers the identified items. Unresolved loose hardware, shop stock, spare quantities and the second detachable bumper kit each require a separate purchasing decision. [Installed article crosswalk](../installed-procurement/installed.csv) · [Loose-item register](../installed-procurement/loose-items.csv)

Each pose contributes 542 installed-body assignments, one included draw wire and two service spaces. The running line, return line and spliced eye on each side share one secondary rope identity. Four primary-rope solids form two purchased lines. The SP1-25 body and draw wire form one transducer assembly. Twelve bearings require three four-packs if stock is zero. Actual stock, final order quantities and inspection weight remain unknown. [Purchasing basis](../shop-package/purchasing.csv) · [Twelve checked cases](review.json)

## Compare the options

| Selected changes to the 545-body baseline | Represented headroom at the high weight estimate |
|---|---:|
| Baseline | +0.892163 kg |
| Aluminum pocket carriers and steel inserts | +0.514542 kg |
| Steel carriers | +0.218958 kg |
| Two lower end stops | +0.384116 kg |
| Aluminum carrier design and end stops | **+0.006495 kg** |
| Steel carrier design and end stops | **−0.289089 kg** |

Each material alternative replaces the same six installed parts. The aluminum option contributes 86 candidate bodies and the steel option contributes 70. Each stop has a fabricated bracket and one catalog damper with a body and rod. The source describes a nut reserve and two sleeved bolt locations on each bracket. The map keeps those parts visible and their final selection open. Its stop mass allowance covers the native geometry before mounting bolts and sleeves. The 115 lb R103 comparison uses the same battery and bumper exclusions as the project mass calculation. Both alliance cases use the same bounded baseline. The comparison holds other changes, including the carriage-bridge study, outside this revision. Weigh the finished robot and qualify its retention and arrest components before using these figures at inspection. [Mass basis](../competition-package/shop-duplex/mass.json) · [Carrier study](../installed-procurement/CLAMPS.md) · [Stop attachment and limitations](../lift-arrest/README.md)

## Reproduce the comparison

From the repository root, set `AGENT_FRC_PLUGIN_ROOT` to the installed Agent FRC 0.20.1 directory. The checked-in red map selects the light carriers and stops. The checked-in blue map selects the steel alternative and stops. The replay tests both choices and the baseline on both colors.

```powershell
node "$env:AGENT_FRC_PLUGIN_ROOT/scripts/assembly-accounting.mjs" design/assembly-accounting/red.json
node "$env:AGENT_FRC_PLUGIN_ROOT/scripts/assembly-accounting.mjs" design/assembly-accounting/blue.json
python design/assembly-accounting/replay.py --plugin-root "$env:AGENT_FRC_PLUGIN_ROOT"
python design/assembly-accounting/build.py blue artifacts/assembly-accounting/blue-light.json --select light stop
```

The replay writes `artifacts/assembly-accounting/review.json`. Compare it with the [recorded review](review.json). Each source map pins the original crosswalk, 545 native names and frame assignments, four matching native STEP files and their reopen receipts, the carrier and end-stop studies and the baseline weight record. The generator checks named STEP products against the complete frame register and the purchased-article rollup. It derives the steel part names from the source builder’s changes to the light design and checks their population against the steel reopen receipt. The replay exercises four rejection cases covering changed native bytes, a changed mass bound, mutually exclusive carrier materials and a 300 g edit to the inspection limit. Agent FRC verifies the baseline, selected deltas and [V11 R103 comparison limit](inspection-limit.json) against pinned project records. The generator also cross-checks the original R103 margin and the exact pound-to-kilogram conversion. The design retains a null whole-configuration margin in every case.
