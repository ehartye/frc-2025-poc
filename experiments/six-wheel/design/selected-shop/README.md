# Shop list for the fixed-low, two-camera robot

<!-- cad-still -->
[![Preliminary design · in progress · layout-002](../../../../docs/robot/preliminary-layout-002.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-002.** Red 2 B′x fixed pocket, held roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 67 representative solids, with the three field fixtures, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 83b3b9c7fe](../../../../docs/preliminary-design/releases/layout-002/receipt.json)
<!-- /cad-still -->

With six accepted bare wheels at **300 g or less each**, the red robot has **0.902 kg of represented inspection headroom** after adding the light inner carriers, wheel screws, fixed camera leads and four welded corners. If the wheels come in at the old 550 g allowance, the same robot is **0.598 kg overweight** at the upper estimate. Weigh the wheel lot before committing to this assembly. The finished robot has no measured weight on file. [Mass calculation](review.json) · [Selected supply list](supply.csv) · [Loose-item and receiving register](loose.csv)

## Pull list for one robot

The selected map identifies 361 purchased instances spanning 67 supply rows, 317 fabricated instances and 98 with an open selection. The supply CSV lists the installed identities behind each row. The quoted pack count assumes an empty parts bin. Every actual order field is blank pending a stock count. A batch of replacement stock stays in the pit. [Source-bound reconstruction](reconcile.py)

| Assembly | Installed demand | What the shop should pull or quote |
|---|---|---|
| Chassis wheels | Six bare white am-0940b, six am-4124 hubs and 36 screws | Order wheels and hubs individually. Candidate screw is am-1592, sold individually. Record wheel mass without hub, screws, shaft or bearing. The manufacturer recommends thread-forming screws for the plastic wheel. Check compatibility before applying threadlocker. |
| Chassis bearings | 12 REV-21-1916 bearings in the six inner/outer carrier pairs | Three four-packs from an empty bin. These bearings support the axle. No additional wheel bearings are required with the hex hub. |
| Drive reductions | Two 20:1 gearboxes | Two REV-25-2109 base kits, each containing its REV-21-2104 socket. Fit one 5:1 and one 4:1 cartridge per side. Specify either the two ratio bundles with the bases, or two cartridges of each ratio with plain bases. The former shop schedule could order two spare sockets as if installed. Do not count the complete gear envelope again when weighing the internals. |
| Wheel-bay clamps | 32 new clamp screws, 32 washers, 16 captured blocks, 4 pocketed end carriers and 2 steel adapters | Replace six old plates. The 24 outboard clamp screws and six outboard saddles were already in the frame. The two cut-to-length low-head center screw lengths have separate rows. The 8 take-up jacks and the 10 accessible axle ends have separate functions. [Dimensioned fastening schedule](../installed-procurement/CLAMPS.md) |
| Current protection | Three 40 A, one 30 A, three 20 A and three 10 A REV ATO breakers | One four-pack per rating with empty stock. Keep the PH branch and the hardware at the isolated lift branch in the inventory. Two 10 A ATM fuses for roboRIO/radio come with one PDH kit. The kit also supplies four 15 A ATM fuses that are surplus to this pull. |
| Observer power | Three DDR-30G-5 regulators, one Pi, two fixed-focus A00532 cameras, two DM6010 adapters | The older shop map orders two Pololu regulators while the active circuit uses three separate 5 V outputs. Retain the old blocks as space reservations and give no weight credit. Pull two 3 A and one 5 A MINI output fuses, three holders, one TS35 rail and retention hardware to suit the panel. Actual panel weight including the three converters must be ≤750 g for this estimate. [Power branches](../../fixed-low-power/dual-camera.md) |
| Moving loom | 23 installed igus Series 10 (width 025, radius 100, option 0) links, one Newnex data assembly, one LAPP wire run, one short power donor | Specify 23 links at 30.5 mm pitch as a 701.5 mm chain assembly. Confirm supplier order increment, mounting links and guides before ordering. A stock reel or the two parallel conductors inside the LAPP jacket do not count as two installed cables. Gauge Newnex plug-to-plug length before accepting the unit. [Lengths and service](../../camera-disconnects/README.md) |
| Sensor acquisition | Two ADS1015 acquisition boards | The original three pads occupy the first board (0x48). The four AS5600 channels require the second (0x49). Select the breakout, protection, wire, enclosure and connectors. Preserve the grouped observation and harness allowances. |
| Air, bumpers and pit items | No match compressor, tank, both saddles, regulator/dump or valve. One PH remains. One bumper kit on the robot. | Prepare a second complete removable color kit for the pit, with eight permanent sleeves on one chassis. Keep both service pins in the pit and weigh onboard pin holders where present. [All 19 loose-item families](loose.csv) |

## Wheel receiving and installation

1. Match the carton and the wheel to am-0940b. The selected 2018 drawing identifies a white 77A tread. The current sales variant uses the same SKU and labels the white wheel 80A. Record the actual lot and require the design owner to accept the tread/traction change if supplied. The family description lists 0.5 lb for the larger wheel, and the exact commerce variant lists 272 g. Neither is a manufacturing tolerance. Our 200–300 g receiving interval is an explicit scale gate. [Six-inch drawing](https://documents.andymark.com/43d8eabd0b55929ee8eadc97fe948f84c4aba155aef6d292cfc0dbd9018459f1/am-0940b%206%20HiGrip.PDF) · [Wheel listing](https://andymark.com/products/higrip-wheels?variant=44493388415148) · [Variant record](https://andymark.com/products/higrip-wheels.js)
2. Measure six holes on the 1.875 in bolt circle in each wheel and hub. The known wheel bore is 0.157 in and the hub clearance is 0.201 in. Fit the long hub shoulder inward. The selected am-1592 is a #10-12 × 1.25 in self-piercing steel screw with external ¼ in hex drive. Nominal listing weight is 0.008 lb and the order record rounds to 4 g. Set aside parts outside 3–5 g each or beyond the 11 mm diameter × 4 mm high head gauge. Check the flange and installed projection with the actual recess. [Screw listing](https://andymark.com/products/10-12-x-1-25-in-self-piercing-hex-head-cap-screw?variant=44497346592940) · [Screw variant](https://andymark.com/products/10-12-x-1-25-in-self-piercing-hex-head-cap-screw.js)
3. Through a full turn the gauged heads clear the other modeled solids by at least **1.525 mm**. A 6 mm head at 221.82° overlaps an outer-cap screw by **26.97 mm³**. The nominal under-head land is at |Y|307.475 mm. A 31.8 mm tip projects 1.625 mm past the inner wheel face. Shank clearance to other modeled parts is at least 4.446 mm. The native check reopened the issued 603-body place STEP, substituted the 86-piece clamp kit, and used an annulus to bound every rotation angle at nominal axle centers. The registered camera/service additions and corner weld material lie outside the wheel bays. The wheel and hub are intentional screw mates and excluded from the obstacle set. Actual pilot engagement, stripping torque, full take-up, debris and deflection still require a fixture pull and rotation check. [Original native calculation](native.py) · [Clearances and failing controls](wheel-fit.json)
4. Check the plates, shims, jack locking and chain guards at both adjustment limits. Turn each assembled wheel by hand on the unpowered shop fixture and gauge the loaded interfaces according to the team's approved procedure. Record actual wheel axial movement and clearance before releasing a production order. The 10 accessible axle ends require a separate retention design and mass allowance.

## Weigh sheet

The six old wheel rows each carried 350–550 g. Replace those exact entries at each endpoint before summing other independent changes. At the high end this removes 3.300 kg, adds up to 1.800 kg of bare wheels, conservatively adds up to 0.180 kg for the 36 screws, adds 0.105 kg for the fixed Y-adapter and short local plug leads and selects the 0.377621 kg light carrier increment. The #82 starting estimate already books the entire four-weld group at up to 0.020 kg. Incoming assumptions do not establish any manufacturer's worst-case weight. [Condition inputs](articles.json) · [Exact mass sources and upper comparisons](review.json)

| Receipt at represented upper weight | R103 headroom |
|---|---:|
| Selected wheel lot, screws, adapters, 32 clamps and four welds | +0.902457 kg |
| Original 550 g wheels, same loose hardware | −0.597543 kg |
| Steel inner carriers with accepted wheels | +0.606872 kg |
| Selected robot plus an additional 1 kg of unbooked materials | −0.097543 kg |

Keep the original 2–5 kg unitemized allowance, 0.65–1.65 kg wiring allowance and underlying observer allowance. A duplicate allowance cannot be removed without a weighed or itemized replacement. Enter actual tare, combined masses and scale resolution for the six bare wheels (≤1.800 kg), 36 screws (≤0.180 kg), fixed adapter/short leads (≤0.105 kg), complete converter panel (≤0.750 kg), both carrier take-up assemblies and mounts, and complete dressed chassis. Keep any kit spares, the battery and the detached bumpers separately labeled. Repeat the final weigh with the selected season inspection exclusions and again with each complete bumper set. At the estimated 7.5 kg bumper upper end, the 51.260666 kg R103 robot becomes 58.760666 kg under R408, leaving 2.4743 kg against 135 lb. Finishing, guards, axle retainers, wiring, adjustments and measurement error can consume the available margin.

## Reproduce

Use the generated #80 selected map and the installed Agent FRC 0.22.0 root. The #82 generator reopens every source pin in the current checkout. The second command emits the selected map, purchase CSV, mass source and result in the output directory. Its input map must be a #82 configuration with all four corner joints selected once. Regeneration of the native check also needs the generated, hash-matched 603-body red place STEP from the [camera datum shop package](../../camera-datum-shop/README.md).

```shell
python frame-corners/account.py "$AGENT_FRC_PLUGIN_ROOT" "$CAMERA_SELECTED_MAP" artifacts/selected-shop-input
python design/selected-shop/reconcile.py "$AGENT_FRC_PLUGIN_ROOT" artifacts/selected-shop-input/joined-frame.json artifacts/selected-shop
node "$AGENT_FRC_PLUGIN_ROOT/scripts/assembly-accounting.mjs" artifacts/selected-shop/selected.json
python design/selected-shop/native.py "$SHOP_PLACE_STEP"
python -m unittest discover -s tests -p test_selected_shop.py
```

The supplier's four-pack part numbers, base-kit content, and PDH kit contents are the purchasing references. Check current stock, lot dimensions and manufacturer revision before procurement. [Breaker pack numbers](https://docs.revrobotics.com/ion-control/pdh/specs/rev-ato-auto-resetting-breakers) · [MAXPlanetary kit contents](https://www.revrobotics.com/maxplanetary-system-kit/) · [PDH kit and included fuses](https://www.revrobotics.com/rev-11-1850/) · [PH mass](https://www.revrobotics.com/rev-11-1852/)
