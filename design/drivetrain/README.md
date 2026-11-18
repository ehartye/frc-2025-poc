# Station-to-L1 differential drive engineering review

The first loaded station-to-reef transfer calls for four straight runs and three turns. At 62 kg the selected six-wheel candidate reproduces the signed 8.9488 m path in 17.081 s within the modeled motor, electrical, traction and chain limits. The tightest field assumption is turn traction. The replay requires a longitudinal coefficient of 0.810 against an assumed 0.90, with lateral scrub fixed at 0.38. The project's previous 96-part configuration remains the reference for the earlier pipe-custody experiment. [Registered route](../../mission/field-registration.json) · [Candidate calculations and adverse cases](route-receipt.json) · [Earlier robot](../station-l1/README.md)

Open [the editable OpenSCAD assembly](review/station-l1.scad) and use its pose and part selectors. Inspect the native solids in [stowed](review/stowed.step), [receive](review/receive.step), [transport](review/transport.step) or [place](review/place.step). All dimensions below are millimetres in robot coordinates, with +X forward and +Y left. Use the component table for exact candidate identities and primary drawings. [Components](SOURCES.md) · [Itemized BOM](review/bom.csv)

## Torque path and physical stack

Each side has one NEO 1.1, a 5:1 input cartridge, a 4:1 output cartridge, a 1/2 inch socket, one rounded-hex center axle and two 1:1 chain loops. The gear socket engages the bar 10 mm with 1.1 mm nominal bottom clearance. Bearings in both rail faces carry the radial wheel and chain loads; the socket transmits torque. A 5 mm adapter rotates the gearbox face so its two opposed 10-32 holes are at Z ±25.4 relative to the axle. Four M5 bolts through the inner rail tie the adapter to the chassis. Rail access holes and edge reliefs in the center carrier give access to the two mounting screw heads. The outer rail stays clear of the new lanes.

| Across each side from the centerline | Nominal interface |
|---|---|
| 224 / 229 / 235 | Gearbox output plane, inner rail inside face, inner bearing carrier mounting face |
| 242.59 | Outboard face of the inner flanged bearing |
| 251.5 and 268.0 | Rear and front drive chain center planes; each pin spans 9.1948 |
| 277.30–302.70 | Wheel width, centered at 290 mm for a 580 mm track |
| 317.41 | Facing flange of the outer bearing |
| 325–350 | Outer rail, with clearance openings for bearing and end retention |

The earlier 5 mm lanes would miss the pin width by 4.195 mm. These lanes leave 7.305 mm between the pin envelopes, 4.313 mm between the first pin and the inner bearing flange, and 4.703 mm from the second pin to the wheel. Tooth thickness is 3 mm versus 3.175 mm nominal chain inner width. The small 0.175 mm total tooth-face clearance makes alignment and supplier fit important. The CAD uses a tooth-tip disk and pin-width side bands. It resolves nominal packaging. It cannot measure chain tracking, tooth engagement or whip. [Parametric drive source](drive_parts.py)

The front and rear wheel centers move from ±250 to ±247.65 in X. With two equal 32T sprockets, 110 even pitches of quarter-inch chain set a 247.65 mm nominal center spacing. Four independently adjusted end axles each use paired sliding carriers, four M5 clamp holes per carrier and a captive-direction M4 jack on each rail face. The center carriers are fixed. Unload both ends of the axle, loosen both carrier clamps, move the two jacks in equal increments, measure axle parallelism and chain slack at the tightest angular position, then tighten and witness-mark the clamps. The ±4 mm slot is for assembly/take-up. The straight nominal path and bearing alignment apply at the checked position. The chosen 25 N slack-side allowance is a modeling input, not a prescribed shop tension.

## Fabrication and service review

1. Fixture and machine opposing bearing-carrier seats together. The model uses a 28.62 mm seat for the 28.58 mm nominal shell with flange capture. Check supplier tolerance and actual stock before choosing a final press, transition or retained slip fit. Lift and reseat the two drop-out plates without loading the inner race through the balls. Add tube crush sleeves or rigid threaded backing at outer-rail clamp locations; size to real bolt grip.
2. Cut the two center axles to 113.0 and four end axles to 93.6. Deburr, check reach with the assembled stock and tap accessible ends to the chosen shaft end-retention system. Fit a thin washer which lands on the inner race and a retained shaft-end screw, recessed within the rail opening. The center socket sets insertion depth through axial retention at the outer rail. End screws, washers, shims and their tool sweeps still require bench confirmation. [Shaft retention guidance](https://docs.revrobotics.com/ion-build/motion/shafts-spacers-collars)
3. Slide sprockets, the original 18 OD × 14.2 ID tube spacers, wheel/hub and outer bearing onto each axle in order. Use each spacer feature's actual axial length in the editable model; the CAD gives 0.10 mm total trim allowance per spacer. Select shims with an indicator so side clearance is positive without preloading the bearings. An inward spacer reaches the metal hub through the wheel pilot rather than clamping the polymer. Check that the spacer ID clears the rounded hex, and its OD contacts only the chosen inner ring. Verify wheel-end retention under reversal.
4. For each wheel, use the six supplied 0.157 in pilot holes on the 1.875 in bolt circle and a #10 thread-forming wheel fastener compatible with the polymer. The hub holes are 0.201 in clearance. Determine length and pullout on a sample with the flange thickness and actual thread-forming specification. Check spoke clearance. Keep the hub fastener from touching a chain at every angle.
5. Order continuous riveted 110-pitch BF25H loops; install each before closing the removable end carrier. Deburr chain guard brackets and provide a service cover over both moving chain planes. Confirm chain speed, lubrication, shock duty and specific joint working rating with Tsubaki. The guard, end washers, harness, wheel fastener heads, tool sweeps and full gearbox input protrusions require detailed fit review. Align the gearbox adapter, seat the center shaft, then tighten four rail fasteners. For the modeled 3/8 inch 10-32 mount screw, nominal thread reach after the 5 mm plate is 4.525 mm. Gauge thread depth and measure protrusion on the actual gearbox before accepting the screw.
6. Budget 747 N on the inner bearing and 520 N on the outer bearing in the conservative single-axle load screen. The maximum summed bending moment is 17.67 N·m with 19.15 N·m of state-limited torsion. An inscribed 12.7 mm nominal solid-round approximation gives 120.5 MPa von Mises. The calculation sums magnitudes from both chains, side weight and tire traction. Bearing L10 capacity, shaft alloy/temper, local notch stress, retention, rail/tube clamp capacity, collisions and fatigue remain to be qualified. [Support screen](route_check.py)

## Current, wire and speed

Run two independent protected branches from the candidate REV PDH high-current outputs, one 40 A REV ATO breaker per SPARK MAX, using 12 AWG stranded copper positive and return as the proposed drive branch. Each SPARK MAX runs a single NEO with three matched phase conductors and its six-pin encoder. Label branches DriveLeft and DriveRight; preserve polarity identification and a daisy-chained CAN bus with the two endpoint terminations. Final PDH channel numbers, CAN IDs, connector part numbers, wire lengths, loom bends, strain relief, return drop, cooling and edition-specific inspection are open. Match the 2025 inspection edition before ordering. The two 38 A phase limits are design settings for this study. A breaker’s timed thermal response and controller current limiting serve separate purposes. [Wiring and product records](SOURCES.md) · [Controller interface](../../competition-controls/io-contract.json)

For each registered trapezoid, the calculation integrates wheel motion and samples every acceleration/cruise/deceleration interval at both ends and at ≤1 ms steps. It calculates tractive force, worst-case spin scrub and yaw inertia, demands motor torque through 82% effective reduction efficiency, then solves the DC motor and shared battery equations. Rotor/wheel effective inertia, loaded mass, 50 mm CG uncertainty, 44% minimum supported side weight, 75% outer-axle share, friction and resistance are explicit inputs. The supply model uses 12.4 V open circuit, 25 mΩ common resistance, 11 A simultaneous non-drive loads and 92% controller efficiency. Measure all these under representative load. [Route and source hashes](route.json) · [Equations](route_check.py)

| Registered result | Calculated | Allowance |
|---|---:|---:|
| Peak phase current, either motor | 35.78 A | 38 A |
| Peak supply current, either branch | 20.99 A | 40 A branch |
| Minimum shared supply | 11.08 V | 9.50 V floor |
| Maximum demanded motor voltage | 7.80 V | Minimum instantaneous bus-minus-demand 3.91 V |
| Dynamic friction coefficient required | 0.810 | 0.900 assumed |
| Single-loop tight load at full 38 A phase limit, including 25 N slack | 616 N | 756 N catalog working load |
| Braking generation, both sides | 38.0 W peak, 64.6 J bound in one transfer | Battery charge acceptance unmeasured |

Independent adverse cases fail as intended. Friction 0.45 fails traction. Scrub 0.80 fails traction and current. A 10.9 V, 40 mΩ supply misses the voltage floor by 1.31 V. A 12:1 reduction demands 58.44 A peak phase current. This study covers one registered loaded transfer. Tire wear, wheel normal-force distribution, surface-dependent scrub, repeated thermal loading, contact shocks, current-loop dynamics, bus transients and actual braking behavior remain test inputs. An alternate field route requires its own replay.

## Verification and reproduction

All four 156-part named STEP exports reopen and match the intended identity inventory through the installed Agent FRC 0.15.0 checker. Removing one observed body rejects the identity. Each pose tests 12,090 possible solid pairs with finite B-Rep intersection after bounding-box rejection. All overlaps above 0.001 mm³ are empty. OpenSCAD 2021.01 also evaluates the four distinct pose selections, and its exported receiver and center axle were compared with reopened STEP. Static pose checks exclude the cable/chain dynamics and unmodeled hardware above. [CAD verification](cad-receipt.json) · [Mass, CG, service and collision records](review/review.json)

The full assembly mass allowance is 39.203–55.093 kg, plus 0.8165 kg modeled maximum payload. Loaded transport midpoint CG is (−29.25, +0.35, 253.96) mm. The total project budget range is USD 2,877–10,050, including reused mechanism allowances and reserves. Those are broad planning values; obtain supplier quotes before approving the BOM.

From the repository root, use an existing Python/OCP environment. The route calculation uses only Python's standard library. The final command takes the paths to an existing installed plugin and OpenSCAD executable.

```sh
python design/drivetrain/route_check.py artifacts/drive-route.json
python design/drivetrain/build.py artifacts/drive
python design/drivetrain/verify.py artifacts/drive-identity.json --review artifacts/drive --plugin "<installed-plugin>" --openscad "<openscad-executable>"
python -m unittest discover -s tests -p test_drive_engineering.py
```
