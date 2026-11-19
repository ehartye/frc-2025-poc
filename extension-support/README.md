# What supports the shelf during extension?

[Move the rails through the stroke](index.html). The 370 mm scoring extension changes both the rail overlap and the material above the two fixed shock absorbers. Carrying the parked support conditions into the scoring pose would give the shelf vertical support that the modeled assembly cannot provide.

## Reopen the hardware at the reached stroke

The native shop exports put the outer channel on the fixed bridge, the middle at half the slide stroke and the inner at full stroke. The two raised-floor inner channels come from the selected clearance kit. The machined shelf retains a polymer face and aluminum insert. The fixed absorber buttons are 6.4 mm across, centered at chassis X118 and Y−66/+42 mm, with free tops at Z484.9 mm. The CAD screen measures the first material above the whole projected button face. The pressure footprint can change within that face.

| Slide stroke | Nearest undeformed shelf material above the free button | Available lower contact along X at each side |
| --- | ---: | --- |
| 0 mm | 0.100 mm | Inner/middle −160 to +58.59 mm. Middle/outer −160 to +160 mm. |
| 10 mm | 9.500 mm | The button passes beneath an underside pocket. |
| 25 mm | 0.474 mm | A different portion of the machined underside reaches the button. |
| 185 mm | 9.500 mm | Inner/middle +25 to +243.59 mm. Middle/outer −67.5 to +160 mm. |
| 218 mm | 0.900 mm | The button partially overlaps the rear of the shelf. |
| 370 mm | No material overhead | Inner/middle +210 to +345 mm. Middle/outer +25 to +160 mm. |

The shelf starts at local X−100 mm. Its rear edge passes the forward edge of the fixed button after 221.2 mm of stroke. At full extension both open channels have zero native interference with the next channel when displaced vertically upward 1.05 mm. The lower floor contact was measured by driving the channel down 1.05 mm, 0.05 mm beyond its nominal lower clearance. These witnesses establish the locations used in the static screen. Contact pressure, an opposing vertical stop and joint compliance need their own evidence. [Rail and absorber geometry](geometry.json)

## Pitch loads and a captured alternative

The four earlier signed, seated pipe states supply independent fixture loads. The calculation uses gravity, the native centroids and volumes of both shelf layers, four moving channels and the carried pipe. It resolves vertical force and pitch moment over the two successive bearing intervals. At full stroke, even placing the upward floor reaction at its most favorable forward edge leaves 1.68 N·m on the inner interface and 6.55 N·m on the middle interface for the first left-hand case. A complete equipment distribution, chassis motion and alternate rear counterweight could change those reactions. [Open-channel force balance](balance.json)

The alternative machines two rounded longitudinal slots into each middle channel. A crosspin in each inner channel runs in the higher slot. Two grooved split rings capture this pin axially between the inner walls. The fixed outer channel carries two opposed screw dogs in locally thickened walls. Their noses run in the lower slot. Each stage retains its original 1 mm downward clearance. The candidate's upward clearance from the nominal unloaded geometry is 0.35 mm. At zero, half and full stroke both rail pairs remain open at 0.34 mm of relative lift and make positive contact at 0.36 mm. A 0.25 mm axial shift brings either retaining clip into its inner wall. The model reserves smooth dog shanks, heads and tapped holes. Final threads, wrench access, locking, groove-root and clip capacity, slotted web strength and wear need separate qualification. [16-part exchange](rail-fixture.step) · [Native engagement and axial retention](keepers.json)

Across the four fixture loads, the captured design needs 14.9–15.1 N downward reaction at the inner crosspin station and 54.7–54.9 N combined at the fixed dog station at full stroke. The fixture distributes each reaction across the two sides. Its two rigid stages pitch as their clearances reverse. At 1, 10 and 100 kN/mm per contact station, the front edge falls 7.11, 6.57 and 6.52 mm respectively relative to stow. A scoring trial must use the changed surface elevation and orientation. The steady fixture carries the remachined stock and new moving hardware. It omits vertical absorber force, distributed plate deformation, drive inertia and the rest of the moving equipment. [Static fixture and stiffness comparison](fixture.json)

The ten new pins, dogs and clips plus the two thickened outer walls cleared the swept inventory of 1,237 other registered CAD shapes and 93 harness edges for nominal horizontal travel. The two synchronizer brackets and the middle stop dog were reopened to resolve broad bounding-box overlaps. Actual tilted envelopes, assembly tools and parts at tolerance require their own sweep. With six installed rails replaced and ten articles added, the upper stock estimate rises by 84.046 g under independent old/new aluminum ranges, bounded steel stock and 5 g of finishing allowance. The 1,199-identity no-shelf-credit robot retains 285.146 g of modeled inspection margin. [Installed clearance](clearance.json) · [Accounting](mass.json)

## Reproduce the prebuild screens

Supply the source project's Python/Open Cascade environment. Keep generated purchasing maps and larger motion inputs with the project. Each command runs from the project root.

```sh
python extension-support/rail_geometry.py extension-support/geometry.json
python extension-support/balance.py extension-support/balance.json
python extension-support/rail_capture.py extension-support
python extension-support/installed_clearance.py <registered-scene.json> extension-support/clearance.json
python extension-support/fixture.py extension-support/fixture.json
python extension-support/rail_mass.py <installed-plugin> <allStocksAndFourteenSetsKg.json> <map-output> extension-support/mass.json
python extension-support/viewer.py
python -m unittest discover -s tests -p test_extension_support.py
```

The mass input is the straight guide’s zero-shelf-credit, conservatively retained stock case. Keep wheel calibration, cargo identity, upstream custody and the actual release observation connected when using the support model in a match replay. The stationary fixture has no inferred score.
