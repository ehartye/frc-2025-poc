# Reinstalling the station guides

<!-- cad-still -->
[![Full swerve robot · stowed, red alliance](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Full swerve robot · stowed, red alliance.** The R3 model brings together the four steerable wheels, fixed-height receiver, electronics and red bumpers. Purchased parts use project-created representations in this public CAD. [Source STEP · 0f96394e6c](../../../cad/full-swerve-r3/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

[Compare both pipe ends in the survey viewer](review.html). In the centered example the weakest lower end has 0.220 mm of overlap after allowances. Moving both guides 0.315 mm to one side leaves the opening width unchanged, yet the weak end can have a 0.095 mm gap. These are synthetic measurements. Positive geometric overlap by itself establishes neither force nor cargo retention.

## Locate the guide before tightening it

The original root screws permit both modules to be installed outboard. This kit changes four root arms and two mast front walls. A removable 4.00 mm pilot passes through the root into a 4.02 mm round bore in the 2 mm mast wall. The lower root has a round bore and the upper one has a vertical slot with 0.30 mm of travel each way. The lower joint locates height and lateral position. The upper joint locates lateral position at the second height while accommodating hole-spacing error. The nominal axis is at Y ±219 mm, Z 692 or 792 mm. The 6 mm grip projects forward to remove the tool. [Named STEP solids](setting.step) · [Tool access and interference checks](fit.json)

The centered trial guide pin occupied 16.51 mm³ of each socket envelope. With the revised axis, all eight screw approaches clear the inserted pins by at least 1.25 mm using a 12 mm socket plus 0.25 mm radial handling allowance. Shifting the module outboard by 0.315 mm intersects every fixed pin. A 0.25 mm vertical shift interferes at the round lower joint and clears the upper slot. Confirm actual socket, mast access and wall condition before accepting the design.

1. Set up the stowed, unloaded assembly in the project chassis reference. Record the metrology fixture, instrument, calibration and actual pipe length and center range. Identify an independent bound on how well the mast frame projects to the guide faces. A small plane-fit residual alone cannot supply this bound.
2. Position and clamp each guide from the two finished liner faces. Survey the signed faces before transferring the hole locations. Drilling through a misplaced guide would reproduce that error on every installation. Account for the mast's thin front wall and the selected reaming setup in the shop drawing.
3. On reassembly, locate each guide with its lower and upper setting pins. Tighten the root screws using the qualified joint procedure. Withdraw all four pins. Record one complete survey after tightening, then remove, reinstall and survey again at the same stations.
4. Compare both signed face profiles with the pins out. The two nominal diametral clearances permit 0.020 mm of lateral root-to-mast motion at one setting and 0.040 mm between opposite settings. An illustrative 4.030 mm maximum bore with a 3.990 mm minimum pin doubles those figures. Machining error, root rotation, mast compliance and liner bonding can add more. Measure repeatability at the finished liner.

## Collect a survey

In one instrument frame, capture 14 mast and 20 liner surface points in millimetres. Supply probe-radius-compensated surface coordinates. Use the point, target normal and tip center in the [probe layout](probe.json). Each approved path fits a 3 mm ball, 2 mm by 35 mm stem and 30 mm straight withdrawal in the registered stowed model. The right front approaches at Z 845 and 820 mm intersect modeled data and power tails respectively. The layout uses Z 930 mm there. The instrument head and its support still need a physical access check.

The six front points set the X plane. The two inner side planes set the lateral midpoint and direction. Two top contacts set Z. The measured faces lie on both sides at each pair of front and rear stations at Z 628, 687, 767, 787 and 810 mm. The last two heights are on the opening lead-in. A known probe error, independent frame projection allowance, pipe center interval and length interval support a signed bound on each end. Missing frame uncertainty, excessive mast residual or a point beyond the ±0.25 mm X/Z station window leaves contact unresolved.

Run from the project root with Python and NumPy. Give each reading a distinct ID and use the same datum description for the comparison.

```sh
python guide-setting/survey.py form before.json
python guide-setting/survey.py form after.json
# Fill both forms with the actual compensated readings.
python guide-setting/survey.py check before.json before-report.json
python guide-setting/survey.py check after.json after-report.json
python guide-setting/survey.py compare before.json after.json comparison.json
```

Open either report in the viewer. To try the complete workflow with simulated readings, replace the two form commands with `example nominal before.json` and `example sideways after.json` using the same script. The difference interval includes uncertainties from both readings.

## Reproduce the native checks

Use the registered scene and installed assembly maps from [the preceding guide study](../guide-mounts/README.md). The setting export contains the six replacement bodies and four removable tools. Merge the six installed replacements by their names into the guide assembly for design analysis.

```sh
python guide-setting/setting_geometry.py <registered-scene.json>
python guide-setting/probe.py <registered-scene.json>
python guide-setting/installed.py <installed-plugin-root> <guide-map-directory> <output-directory>
python guide-setting/build_review.py
```

The four inventory comparisons retain 1,189 installed identities and give zero credit for removed chips. If the entire old hoist allocation is released, the modeled upper mass leaves 1.002 kg with matched shelf material or 0.082 kg without that credit. Keeping the old allocation leaves deficits of 0.798 kg and 1.718 kg. As-built mass, loaded guide position, acquisition, camera visibility and scoring require separate evidence. [Installed accounting](accounting.json)
