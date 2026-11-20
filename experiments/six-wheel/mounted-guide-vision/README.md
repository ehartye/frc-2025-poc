# What the mounted guides hide

<!-- cad-still -->
[![Team 1780 · captive swerve receiver](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Team 1780 · captive swerve receiver.** The public Fusion capture candidate at rest, with 1,001 robot solids and the hollow practice CORAL. Project-drawn representations occupy the supplier interfaces. [Source STEP · 0dad9e862e](../../../cad/swerve-captive/public-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

[Open the student camera replay](viewer.html). Scrub actual sector time and compare both pupil windows for each fitted installation.

The lower left guide arm hides one of the two surveyed fasteners at scoring sector A. With the nominal and outward-fitted guide modules, both signed approach controllers stop before the scoring target. The closed-loop stops leave the smaller-left-wheel robot 74.97 mm short and the smaller-right-wheel robot 27.48 mm short. Both keep the scoring rack stowed. [Field trials](sector.json) · [Native part attribution](obstruction.json)

The fixed stereo camera retains the five station position observations on both original folded-guide arrival models. The rigid-carriage source and the independently moving guide-bearing source have separate state records. The roof camera uses two observed pipe ends, surveyed robot reference patches, a rear tangent and a finite position interval. Its stricter 0.05 mm backing challenge removes parts of the outer contour. The student should distinguish a field pose observation from a witnessed stored pipe. [Signed station trials](station.json)

The numerical trial reopens all 58 native guide bodies. The 56 added articles form the rendered mask and the two earlier opaque mast shapes conservatively remain in place. The outward installation uses the manufactured datum control, with modules shifted 0.315 mm, sleeve axes 0.040 mm and root fasteners 0.120 mm on each side. The other robot and moving-cable raster comes from the machined 998-body receiver and 629 fixed harness bounds. Calibrated synthetic crops do not establish actual field illumination, surveyed optical pupils or human-feed dispersion. [Installed guide](../guide-mounts/README.md) · [Machined source](../observed-reef-a/README.md)


## What blocks the field view?

The kit reopens on its native red robot frame. NAUO1187 remains visible in every formerly clear window. The smaller-left-wheel robot loses 52 formerly clear NAUO1192 windows in the left pupil. The lower arm at Z687 crosses 50 and the guide backing crosses two. The smaller-right-wheel robot loses 46 such windows. That arm covers all 46 and the backing also covers three. On both paths the right pupil still has its original usable NAUO1192 windows. At the earlier controller's close poses, the obstructions include points on the lower-left arm between X86.3 and 207.7 mm. The independent deformation/fit and structural load studies govern changes to that material. [Window attribution and robot coordinates](obstruction.json)

Removing only the lower left arm from the images permits both signed controllers to reach the sector. Removing only the backing leaves both stopped. Removing both articles allows the right case and inhibits the left. These are deliberately incomplete optical controls. Only the complete kit can support a mechanical handoff. The setting-hole revision, with four pit tools removed, reproduces all 24 installed-kit wheel trajectories. [Removal controls](ablation-sector.json) · [Service setting revision](setting-sector.json)

## Can the camera still confirm the pipe?

Every nominal fixed-eye and roof exposure originates from that signed first pipe's recorded arrival. The original guarded roof observer requires consecutive visible pixels throughout an end contour. A new option allows intervals of unknown backing between actual bright pixels. It still requires known outside pixels at the outer end edge, observed curvature bounding both sides of the end extremum, a measured rear tangent, two-sided yaw evidence and four separated robot-reference patches. A single known blank within a long internal gap still rejects that row. The option leaves the released observer's default behavior intact. [Station data](station.json) · [Opaque and displaced controls](controls.json)

At 31.25 µs, all six guarded exposures per side qualify on the nominal rigid-carriage guide. The separately registered 15.625 µs left arrival differs by about 2.46 mm in final X and independently qualifies at all six samples. Under the outward installation, the right source's potential lateral center extends to −14.46 mm and its bottom extends to +25.33 mm. Neither interval meets the custody gate. The separate nominal two-stage carriage plus recovered Java program qualifies on the left at 31.25 and 15.625 µs. The right stays held at both resolutions. The 3,500 Hz rigid-carriage controls qualify on both sides. Forty out-of-band, missing, or damaged-image controls yield zero positive decisions, as do matched-brightness images. The earlier 850 mm guide program remains a distinct source. [Signed final trajectories](../guide-impact/station-frames.json) · [Two-stage mechanical inputs](../compliant-carriage/README.md)

With 15 ms packet delivery and a 5 ms service tick, the five station captures through Java 5,440 ms arrive by 5,455 ms. The next exposure at 5,457 ms first becomes available at 5,475 ms. The proposed station-only gate uses the five frames available at the 5,460 ms departure service, requiring a fixed position witness paired with an independently bounded roof position and a yaw interval within ±2°. Source truth is reserved for checking the resulting intervals. [Timestamped station gate](handoff.json)

Both complete-guide field trials hold the scoring rack stowed. The pipe placement and second feed remain unqualified. A field permit depends on camera survey, fixture contrast and an as-built marker that stays in view. A stationary custody result does not advance the pipe into another assembly's run.

## Reproduce

Use the upstream registered field, guide-impact arrival packets, 998-body machined scene and fixed harness inputs. Keep the render environment's original optical calibration.

```sh
python mounted-guide-vision/station.py "$ARTIFACTS" "$FIELD_STEP"
python mounted-guide-vision/station.py "$ARTIFACTS" "$FIELD_STEP" --setting guide-setting
python mounted-guide-vision/sector.py "$ARTIFACTS"
python mounted-guide-vision/sector.py "$ARTIFACTS" --setting
python mounted-guide-vision/obstruction.py "$ARTIFACTS"
python mounted-guide-vision/ablation.py "$ARTIFACTS"
python mounted-guide-vision/sector.py "$ARTIFACTS" --ablation
python mounted-guide-vision/controls.py "$ARTIFACTS"
python mounted-guide-vision/handoff.py
python mounted-guide-vision/student.py "$ARTIFACTS"
```
