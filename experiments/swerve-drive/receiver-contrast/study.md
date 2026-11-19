# Seeing a white pipe on the R4 receiver

Specify the existing machined receiver facing in bulk-black polyethylene for a contrast prototype. The stationary and moving stereo cameras keep their four original pupils. At the issued positive and negative wheel errors, a genuinely equal-brightness pipe and shelf leave the robot's roof camera unable to locate the rear edge in any of 24 simulated exposures. Giving the **robot's facing alone** a dark response recovers that edge and a bounded angle in both eyes. This is a conditional prebuild observation at the station.

![Two simulated roof-image scanlines. At matched brightness the occupied and empty profiles overlap. On dark backing the occupied scanline rises where the pipe crosses it.](scanline.png)

## What changes in the robot

Machine one **Polystone G Utility black ASTM** PE facing to the existing `r6-polymer-facing` drawing. The outer limits in robot coordinates remain X −100…165, Y −210…210, Z485…500 mm. Retain the 4 mm contact skin over the underside ribs, two Ø12 bottom-up ports, attachment lands and both separate surveyed registration marks. The exposed triangulated upper face has 107,537 mm² of material before camera occlusion. The metal insert, belts, marks, guides, stock game piece and field retain their issued geometries. Bulk pigment introduces no overlay edge, adhesive, projector or electrical load. [Issued geometry and interfaces](../../six-wheel/shelf-structure/tray.py) · [Selected material](https://www.roechling.com/us/industrial/materials/thermoplastics/industrial-plastics/pe/pe-hd-pe-300/polystone-g-utility-black-astm-717982)

Reopened R4 material volume is 665,611.929 mm³. The supplier's guideline density of 950–960 kg/m³ gives **632.33–638.99 g** for the finished facing. The existing PE budget covers 930–980 kg/m³ and already charges this exact part. Keeping the original charge leaves **169.93 g** of modeled R103 headroom. Comparing the lightest previously assumed facing to the heaviest proposed one spends 19.97 g and leaves **149.96 g**. No purchased blank size, physically inspected mass, RF/optical reflectance or force coefficient is assumed. [Accounting calculation](material.py) · [Numbered robot bill](../mass-closure/r4-receiving.json)

FIRST's final 2025 English manual §8 V11 requires safe exposed surfaces and materials under R202 and R203. Rule R203 addresses interference with another robot's sensors, exposed lasers and high-intensity lighting. This proposal changes the solid color of an inward-facing robot part. It adds no lighting or fiducial and preserves the disabled piece-removal access relevant to R204. Inspect radiused edges, swarf removal, retention, wear and possible material transfer before physical use. The manual used for this review is a later edition than the kickoff rules. The design has no event inspection ruling. [Official 2025 manual](https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf)

## How the camera comparison works

The scene reader verifies the R4 STEP and field archive, reopens the full registered 975-solid public robot, isolates the named body's top triangles at Z500, then renders a separate part-identity pass from each pupil. Depth tests reject field or robot material in front. One treatment colors all visible surfaces of that body in both heads. A second treatment restricts the change to the verified upward face and leaves its one-pixel roof perimeter ambiguous. The unchanged reference and occupied view use the same declared finish. The bright pipe goes through the original per-pixel finite-cylinder depth test. Neither center, yaw nor robot port state supplies the custody decision. Known motion is compared with the fitted interval only after a possible acceptance. [Native projections](native_scene.py) · [Independent observer and intervals](../motion/receiver_witness.py)

The unmodified roof fit requires a pipe intensity of at least 185 and a difference from the surveyed empty background **greater than 36**. At pipe value 220, a facing value of 184 fails while 183 passes in this noiseless exposure model. Value 80 provides 140 counts of modeled contrast. These 8-bit inputs describe camera response. A black material designation or a CAD display color cannot establish that response. The fixed head has a different oblique view and must independently locate the piece.

| Issued station case | Timely decisions at 15 ms | What the failure shows |
| --- | ---: | --- |
| Original matched-shelf rendering | 0 / 4 | All 24 four-frame image hashes reproduce the earlier study. |
| Both camera heads see the facing at pipe brightness, 220 | 0 / 4 | The roof loses the rear edge. Some fixed fits also leave the custody box. |
| Only the top face supplies dark contrast, 80 | 4 / 4 | The existing exposed face alone supplies enough visible backing. |
| Bulk-black material response modeled at 80 | 4 / 4 | Both cameras see the changed physical article, including exposed side faces. |
| Pipe 220 against facing 184 / 183 | 0 / 4 and 4 / 4 | A single count crosses the strict threshold under these exact images. |

The four issued histories cover both attained wheel-error signs and 31.25/15.625 μs plant steps. Six actual states per history supply capture times 5372, 5389, 5406, 5423, 5440 and 5457 ms. The first five must all arrive at or before wheel service at **27.035 s**. With 45 ms delivery the dark-facing case holds in all four runs. On each history the empty scene, 18 mm lateral shift, 7° yaw, a missing fixed eye, a missing roof eye, and an opaque obstruction over the roof's sight area all hold. An exposure or lighting change that makes the occupied and empty pixels equal remains unobservable. Multiplying two equal values by the same gain cannot supply the missing edge. [Issued results](replay.json) · [5 ms wheel gate](../motion/station_clock.py)

## Corrected wheel poses and potential module occlusions

The independently recomputed reciprocal wheel-drive arrivals differ from the earlier poses. Their normal/lateral/yaw errors are (+0.434 mm, −0.063 mm, −0.00150°) and (+0.658 mm, −0.127 mm, −0.00908°). A fresh field projection and four new signed pipe histories reproduce every unmasked historical matched image for these positions. All four corrected runs meet the same 15 ms gate with the dark candidate. Equal brightness, 45 ms delivery, and the six scene or camera faults still withhold departure. [Corrected-wheel results](reciprocal-drive.json)

The static R4 rendering depicts each wheel at one angle. A separate projection of 300 all-azimuth material reservations marks 5,792 and 6,601 roof pixels uncertain, plus 121 pixels at the left fixed pupil. The observer vetoes those roof pixels in both actual and reference images, and excludes any fixed contour that touches the uncertain neighborhood. The dark candidate passes all 24 exposures and four 15 ms wheel gates after these exclusions. Dropping every fixed scanline that contains any marked pixel gives an incomplete fixed contour on every exposure and withholds all four decisions. The equal-brightness roof and late deliveries still hold. This test covers the registered start pose. [All-azimuth comparison](all-azimuth.json) · [Motion registration and source](https://github.com/ehartye/frc-2025-poc/pull/143)

## What has to transfer before a match claim

- Survey actual empty images in both heads on the assembled robot. At expected lighting extremes, compare the unchanged pipe and empty reference within the eligible, geometrically known pixels. Include glare, shadow, surface scuffing, soiling, partial occlusion and dropped frames. Record exposure, gain, registration and capture-to-wheel timestamps. A single gray reading or an isolated frame cannot authorize motion.
- Retain the existing dual-port acquisition sequence as a separate observation. Both ports can remain blocked while the independent roof edge is invisible. Check late delivery, data loss, failed registration and unexpected piece pose against recorded images with the gate enabled.
- The material page gives a 150,000 psi tensile modulus (roughly 1.034 GPa), close to the model's comparison modulus. It supplies no pipe friction, damping, optical response or incoming-stock tolerance. The nominal structural model uses 940 kg/m³. The proposed density could add 6.66–13.31 g of moving material relative to that nominal. Retain the original finite-contact trajectories as conditioned inputs until newly supported material/contact parameters reproduce acquisition and release. [Source plate model](../../six-wheel/shelf-structure/response.py)

## Repeat the simulated check

Use the project's OCP, NumPy, VTK and Pillow environment. Supply a locally authorized 2025 field STEP. Its required SHA-256 is recorded in each result. The renderer rejects a different field file and stores only derived camera arrays in the specified scratch directory.

```text
python experiments/swerve-drive/receiver-contrast/native_scene.py FIELD.step SCRATCH
python experiments/swerve-drive/receiver-contrast/finish.py SCRATCH issued-results.json
python experiments/swerve-drive/receiver-contrast/scanline.py SCRATCH scanline.png
python experiments/swerve-drive/receiver-contrast/material.py
python -m unittest discover -s tests -p test_receiver_contrast.py
```

For another registered pair of attained station poses, pass `--docks INPUT` to both the scene and finish commands and use a separate scratch directory. Each `INPUT/*us/` supplies `feed.json` and `station.json` from the motion solver. The feed file digest, pose, trajectory identity and original matched exposure hashes must agree. [Corrected wheel comparison](reciprocal-drive.json)

For the corrected comparison, reconstruct each signed unmasked station observation by passing the appropriate single row of the motion study’s `loaded-feed.json`, wrapped as `{"attempts":[row]}`, to `receiver_witness.py` with `--opening energy.json --feed-register feed.json`. Use the exact locally regenerated pipe trajectories and field STEP. Supply the matching four receipt folders as `INPUT`. Then run `wheel_case.py SCRATCH INPUT wheel-visibility.json all-azimuth.json` with the motion study’s paired mask JSON/NPZ in one directory. It verifies the mask source, tests both fixed-row policies and retains each received four-image hash. [Registered input generator](../motion/receiver_witness.py) · [All-azimuth mask generator](../motion/wheel_visibility.py)