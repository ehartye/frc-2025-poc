# Which material and light changes can the receiver tolerate?

**Keep the bulk-black PE facing as a stationary acquisition candidate. Leave the selected R4 drawing and Fusion assembly at their issued revision.** The complete material study seated the pipe in 12 of 12 simulations. A revised four-eye check confirmed those 12 pickups under the admitted lighting. Healthy and failed belts nevertheless produced identical pictures before the robot moved. The failed belts let the pipe travel nearly 49 mm during the first drive and brake. The robot needs a defensible transport restraint or timely independent slip trip before this finish can receive match-wide custody credit. [Contact and optical results](results.json) · [Matched healthy/faulted drive](retention.json)

The optical prototype uses one bulk-black PE facing and four existing camera pupils. A material change also affects the pipe that reaches the cameras. Replay the actual contact plant before transferring the R4 custody decision. The project uses right-handed robot coordinates with X forward, Y left and Z up. [Original contrast study](../receiver-contrast/study.md)

## Contact experiment

Röchling lists 950–960 kg/m³ and a tensile modulus of 150,000 psi for Polystone G Utility black ASTM. The issued model uses 940 kg/m³ and 1.034214 GPa. This experiment substitutes the measured-property guideline endpoints and the exact conversion, 1.034213594 GPa, in the plate mass and stiffness matrices of the two-stage carriage. Each fresh trial constructs its frozen shelf contact law, keeps the original guide/belt coefficients, and exchanges actual port states and belt commands with the original desktop Java controller. The preload, friction-transition velocity, normal compliance, damping, guide and plunger geometries keep their issued values. [Material page](https://www.roechling.com/us/industrial/materials/thermoplastics/industrial-plastics/pe/pe-hd-pe-300/polystone-g-utility-black-astm-717982) · [Contact adapter](contact_material.py) · [Carriage equations](../../six-wheel/compliant-carriage/model.py)

The original pipe/shelf friction coefficient was a model input of 0.10. The design experiment pairs each of 950 and 960 kg/m³ with both 0.05 and 0.20, and checks 955 kg/m³ at 0.10. It uses each signed reciprocal-wheel arrival at a 31.25 μs contact timestep. The central material point also runs at 15.625 μs. These are individual evaluations of a coupled nonlinear system. Interpolation between them needs its own evidence.

Two published experiments illustrate why a coefficient quoted for PVC or HDPE alone would be misleading. In the 2022 pipe study, both materials slide against a steel disc; the other study tested PP-on-PET, PP-on-HDPE and PP-on-PP. Neither experiment identifies the pairing between this machined facing and game pipe. Surface texture, assembly load and sliding velocity can vary. [Steel counterface study](https://www.tandfonline.com/doi/full/10.1080/2374068X.2022.2036584) · [Other polymer pairs](https://www.sciencedirect.com/science/article/pii/S0301679X15003692)

## Robot-owned exposure references

The original 8-bit contour detector needs a bright pipe and sufficient local contrast. Multiplying the full image by 0.72 and adding four counts can move its apparent brightness below the raw detector's threshold even when the pipe remains distinct from the shelf. The candidate estimates exposure independently at each eye before running the unchanged contours.

1. Reopen the entire R4, obtain the optical ray in the actual pupil frame and identify the nearest opaque native robot surface against the registered real field. Exclude uncertain wheel pixels. Also exclude every ray that could pass through a prospective pipe volume X −35…212, Y −205…205, Z 475…665 mm before hitting the background. This leaves 100,717 and 104,256 fixed-head pixels and 24,898 and 24,750 roof pixels for the two signed dock poses.
2. Sample those regions by a four-pixel grid. Split 32-pixel tiles between estimation and independent checking. A reference level must have at least 12 samples in each partition. Fit an affine transfer from the medians of at least three surveyed brightness levels spanning at least 35 raw counts. Reject gains outside 0.55…1.35, offsets outside −20…25, or a held-out group's 90th-percentile error over three counts.
3. Correct the received image, preserve camera identity and remove all uncertain wheel pixels. Then run the fixed position witness, roof rear-edge and yaw fit, followed by the five-exposure wheel gate. Simulator state is available to evaluate the final fitted intervals and to check the test occupancy box. It never estimates the light correction.

The global gain cases 0.72 with offset +4 and 1.12 with +6 and the independent per-eye gains are declared camera scenarios. They are not measurements of arena illuminance or guarantees provided by a product color name. Saturation, local pipe shadow, washed-out backing, an occluder, lost registration, independent missing-eye and late-packet cases must withhold any unsupported motion. [Native camera-reference masks](calibrators.py) · [Estimator](radiometry.py) · [Four-eye evaluator](observations.py)

## Future inspection

To characterize a particular lot, measure density and the cut part, compare a restrained non-rolling pipe sliding on the actual face across loads and speeds, and record the slip transition and wear. A freely rolling tube on an incline does not measure the sliding coefficient used here. With all four pupil settings recorded, inspect empty and occupied images under arena-like lighting, exposure transitions, scuffing and shadows. Keep as-built mass, friction, contrast and inspection acceptance separate from the prebuild results.

## What the cameras could see at pickup

All 12 plant runs reached the desktop controller's seated state in 1.340–1.380 s and preserved it through 5.460 s. At the first camera capture the pipe centers were 74.06–77.03 mm forward of the shelf datum. The original roof test required a visibly bracketed end peak. Only six of the 12 attained seats met it. The replacement combines the existing fixed-stereo position interval and independent roof rear-edge yaw bound with an image-space fit of both known circular ends. Each eye needs at least 45 depth-backed samples over 65 rows, with 88% of eligible rows agreeing within 1.5 pixels. The contour also needs a measured departure from a straight edge. Simulator coordinates check the independent position/yaw bounds after a grant. The fitted conic supplies corroboration of the end shape. Its five fitted parameters do not establish independent uncertainty bounds. [Projected ends](model_arc.py)

| Input to the wheel-service gate | Historical peak | Paired curved ends |
|---|---:|---:|
| Nominal, dim with correction, bright, or independently varied pupil gains | 6/12 in each case | 12/12 in each case |
| Raw dim image without correction | 0/12 | 0/12 |
| Genuinely matched pipe and facing | 0/12 | 0/12 |
| 45 ms delivery | 0/12 | 0/12 |

The low-friction positive arrival also held on an empty image, a locally shadowed pipe, a plugged sight line, an 18 mm sideways error, a 7° yaw error, saturation, a missing pupil, discordant brightness references, and a flat surrogate in one or both roof pupils. A white uniform cylinder can spin about its long axis without changing its silhouette. Even a perfectly aimed camera observing its stationary outline cannot measure the friction reserve at the end belts. These are modeled 8-bit camera responses. The supplier specification establishes polymer properties; it does not guarantee a pixel value.

## What actually retains the pipe

At the wheel-service boundary, the low-friction positive arrival gives two histories the same full pipe, carriage and guide state, wheel trajectory and preceding images. In one, the end belts keep their issued tangential contact. In the other, the belt coefficient becomes zero. With the same 600 ms drive lease and 900 ms braking tail, the healthy pipe moves at most 0.254 mm. The faulted pipe moves 48.698 mm and ends at X122.73, Y−16.58 mm. Both simulations still have the pipe on the shelf. Their centers first differ by more than 1, 4 and 8 mm at 131, 213 and 266 ms. The peak translational acceleration of 1.147 m/s² requires 0.937 N for the 0.816 kg pipe. Even placing its full weight on the low-friction shelf gives only 0.400 N in the ideal sliding-force comparison. The contact solver supplies the actual motion, including rotation. Both continuations are diagnostic inputs, irrespective of the camera's later hold. [Loaded replay](loaded_material.py) · [Comparison and registered signals](premise.py)

| Candidate or installed evidence | What changes the next decision | Remaining ambiguity |
|---|---|---|
| Bulk-black facing and paired end arcs | Confirms position at the surveyed station and rejects an empty or wrongly oriented object. | A local shadow or truly matched return can hide the edge. A good image at rest cannot reveal impending traction loss. |
| Installed Entry and Seat ports plus passive idlers | A missing expected return or stationary idler can veto the acquisition probe. Historic locked belts tripped at 640 ms. | The historic empty replay still produced IN_BAND at 1,340 ms with copied port and idler signals. A fresh, physically calibrated in-pocket slip trip and its entire causal braking tail remain open. [Idler controls](../../six-wheel/grip-motion/README.md) |
| Three installed end pads | Contact indicates approach to the front release edge. | The closest active pad remains at least 16.45 mm beyond the pipe's forward projection across the acquired fixtures. Empty shelf and properly seated pipe can both produce zero load. [Source pad geometry](../../six-wheel/release-observer/build_mount.py) |
| Positively captured pocket with independent closure, occupancy and release checks | A certified physical restraint could carry an identified piece through a bounded disturbance during a camera outage. A fresh exit check would precede release/retraction. | Closing a gate on an empty pocket also trips a closure switch. The earlier outer-diameter hood and split cassette reached only 52/64 of their own modeled combinations and have different receiving geometry, mass and release motion. They are a comparison design. [Mechanical alternate](../../six-wheel/od-gate/README.md) |

Continuous imaging is a rule of the issued transport controller. The physical requirement is to keep the pipe in its permitted envelope throughout motion and braking. A demonstrated capture geometry and load capacity could replace that camera lease. A camera can still serve acquisition, re-acquisition and release. This study issues no moving authorization, score or material substitution on the full robot.

The first healthy loaded replay also examined 88 subsequent four-eye exposures under dim light. Both the deliberately robot-only comparison and the conservative moving survey held at wall 27.065 s. The original fixed-eye position estimator withheld all robot-only heights. The field-surveyed comparison restored the fixed estimate, but the implementation compares a guard-shifted roof depth (up to 0.05 mm) to the native surface at a tighter 0.002 mm tolerance. This can exclude actual black material. These moving frames establish fail-closed behavior for that restricted implementation. They leave the real moving-view performance unresolved. [Moving image conditions](loaded_images.py)

## Reproduce

Use a project-local raw output directory, the desktop JDK from the original station replay, the two reciprocal drive CSVs from the [motion study](../motion/energy.json), and the locally generated field-backed and wheel-mask artifacts from the [facing experiment](../receiver-contrast/study.md). Run the following from this study directory. Substitute your paths for the upper-case names.

```text
python contact_material.py JDK CLOCKS RAW --condition 950 .05 31.25 --condition 950 .20 31.25 --condition 960 .05 31.25 --condition 960 .20 31.25 --condition 955 .10 31.25 --condition 955 .10 15.625
python calibrators.py DOCK_GEOMETRY WHEEL_MASK_JSON RAW
python evidence.py RAW DOCK_GEOMETRY WHEEL_MASK_JSON
python evidence.py RAW DOCK_GEOMETRY WHEEL_MASK_JSON --rims
python evidence.py RAW DOCK_GEOMETRY WHEEL_MASK_JSON --rims --full --match rho-950-mu-0.05-plus-31.25us
python loaded_material.py RAW/rho-950-mu-0.05-plus-31.25us CLOCKS
python loaded_material.py RAW/rho-950-mu-0.05-plus-31.25us CLOCKS --lose-belt-traction
python loaded_images.py RAW/rho-950-mu-0.05-plus-31.25us ../transit NATIVE_SCENE FIELD_STEP DOCK_GEOMETRY WHEEL_MASK_JSON --robot-only
python loaded_images.py RAW/rho-950-mu-0.05-plus-31.25us ../transit NATIVE_SCENE FIELD_STEP DOCK_GEOMETRY WHEEL_MASK_JSON
python register.py RAW results.json
python premise.py RAW retention.json
```

For each additional healthy continuation recorded in `results.json`, run `loaded_material.py` once with its corresponding raw case directory before registering. Estimated weight stays inside the earlier 930–980 kg/m³ stock interval; the conservative independent low-to-high comparison retains about 150 g of modeled headroom. A physical weight, friction measurement and inspected assembly remain unrecorded. [Original single-part mass accounting](../receiver-contrast/study.md)
