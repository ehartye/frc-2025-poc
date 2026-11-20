# Student Fusion design

## Inspect the fixed-low engineering alternate

[Download the front-jack engineering master](assembly-engineering/issue/Team-1780-engineering-shallow.f3d) to inspect the module saddle lands, equipment mounts, battery tie-downs and separate jack transmissions. Use the [parked assembly viewer](https://ehartye.github.io/frc-2025-poc/engineering-assembly.html), [unit-by-unit bill](assembly-engineering/issue/material-register.csv) and [shop assembly sequence](assembly-engineering/issue/drawing-traveler.md) together. The independent [cold Fusion measurements](assembly-engineering/issue/cold-reopen.json) record geometric tolerances and the motion graph for this issued assembly.

## Download the editable student robot

[Open the public capture assembly](swerve-captive/Team-1780-public-captive-swerve.f3d) to drive all four steering and roll pairs, the two coupled receiver stages and the opposing capture yokes. The file includes the numbered bumpers and a separate practice CORAL. [Joint guide](swerve-captive/study.md) · [Exact archive reopen](swerve-captive/student-reopen.json)

## Try the front-jack climb variant

The separate full assembly adds a forward shallow-cage jack to the stowed receiver and four-module chassis. Drive the crosshead, both shoes and lower clamp to inspect the bearing contact. Added mass and field-plate load still exceed the evidence for a match-ready climb. [Download the climbing variant](shallow-cage/Team-1780-shallow-cage-study.f3d) · [Rotate and operate the climb](https://ehartye.github.io/frc-2025-poc/shallow-climb.html) · [Native measurements and loads](shallow-cage/study.md)

## Supplier-detail capture candidate

The season artifact `artifacts/cam-scoring/FRC-2025-Team-1780-captive-supplier-swerve.f3d` includes the four supplier module assemblies, 26 capture parts, cam shelf and two sprung-probe sliders. Fusion checked 31 actual occurrences at six poses before export. The independent reopen for this 143,014,397-byte supplier-detail file is pending. The public student file above has its own exact-byte reopen and uses published supplier representations. [Private archive hash and export measurements](swerve-captive/supplier-export.json)

## Drive the earlier supplier-detail receiver in Fusion

Open `artifacts/swerve-articulation/FRC-2025-Team-1780-articulated-swerve.f3d` from the season project artifacts. In the colored project hardware component, right-click **Receiver / slide**, choose **Drive Joint**, and move from 0 to 370 mm. The second stage moves 185 mm. Watch the shelf camera, end grippers and machined receiver travel together while the numbered bumper and four MAXSwerve assemblies stay at their original locations. Return the slide to zero to inspect the stowed robot. [Native measurements and reconstruction](swerve-articulation/study.md) · [Watch the pickup and release study](../experiments/swerve-drive/score-stroke/receiver-motion.webm)

## Open the full Team 1780 robot

Open `artifacts/numbered-swerve-fusion/FRC-2025-Team-1780-full-swerve-R4.f3d` from the season project artifacts. Select the stowed Team 1780 assembly to inspect the receiver, straight guides, chassis, repaired rear electronics and four REV MAXSwerve modules. The numbered red bumper covers carry the team's 2024 artwork on four outward faces. The archive is 134,437,517 bytes with SHA-256 `1eb7a281eba9173d91e2ff47c3175a03c4f05c7d2374c1c4c494ad2b80f5064a`. The companion Fusion image is `artifacts/numbered-swerve-fusion/full-swerve-R4.png`. [Reopen and appearance measurements](full-swerve-r4/native-reopen.json)

Fusion independently reopened 2,071 bodies in six assembly groups. The two camera housings and the signal light account for 27 supplier bodies. Across 948 visible project solids, each stable part name, color and placed bounding box matched the design register. The greatest coordinate difference was under 0.000014 mm. Both red covers and all 16 white digits persisted. All 1,096 bodies in the four purchased module assemblies retained their source material and appearance, and each wheel center retained its measured R2 position. Archive access requires the project-local supplier data. Students using the public STEP can inspect the original chassis, receiver and bumper lettering alongside 27 project-drawn supplier representations. [Rotate the numbered robot](../docs/robot/cad/index.html) / [Public STEP](full-swerve-r4/full-robot.step) / [Bumper and supplier scope](full-swerve-r4/design.md)

## Earlier chassis studies

Open `artifacts/swerve-package/FRC-2025-MAXSwerve-student-chassis-R2.f3d` from the project checkout for the revised four-module chassis. This 182,274,836-byte local archive has SHA-256 `f4b14c738dd13e510297e5f34ee64f451d27da16fbd2bddf4e19cc6fdf8fa915`. Fusion independently reopened six root occurrences with the elevated host plate, one drilled deck and all four wheel centers still within 0.0032 mm of their registered positions. The original project-owned rear plates and bracket are available separately as STEP. [Reopened R2 assembly](../experiments/swerve-drive/packaging/student-archive-r2.json) · [Rear hardware](../experiments/swerve-drive/packaging/rear-electronics.step)

For the first fit, open `artifacts/swerve-package/FRC-2025-MAXSwerve-student-chassis-r1.f3d`. The independently reopened archive is 183,364,240 bytes with SHA-256 `7c59a430a87d731c22fed4e266be2bc97b5d8b23ee149cf867081e9bf616ad92`. Its companion full-assembly still is `artifacts/swerve-package/student-chassis.png`, SHA-256 `62334de8c821bfbd2a4e34ce9444c87ed2665ace732f9652d1b7511c5feeefa0`. Both are project-local because the supplier solids have separate distribution terms. [Reopen record](../experiments/swerve-drive/packaging/student-archive.json)

The archive contains four actual REV assembly occurrences and 79 project frame and mounting bodies. Each corner reopened with 74 supplier occurrences and 274 Fusion bodies. The wheel centers agree with the registered kingpins within 0.0032 mm. The named parameters are `Wheelbase` (564 mm), `Track` (554 mm) and `NominalWheel` (76.2 mm). Read the packaging record before changing a dimension. Work through the positional model and a fresh interference check after an edit. [Design sources and reconstruction](../experiments/swerve-drive/packaging/) · [Native coordinate contract](../experiments/swerve-drive/packaging/module.json)

The selected product is MAXSwerve V1.1 with a V2 wheel, a 12-tooth pinion, NEO drive and NEO 550 steer. The integral supplier assembly is the 2022 export. It supplies the principal module shape, axes and placement. Later revisions and hardware substitutions have separate source and envelope checks. The R2 archive covers the drivetrain chassis; the new controller trays, cable reservations, retained superstructure and scoring receiver have their own engineering records.

For a directly downloadable native exercise, open [tab-saddle.f3d](../experiments/swerve-drive/packaging/tab-saddle.f3d) in Fusion. This project-designed saddle has four native sketches and extrusions, with six editable dimensions. Its volume agrees with its independently reopened structural STEP body to within 0.006 mm³. [Saddle evidence](../experiments/swerve-drive/packaging/saddle-fusion.json)

The scoring-stroke study also includes a [downloadable native cam-land assembly](../experiments/swerve-drive/score-stroke/cam-lands.f3d). Open either of its two components to see how the underside lands and relief are machined. The candidate has six editable Fusion extrudes and one shared lane-width parameter. [Compare its dimensions, cap clearance and station trials](../experiments/swerve-drive/score-stroke/study.md)

[Rotate the current numbered robot](../docs/robot/cad/index.html) · [Open the structural STEP](../experiments/swerve-drive/packaging/swerve-frame.step) · [See the previous six-wheel campaign](../experiments/six-wheel/README.md)

The editable [shop master](assembly-engineering/issue/Team-1780-engineering-shallow.f3d) is SHA-256 `29c777fc5e875e5f2131ee2e34c8d3eaaf46e5ff5f8edffdf3399a17c6182254`. Its [cold reopening](assembly-engineering/issue/cold-reopen.json) covered 1,184 bodies and all 21 moving joints. The [revised battery bay](assembly-engineering/battery-fit/traveler.md) has a separate STEP and load-path review.
