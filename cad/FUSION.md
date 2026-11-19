# Student Fusion design

Open `artifacts/swerve-package/FRC-2025-MAXSwerve-student-chassis-r1.f3d` from the project checkout to inspect the vendor-bearing chassis in Fusion. The independently reopened archive is 183,364,240 bytes with SHA-256 `7c59a430a87d731c22fed4e266be2bc97b5d8b23ee149cf867081e9bf616ad92`. Its companion full-assembly still is `artifacts/swerve-package/student-chassis.png`, SHA-256 `62334de8c821bfbd2a4e34ce9444c87ed2665ace732f9652d1b7511c5feeefa0`. Both are project-local because the supplier solids have separate distribution terms. [Reopen record](../experiments/swerve-drive/packaging/student-archive.json)

The archive contains four actual REV assembly occurrences and 79 project frame and mounting bodies. Each corner reopened with 74 supplier occurrences and 274 Fusion bodies. The wheel centers agree with the registered kingpins within 0.0032 mm. The named parameters are `Wheelbase` (564 mm), `Track` (554 mm) and `NominalWheel` (76.2 mm). Read the packaging record before changing a dimension. Work through the positional model and a fresh interference check after an edit. [Design sources and reconstruction](../experiments/swerve-drive/packaging/) · [Native coordinate contract](../experiments/swerve-drive/packaging/module.json)

The selected product is MAXSwerve V1.1 with a V2 wheel, a 12-tooth pinion, NEO drive and NEO 550 steer. The integral supplier assembly is the 2022 export. It supplies the principal module shape, axes and placement. Later revisions and hardware substitutions have separate source and envelope checks. The archive covers the drivetrain chassis; the new controller trays, cable reservations, retained superstructure and scoring receiver have their own engineering records.

For a directly downloadable native exercise, open [tab-saddle.f3d](../experiments/swerve-drive/packaging/tab-saddle.f3d) in Fusion. This project-designed saddle has four native sketches and extrusions, with six editable dimensions. Its volume agrees with its independently reopened structural STEP body to within 0.006 mm³. [Saddle evidence](../experiments/swerve-drive/packaging/saddle-fusion.json)

[Rotate the current project-owned swerve frame](../docs/robot/cad/index.html) · [Open the structural STEP](../experiments/swerve-drive/packaging/swerve-frame.step) · [See the previous six-wheel campaign](../experiments/six-wheel/README.md)

