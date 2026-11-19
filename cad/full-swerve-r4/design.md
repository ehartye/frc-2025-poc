# White 1780 lettering on the fixed-height swerve robot

Team 1780 appears in white on all four sides of the actual red 2025 bumper. The project reuses its 2024 digit outlines at their original size. Each digit is 105 mm high, the major strokes measure 15 mm and digit origins are 80 mm apart. The rows sit between Z59.5 and Z164.5 mm, with 17 mm cloth margin above and below. The front and rear numbers sit at the midpoint of their covers. The left side number sits on the front half, centered at X+200 mm. The right side number sits on the rear half, centered at Xâˆ’200 mm. Neither side marking crosses the removable joint. [Numbered model](full-robot.step) Â· [Ordered color and part register](appearances.json) Â· [Source outlines and placements](markings.py) · [Independent comparison of all four white glyph areas to the issued 2024 model](2024-reference.json)

The four outward normals and reading directions are registered in robot coordinates. From outside the chassis, each face reads **1780** left to right. Each digit is a separate, named, 0.05 mm display layer representing white print on the cloth. When the layer is shifted 0.05 mm inward, the underlying 2025 cover backs its entire area. The original nominal layers have zero material overlap. This gives 16 separate contact witnesses, including both holes in the 8 and the hole in the 0. The labels are appearance geometry and carry no independent fabricated-part or mass credit. Fabric printing, inspection and real-world wear remain open. [Per-digit fit](lettering.json) Â· [Project palette](../component-palette.json)

The R4 exchange contains 975 solids. The public version includes 24 project-drawn module representations and three supplier envelopes. The straight-guide pose, physical bumper and other registered component bounds carry forward from R3. Its 352 previously positioned STEP leaves have been baked into robot coordinates to keep each physical solid connected to its identifier and appearance when imported in other CAD tools. [Previous assembly and sources](../full-swerve-r3/design.md) Â· [Student Fusion handoff](../FUSION.md)

```sh
python cad/full-swerve-r4/markings.py <installed-agent-frc> <ignored-project-output>
```

The command rebuilds the public STEP and its ordered appearance register, checks all sixteen text solids against the native cover shapes, and writes the locally importable project-only geometry and resolved appearance palette to the output directory. Manufacturer module geometry stays in the authorized local Fusion assembly.

