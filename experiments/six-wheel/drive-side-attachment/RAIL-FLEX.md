# Inner drive rail under axle thrust

Under a conditional 500 N side load delivered to one end bearing, the uniform-web calculation gives **0.258 mm** of sideways travel with fixed ends or **2.789 mm** with pinned ends. The previous calculation checks the ear, weld and bolts. A joint can carry its reactions while the rail between the joints bends. [Joint and alignment requirements](README.md) · [Joint loads](loads.py)

The model uses the 100 mm tall by 6 mm thick web, the project's assumed 69 GPa aluminum modulus, support lines at X ±312 mm, and the end bearing at X 247.65 mm. The 4 mm deck meets the top edge of the rail at Z150. Planar seating alone does not establish reversible restraint against lateral load. The calculation omits deck friction, the shafts, opposite rail, adapter stiffening, carrier holes, take-up tabs, joint compliance and torsion from the actual load height. The two ideal end conditions compare sensitivities. They do not bound the complete chassis. [Issued rail and ears](joint.py) · [Axle location and take-up](../design/drivetrain/README.md) · [Project material assumption](../camera-datum-shop/README.md)

| Applied load case, on one rail | Pinned ends | Fixed ends |
|---|---:|---:|
| 250 N at nominal end bearing | 1.395 mm | 0.129 mm |
| 500 N at nominal end bearing | 2.789 mm | 0.258 mm |
| 500 N at inward take-up limit, X 243.65 | 3.102 mm | 0.303 mm |
| 500 N at nominal bearing, supports moved to inner edge of ears at X ±300 | 1.838 mm | 0.146 mm |
| 500 N at X0, sensitivity only | 20.378 mm | 5.094 mm |

The forces are assumed test cases. The driver's route does not establish this thrust, and the center clip and shim can send thrust to the outboard bearing. Large center results flag the importance of the assumed load path and the limits of small-deflection theory. Axial translation along a shaft and angular bearing misalignment are separate quantities. The assembly's diametral alignment tolerance cannot serve as an allowable axial displacement. [Thrust paths and endplay](../axle-retention/README.md)

**Design decision.** Keep lateral stiffness open on the shop review. Before selecting a brace or spending the remaining mass allowance, resolve which bearing takes thrust in each direction from measured stack endplay. With the chassis restrained and the drive unpowered, a fixture can then load that stack incrementally up to an independently established safe test load. Record applied force, relative inner-to-outer carrier travel, joint slip, socket engagement and residual motion after unloading. An independent chassis datum distinguishes rail bending from movement of the whole fixture. A validated elastic model or fixture result and allowable bearing/socket travel are needed to accept the assembly.

Run `python drive-side-attachment/rail_flex.py` to reproduce both forces, support locations and all three end-axle positions. The calculation uses displacement at the applied load. For span L and distances a and b from the load to the supports, the pinned result is P a² b² / (3 E I L). The fixed result is that displacement multiplied by a b / L². The web's weak-axis I is 1,800 mm⁴.
