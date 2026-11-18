# Station receiver entry and first impact

At a 0.5 m/s incoming feed and 40 mm extra chassis stand-off, the 220 mm receiving extension redirects CORAL toward the station for both shelf shapes studied. Extending to 240 mm reverses that result. The tube initially moves into the robot for every tested restitution at the same stand-off. A further 20 mm of stand-off loses that advantage. Keep the 240 mm extension as a candidate for a station-envelope and loaded-retraction check. The results cover the instant after an idealized first contact. They leave capture and a repeatable scoring cycle open.

## Recovering the design intent

The preloaded floor-patch study in [the original design](DESIGN.md) starts after acquisition. The [kickoff exercise](kickoff/README.md) compares station-fed CORAL placement with ALGAE support under a common clock. A separate development record in this repository at Git revision `092dffaace65afc79392f4a710b546541a6af5c6`, `STRATEGY.md` and `B4.md`, describes station-fed L1 as the first investigation and explicitly leaves competition role selection open. Its differential-chassis and slide candidate uses 220 mm extension, a 265 × 420 × 15 mm shelf, and a 200 mm raised illustrative reserve. The preserved 160 mm reserve interfered in the earlier study. The intended hold, deliberate release and operator recovery states remain distinct. Students still own the choice of role and retention mechanism.

The [station-edge departure](STATION-EDGE.md) and [receiver contact](RECEIVER-CONTACT.md) records supply both the coordinate registration and the preceding square-entrance result. Original vertical panel datum is X = 0 mm. With 434.525 mm centered chassis stand-off and a 220 mm extension the front starts at +49.525 mm. With 240 mm the same entrance is +29.525 mm. Positive X points into the robot. This one nominal separation from the panel leaves other station material, moving/controlled CORAL, off-axis approaches and G413.A contact duration to check. The later English manual and the V0 release exercise require separate scoring reviews. [FIRST REEFSCAPE manual](https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf) · [Team Update 16](https://firstfrc.blob.core.windows.net/frc2025/Manual/TeamUpdates/TeamUpdate16.pdf)

## Editable mechanical candidates

Open the complete [square shelf STEP](cad/station-receiver/shelf-r0.step) or [6 mm rounded shelf STEP](cad/station-receiver/shelf-r6.step). The [dimension file](cad/station-receiver/design.json) and [CAD builder](cad/station-receiver/build_shelves.py) define both solid sections. The 6 mm treatment removes material from the top entrance edge while preserving the square shelf's bounding box. Each STEP reopened as one valid solid with bounds X = 0…265, Y = −210…210, Z = −15…0 mm. Volumes were 1,669,500.000 and 1,666,255.220 mm³. Translate the shelf in the station frame to test slide-extension and chassis stand-off independently. The parts omit supports, mounting pattern, guide hardware, compliance and finish. Material and allowable loading remain unspecified.

## What the comparison measures

The model follows the centered 301.625 mm transverse tube, radius 57.15 mm. After station separation its center follows the registered [gravity trajectory](scripts/station_edge_contact.py). The rounded entrance is a circle of radius 6 mm in side elevation. First contact occurs on the receiver top or when center-to-arc-center distance equals the sum of radii in the upper entrance quadrant. A clear result ends at center-at-arc-center height. Later underside or side contact is outside this interval.

For first contact, the surface normal points into the incoming tube. With an immobile rigid receiver and zero tangential impulse the normal velocity changes sign and is multiplied by restitution. Inward velocity below uses dimensionless trial restitution 0, 0.25 and 0.5 in order. Tangential impulse, friction, spin transfer, actual CORAL orientation, material properties and deformation need separate evidence. No time after this instantaneous response is integrated.

| 0.5 m/s feed, extra stand-off | Extension | Entrance radius | Robotward speed immediately after impact (m/s) |
| --- | ---: | ---: | --- |
| 40 mm | 220 mm | 0 mm | −0.014, −0.126, −0.239 |
| 40 mm | 220 mm | 6 mm | −0.081, −0.209, −0.338 |
| 40 mm | 240 mm | 0 mm | +0.388, +0.377, +0.366 |
| 40 mm | 240 mm | 6 mm | +0.284, +0.246, +0.209 |
| 60 mm | 240 mm | 6 mm | −0.081, −0.209, −0.338 |

The full screen covers three assumed speeds, six stand-offs, two radii and two extensions. Of 72 cases, 30 strike the top, 14 strike a square corner, 14 strike the fillet and 14 clear the surface through the cutoff. At 1 m/s, 60 mm extra stand-off, 220 mm extension and the 6 mm radius, the three speeds are +0.170, +0.007 and −0.155 m/s. A measured contact response could change the direction in that configuration. Neither a short-lived positive velocity nor an impact on top establishes seating, maintained support or legal scoring.

## Reproduce and qualify

1. Run `python scripts/station_receiver_impact.py` to print all 72 cases with times, normals and velocities. Run `python -m unittest discover -s tests` to check sign conventions, normal/tangential impulse and dissipation, the inherited square-geometry boundaries and clear-through controls.
2. With OCP 8.0.1 available, run `python cad/station-receiver/build_shelves.py artifacts/receiver-comparison` to generate and reopen both candidates. The builder checks their volumes and bounding boxes against separate section formulas.
3. Run `python cad/station-receiver/verify_contacts.py` against the checked-in STEP solids. Compare with the [six-case receipt](cad/station-receiver/contact-receipt.json). The independent finite outer-cylinder checks measure separation 0.5 ms before impact, tangency at the analytical time and intersection 0.5 ms later **if response is deliberately suppressed**. The deeper placement is a negative control. Each receipt identifies the source and STEP bytes. These selected states do not establish a swept whole-robot clearance.

A subsequent integrated acquisition experiment must carry the same pose, piece identity and state from actual intake through retraction, retained transport, intended release and a second station pickup. Preserve failures and timeouts. Check the changed reach against the full station envelope, deployed perimeter limits, controlled-piece condition and interference with the chassis, bumper, carriage and partner. The L1 score predicate, release test, measured forces and cycle clock are independent gates.
