# Retain and service the six drive axles

<!-- cad-still -->
[![Fixed pocket 1.0, preliminary design](../../../docs/robot/fixed-pocket.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Project CAD · Fixed pocket 1.0, preliminary design · in-progress.** Whole-robot blockout on four-module swerve: fixed pocket, held two-roller nip and deploy-once exit lip. Parked pose at the end of the withdraw-park case, lip deployed and pocket empty; field parts, the delivered pipe and the two service hand gauges are hidden. It is not detailed CAD. [Open the design review](https://ehartye.github.io/frc-2025-poc/preliminary-design/)
<!-- /cad-still -->

The two center axles could slide through their entire 10 mm socket engagement even with an end screw fitted. The new design closes the outward load path with a removable radial clip and thrust washer. Ten end screws and shims close the exposed shaft ends. Twelve covers capture the flanged bearing outer rims against their carriers. The installed shop map adds **103–150 g** to the selected robot, leaving **753 g of represented headroom** at the upper estimate. Finished weight, axial proof load and durability require a built assembly. [Interactive section and service stages](explore.html) · [Axle kit](axle-kit.step) · [Bearing keepers](bearing-keepers.step) · [Combined accounting](accounting.json)

## Cut and procurement schedule

Dimensions are nominal millimetres in robot coordinates. Positive |Y| points outward. Keep the finished machining dimensions with the actual shaft, bearing, shim and carrier serial or match-mark.

| Item | Installed quantity | Fabrication or receiving requirement |
|---|---:|---|
| Center and end axles | 2 × 113.0 mm, 4 × 93.6 mm | Cut REV-21-1135 untapped rounded hex stock, 12.7 across flats, 13.75 across rounded corners. The supplier identifies 7075 aluminum. Obtain lot/temper information. Cut-tap each accessible end #10-32 UNF, proposed class 2B. Keep a 17 mm full-diameter No. 21 pilot with drill-point clearance and at least 14 mm usable thread. Gauge the first part. |
| Center groove | 1 per center shaft | Locate the inboard groove flank 12.81 ±0.03 from the outside end. Turn Ø11.43 ±0.05 with 1.03 ±0.02 width and R0.10 maximum at both roots. Deburr the interrupted rounded-hex sections. Check the groove over a full rotation. Preserve the adjacent lands. |
| Radial retaining ring | 2 | Rotor Clip C-50, spring steel, using the unplated thickness range. Check finish, thickness, snap seating, speed and current drawing with the supplier. The purchase envelope in STEP represents rotation through 360°. The contact calculation uses the manufacturer's crescent profile. |
| Shaft end screw | 10 | REV-21-2805-PK25, #10-32 × 12.7. One 25-pack supplies one robot before spares. Maximum modeled head Ø17 × 3.8, 5/32 in drive. The screw bears on the shaft end. Inspect full engagement, the locking patch and underside contact. |
| End thrust shim | 6 outer, 4 inner | Machined steel. OD16.70, bore Ø13.90. Outboard thickness 1.55, inboard 1.15. Leave a 0.30-high annular nose, Ø15.00 inside, on the bearing side. Adjust thickness to the measured stack. The other face is flat to the screw. |
| Center thrust washer and wheel-side sleeve | 2 each | Steel washer 2.18 ±0.02 thick, with the same annular nose. Keep the clip-facing Ø13.90 opening square and verify contact after deburring. Shorten the existing 18 OD × 14.2 ID aluminum sleeve to the span |Y|310.70–314.05, about 3.35 long. Keep the two parts separate for radial service. |
| Bearing keeper | 12 | Machine 6061 aluminum to 3.10 thick, with a Ø26.80 opening and Ø32.00 × 1.65 flange pocket on the carrier side. Four Ø3.30 holes on a Ø37.80 bolt circle, clocked 14°, 104°, 194°, 284° from +X toward +Z. Four R3.95 lobes surround the holes on a Ø35 central disk. Countersink 90° to Ø6.80 on the wheel side. |
| Carrier screws | 48 | Émile Maurin 243013X8 or dimensionally qualified equivalent, M3 × 8, class 10.9, zinc, 2 mm hex, Ø6.72 maximum head. Drill the **selected** carrier with four Ø2.50 cut-tap pilots and tap M3 × 0.5 through its 6 mm thickness before assembly. The nominal engagement is 4.90 mm. Set the screws flush, verify underside clearance and qualify torque and locking on the actual carrier material. |

The ring-maker's 2017 catalog gives a C-50 groove Ø0.447–0.453 in, width 0.039–0.042 in, unplated thickness 0.033–0.037 in and 0.050 in minimum edge margin. Its web table reverses the width range. The design uses the catalog range and calls for current supplier and lot confirmation. Ring and groove ratings in the catalog have different bases. The cold-rolled-steel groove rating cannot establish capacity of this tapped aluminum axle. [Rotor Clip catalog, printed pp. 52–53](https://www.rotorclip.com/wp-content/uploads/2023/03/rotor_clip_catalog-2017.pdf#page=54) · [Product and tooling](https://www.rotorclip.com/product/c-50/) · [Installed preview](https://www.traceparts.com/els/rotorclip/en/api/viewer/3d?SupplierID=ROTOR_619713541&PartNumber=C-50)

REV's V2 screw drawing and rounded-hex stock table identify the purchased geometries. Guhring lists the No. 21 pilot as 0.1590 in, approximately 4.04 mm. Gauge thread fit, depth, stripping and actual head-to-shim bearing on the finished assembly. [Shaft stock](https://www.revrobotics.com/.5in-Rounded-Hex-Shafts/) · [Screw drawing](https://www.revrobotics.com/content/docs/10-32_Shaft_End_Screw_V2-DR.pdf) · [Tap drill chart](https://guhring.com/media/support/Decimal-Equivalents-drill-sizes-and-popular-tap-drill-sizes.pdf) · [M3 keeper screw](https://fixation.emile-maurin.fr/custom/docs/modele/24301_f1/vis-tete-fraisee-six-pans-creux-iso-10642-acier-classe-10-9-zn-pdf-51-ko-24301-lmod1.pdf)

## Two load paths at the bearing

The supplier 1916 STEP contains one solid with coaxial faces that represent the bearing envelope. Its bore-adjacent flat spans radii 7.129–9.525. The neighboring face is recessed 0.508 mm. Under the assumed ±0.20 mm radial error the shim nose stays on that flat with at least 0.171 mm inner and 0.975 mm outer margin. Each of the twelve thrust faces was registered to the actual supplier model and challenged at eight eccentric directions. Moving 0.05 mm beyond contact produced 2.116 mm³ common material per face. The face identities and loads need confirmation on the actual bearing. [Exact supplier STEP](https://www.revrobotics.com/content/cad/REV-21-1916.STEP) · [Face and clearance results](bearing.json)

The stationary keeper has a 0.0625 mm gap to the supplier's outer flange at nominal dimensions. Its inner opening stays at least 0.45 mm outside the recessed shield region at 0.25 mm offset. Without a cover the bearing can move 0.50 mm toward the wheel without engaging its carrier. The cover and carrier now stop opposite directions. The nearest neighboring light-clamp nut is 0.939 mm away. The conservative lower bound against fixed hardware through the ±4 mm adjustment is 0.599 mm. Install keepers on all twelve carriers, including the center inboard bearings. [Reopened keepers and adverse controls](keepers.json)

On a center axle the inboard groove flank presses the clip outward against the steel washer. That washer transmits the load to the rotating bearing face. The opposite screw and shim limit inward travel. At nominal fit the outward hard stop is about 0.15 mm from the registered axle position. Groove location, unplated ring thickness and washer thickness produce a calculated range of 0.053–0.254 mm before bearing internal play, elastic deflection or installation error. Verify socket insertion and bottom clearance at both measured extremes. Gauge total endplay after the keepers and actual bearings are installed.

## Service sequence and receiving trial

1. Support the de-energized robot with clear access below the wheel bay. Mark chain tension, axle adjustment, wheel orientation and each selected shim. Remove the corresponding bumper half for front/rear outer access, or both halves for the center seam. The installed bumper blocks the straight driver at all six outboard positions. The four inboard screws have clearance with the bumper installed.
2. For end-screw access, the study checks a Ø6 × 65 driver extension and a Ø22 × 70 handle through a further 16 mm axial withdrawal. Support the hub and sprockets. Take off the outside screw and shim and, on the four end axles, the inner screw and shim. Keep the mounting screws and belt/chain tension settings identified.
3. On a center shaft, unload axial thrust and park the loose washer against the bearing and the inner sleeve toward the hub. The radial corridor accepts the modeled 21 mm-wide, 0.75 mm-thick fork with a 57 mm blade. The wider handle stays below the keeper. The minimum modeled working clearance is 0.15 mm. The short-blade control intersects the keeper. The A-290 catalog lists blade width and length. Its undocumented thickness, flex, applied force and real handle shape must pass this fixture before tool approval. Orient the ring mouth toward the shaft during introduction. Confirm complete seating around the groove. [Tool data](https://rcwebcatalog.s3.amazonaws.com/applicator.pdf#page=1)
4. With the ring removed, support loose articles on a guide and pull the shaft outward. A continuous 160 mm native sweep clears the welded frame at each of the six registered wheel positions. The center cassette, welded frame and gearbox stay assembled for this shaft operation. Reinstall in reverse with components supported, gauge the clip visually, restore the measured shim stack and check axial play, wheel rotation, socket engagement and torque witness marks. [Extraction and tool sweeps](service.json)

An integral-shoulder alternative closed the geometric stop but blocked the ordinary outward pull. With the packed drivetrain, 1 mm outward reached the outer bearing and 0.20 mm inward reached the spacer. Removing an entire 88-body drivetrain side was feasible in nominal geometry with 14 mm inward translation and a 180 mm drop. It required a reversible 660 mm inner-rail joint and gang alignment fixture that the shop has yet to qualify. [Original free-shaft control](original.json) · [Preliminary integral-shoulder contact study](shoulder.json)

For a wheel change, remove both bumper halves and support the loose spacers. The [six-wheel removal procedure](wheel-service/README.md) covers the supported 8.8 mm drop, removal of one hub screw, and the alternate center-wheel route that moves the sleeve 1.50 mm. The 0.05 mm minimum nominal clearance requires a measured trial on the first assembly. [CAD clearance results](wheel-service/clearances.json)

## Loads and weight

The project drive screen supplies 747 N inner and 520 N outer radial reaction, 17.67 N·m peak bending and 19.15 N·m torsion. The axle sample uses a conservative Ø5 major-bore reservation at the minimum groove section. With 500 N axial force, applying the full historic bending and torque at the groove gives 179.74 MPa nominal equivalent stress. Limiting bending to the outboard reaction and its local 11.19 mm lever gives 129.23 MPa with the same pessimistic torque. Groove notch effects, fatigue, bearing thrust capacity, clip prying, startup reversals and local stress peaks remain test inputs.

| Assumed axial demand | Clip on groove shoulder | Clip on thrust washer | Washer on inner face | Keeper screw load, 2 sharing |
|---:|---:|---:|---:|---:|
| 250 N | 52.82 MPa | 23.72 MPa | 5.91 MPa | 125 N each |
| 500 N | 105.64 MPa | 47.45 MPa | 11.81 MPa | 250 N each |

At 500 N, a simplified keeper-lip calculation gives 28.03 MPa nominal bending with full-ring contact and 112.14 MPa if one quadrant carries the load. It treats the 1.45 mm lip as radial strips clamped at the Ø32 pocket, uniformly loaded across the 2.60 mm span with pressure based on a common annular contact band. Actual load distribution, cover flex, fastener prying and local notch stresses require proof on the selected carrier.

The clip figures bound mean pressure under the stated distributed parallel-contact assumption. The supplier's registered preview has 16.665 mm² projected washer contact when concentric and at least 10.537 mm² for any offset up to 0.20 mm. The groove calculation uses a smaller annulus entirely inside the cut hex shoulder. Have the shop agree the proof factor, cycles and acceptance criteria, then test a production-stock groove, keeper, two bearings and retained screw in both directions. Record applied force, slip, shaft return, witness marks, bearing drag and ring or groove damage after reversal. [Source-backed demand calculation](load-demand.json)

The map keeps the six shaft and two sleeve identities, replaces their prior mass once, and credits the twelve drilled carriers conservatively. It adds 24 made and 60 purchased instances. The upper mass estimate assumes the earlier 300 g cap per bare wheel and this study's receiving limits on all 48 M3 screws and ten end screws. The upper-end 0.753 kg allowance falls to roughly −0.747 kg if each bare wheel reaches the former 550 g cap. Weigh the selected loose items, actual robot and removable bumper sets before comparison with inspection limits. [Mass inputs](mass-input.json) · [Upstream shop bill](../design/selected-shop/README.md)

## Replay

Use the project CAD interpreter. The two vendor models remain external inputs identified by hash. The original project exchanges contain simplified screw and rotating clip envelopes, original machining features and the resulting carriers. The source mesh is used separately for contact area. The assembled robot, service tools and source vendor models are outside these two exchanges.

```text
python axle-retention/measure.py <artifacts> <out>/original.json
python axle-retention/fit.py <artifacts> <out>
python axle-retention/keepers.py <artifacts> <out>
python axle-retention/bearing.py <REV-21-1916.STEP> <out>/axle-kit.step <out>/bearing.json
python axle-retention/service_path.py <artifacts> <out>
python axle-retention/demand.py <C-50-installed-preview.zip> <out>/load-demand.json
python axle-retention/mass.py <installed-plugin> <selected-shop-map> <map-output>
```
