# Shop and procurement audit · 545-body robot

<!-- cad-still -->
[![Four-module swerve frame and mounting hardware](../../../../docs/robot/cad/assembly.png)](../../../../docs/robot/cad/index.html)

**Project CAD · Four-module swerve frame and mounting hardware.** Original robot structure for the swerve candidate. The project-local Fusion assembly adds four manufacturer module occurrences. [Source STEP · f8b75f2df4](../../../swerve-drive/packaging/swerve-frame.step) · [Rotate the model](../../../../docs/robot/cad/index.html)
<!-- /cad-still -->

A team can quote the named motors, drive articles and several machined families from this revision. A complete order and shop release still depend on unselected parts, loose hardware and physical interface evidence. The four hash-bound poses and historical inspection estimate identify the [installed 545-body robot](../competition-package/shop-duplex/README.md).

| Use | Record |
|---|---|
| Compare independent red/blue bills and the two carrier materials with end stops | [Reproducible assembly accounting](../assembly-accounting/README.md) |
| Locate an exact part in either alliance's assembly | [545-row installed crosswalk](installed.csv) |
| Count distinct articles represented by several solids | [305 grouped line items](item-rollup.csv) |
| Identify candidates, quantities, pack sizes and assembly-level order decisions | [Earlier purchasing schedule](../shop-package/purchasing.csv) · [electrical and inspection dossier](../electrical-inspection/README.md) |
| Catch items omitted from the visible solids | [Loose parts, unselected equipment and fabrication gaps](loose-items.csv) |
| Machine a candidate for the critical inner wheel clamps and gearbox attachment | [Dimensioned clamp sheet](clamp-sheets.svg) · [operation table](machining.csv) · [installed candidate STEP](inner-clamps.step) · [assembly and first article](CLAMPS.md) |

The crosswalk has 263 make-body records, 104 commodity fastener records, 89 named buy-candidate bodies, 86 bodies with part selection open, two reserved spaces and one included cable. Multiple geometric bodies can represent one ordered article. For example, the four primary rope segments represent two purchased working lines. Two backup line pairs and their splice envelopes represent two terminated lines. The SP1-25 measuring wire comes with its transducer. Each STEP contains a single alliance's bumper kit. Stock for a complete robot requires both removable color sets and one set of eight permanent frame sleeves. The combined motor/controller rollup includes three NEO V1.1 motors, three NEO 550 motors and six SPARK MAX controllers.

**Parts that still need a decision.** The historic lift's combined Sport/ratchet reserve has no matched, available gearbox, pinion and pawl selection. The two transfer reaches share one unresolved drive. Some receiver pulleys use only pitch/root envelopes. The sensor-board and cable volumes do not establish working harness assemblies. The backup drum core and flange IDs describe one drum per side, with unresolved winding retention. The lift safety follow-on has its [own engineering record](../receiver-lift/RETENTION.md). The clamp change here is a separate candidate and carries no physical proof rating.

The original nine drawing families and their 27-instance inspection record remain in the [shop package](../shop-package/README.md). Match the active revision for each affected ID. The drilled inner slide, pinned receiver anchors, drilled lift crossbar and modified masts supersede their original stock drawings. The [545-body pose registers](../competition-package/shop-duplex/red/part-frames.json), [shop-joint detail](../shop-package/JOINTS.md) and [duplex mass/geometry inventory](../receiver-lift/duplex-inventory.json) specify where to apply each operation.

Each route describes the make-or-buy intention and the state of its identity. “Make” can still require a supplier drawing, stock lot, process choice, tolerance or load test. “Buy candidate” identifies a supplied article or family and may use a packaging proxy. The item rollup counts distinct installed instances per alliance. Shared article family names carry multiple source records. Order quantities for kits, individual loose inserts, consumables, harnesses and the two color sets require the accompanying schedules.

Rebuild the two CSV tables with `python design/installed-procurement/inventory.py`. Rebuild the shop patterns with `python design/installed-procurement/drawings.py`. The native runner and its first-article limits are in the clamp notes.
