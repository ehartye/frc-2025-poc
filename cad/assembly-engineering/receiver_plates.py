"""Print the newly machined interface sheets from the checked neutral issue."""

import pymupdf as pdf
from drawings import sheet
from attachments import base
from installed_audit import sha
from service_variant import PRODUCTION

PAGES = [
    (
        "RC-01",
        "Grip drive plate and integral cheek bosses",
        [
            "grip-R-cartridge-mount",
            "grip-R-upper-cheek",
            "grip-R-output-tie-east-outer",
            "grip-R-cheek-tie--49.5-165.5",
        ],
        [
            "Two opposed sets, 6061-T6. Output centre (-84.50, +/-183.5625), 22.70 clearance, 3.00 plate at Z581..584.",
            "Four M3x8 under each output. Hole circle 32.00. Pattern offsets along X +/-13.8564, along Y +/-8.00.",
            "Four M3x10 above each 6.80 boss. Set Z581.0 boss tops in one setup. Fit and spin check the unchanged driven spur.",
            "First article check: source output flange seats, max runout and received 22 mm boss clear the finished bore.",
        ],
    ),
    (
        "RC-02",
        "Adjustable end switch",
        [
            "slide-retracted-switch",
            "slide-retracted-switch-lever",
            "slide-retracted-switch-bracket",
            "slide-retracted-switch-shoulder-screw-0",
            "slide-retracted-switch-locknut-0",
        ],
        [
            "Two opposite stations. Housing spacing 6.50. Closed 2.10 wide slots allow +/-1.65 setting around hole-axis Z449.70.",
            "Clamp with paired 1.90 smooth shoulders, 8.40 effective grip and M2 threaded tails. Fasten bare retracted bracket first. Fit switch before its end stop.",
            "Use the STEP coordinates for the two underside socket pockets. Lower pocket relief 6.70 diameter where indicated.",
            "Set lever 0.35 past observed operation at the plateau, with acceptance +/-0.10. Do not set from the nominal hole height.",
        ],
    ),
    (
        "RC-03",
        "Middle-stage cam and two physical stops",
        [
            "slide-middle-stop-dog",
            "slide-retracted-stop",
            "slide-retracted-stop-M3",
            "slide-middle-dog-M2-0",
            "slide-middle-dog-M2-1",
        ],
        [
            "Machine dog, cam and tongue as one blank. At stow X-97.5, Y-150.5. Flat 8, flanks rise 4.5 in 15, lower Z456.15.",
            "Two M2x6 in the 6.65 mm rail ligament. Attach to intermediate before fitting the outer pan. Final shelf stroke 370.",
            "Both stop blocks: 3.8 x 4.0 x 1.2 locating tongue. Bridge pocket 4.0 x 4.2 x 1.3. One underside M3x8 per block.",
            "The tongue supplies lateral bearing after its 0.10 mm side gap. Inspect stops and simulate braking at the intended speed.",
        ],
    ),
    (
        "BA-02",
        "Matched battery keeper and retention rods",
        [
            "battery-hold-down",
            "battery-front-tie-rod",
            "battery-rear-tie-rod",
            "battery-front-compression-sleeve",
            "battery-rear-compression-sleeve",
        ],
        [
            "Insulating FR4 cap, 3/16 in stock. Two 197 mm M5 rod cuts and two 166.80 mm compression sleeves.",
            "Select and measure NP18-12B in the nominated orientation. Machine stop faces to case size plus 0.90 per side.",
            "Keeper underside Z324.80. Tighten against the sleeves. Insulate terminals; assumed lid/guard contact needs actual verification.",
            "Disconnect leads. Remove upper nuts, keeper and sleeves. Lift 250 vertically before carrying the dressed pack sideways.",
        ],
    ),
    (
        "PW-01",
        "Fixed SB50 through-mount",
        [
            "SB50-fixed-housing",
            "SB50-base-0-spacer",
            "SB50-base-1-spacer",
            "SB50-base-0-M3x25",
            "SB50-base-0-locknut",
        ],
        [
            "Use the 992G1-family fixed housing and two M3x25 throughs. Tool and bearing checked against the vendor solid.",
            "At X-284: Y-34.30 spacer 1.889 long, Y-15.25 spacer 1.127 long, both OD6/ID3.4. Datum is top of tray Z155.",
            "Drill 3.40 through tray and deck. Top heads seat at Z168.827. 7 mm OD washers and 4 mm deep AF5.5 locking nuts beneath.",
            "Public housing is an installation envelope. Check paired color key, contact engagement, insulation, polarity and lead strain.",
        ],
    ),
    (
        "CB-01",
        "Upper fixed USB guide",
        [
            "roof-boom-mast-socket",
            "roof-guide-6061-boom",
            "guide-stay-head-plate",
            "roof-mast-rear-saddle",
            "guide-cable-keeper",
        ],
        [
            "Aircraft Spruce 03-35500-3, 6061 tube 3/8 x .035 in wall, cut 114.865. Two positively cross-pinned 20 mm sockets.",
            "Two M3x25 bolts clamp the mast from outside. Locator 3.00 in 3.20 bore through ONLY right outboard 2 mm mast wall.",
            "Upper cable nominal OD6.30 with 6.60 passage. Use matched receiving limits for jacket, keeper and allowed bend radius.",
            "Support assembly and withdraw 205 outboard. Rear saddle moves down32, in14, rearward75, then out239.",
        ],
    ),
]


def build():
    path = PRODUCTION / "station-and-jack-interfaces.step"
    parts = base.named(path)
    doc = pdf.open()
    for i, (key, title, ids, notes) in enumerate(PAGES):
        sheet(doc, title, [parts[k] for k in ids], notes, sha(path), i, key, len(PAGES))
    doc.set_metadata(
        {
            "title": "Receiver, battery and upper guide assembly sheets",
            "author": "Team 1780 design study",
        }
    )
    (PRODUCTION / "assembly-drawings.pdf").write_bytes(
        doc.tobytes(no_new_id=True, deflate=True, garbage=4)
    )


if __name__ == "__main__":
    build()
