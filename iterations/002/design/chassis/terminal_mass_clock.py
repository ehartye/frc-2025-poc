"""Replace estimated end tubes with four measured cartridges and retain the whole splice reserve."""
import hashlib
import json
from pathlib import Path

from crossframe_clock import run as replay


HERE = Path(__file__).resolve().parent


def run():
    native_file = HERE.parent / "cad/private/port-504-native.json"
    native = json.loads(native_file.read_bytes())
    baseline = json.loads((HERE / "low-line-frame-clock.json").read_bytes())
    low_line = json.loads((HERE / "low-line-frame.json").read_bytes())
    folded = json.loads((HERE / "folded-crossframe-screen.json").read_bytes())
    density = {"002 6061-T6 tube": 2700, "ATI 17-4 H1150, 7.86 g per cubic cm": 7860, "Steel": 8000}
    articles = {row["path"]: {"material":row["material"], "volumeMm3":row["volumeMm3"],
                "rhoKgM3":density[row["material"]], "allocatedKg":density[row["material"]]*row["volumeMm3"]*1e-9}
                for row in native["coldCartridge"]}
    if len(articles) != 16:
        raise ValueError("Incomplete measured cartridge")
    per = sum(row["allocatedKg"] for row in articles.values())
    four = 4*per
    old = low_line["addedMassKg"]["fourEndTubesKg"]
    reserve = folded["remainingAttachmentAllowancesKg"]["remainingMiterSplicesAndLowerRailTiesKg"]
    # Charge all the new bolts, blocks and spacers while keeping the full previous splice allocation.
    # No reduction for machining the four rails or for any finished member that weighs less.
    high = four-old
    general = {}
    for label, original in (("low-line-budget", baseline["nominal"]), ("heavy-offset-control", baseline["heavyOffset"])):
        mass = original["massKg"] + high
        # Keep adverse lateral CG offsets even though the four equal cartridge additions are symmetric.
        cg = list(original["cgMm"])
        cg[2] = max(cg[2], (original["massKg"]*cg[2] + high*177.8)/mass)
        general[label] = (mass, original["yawKgM2"] + high*.4**2, cg)
    return dict(source=native["verifiedAuthority"], articleNativeReceiptSha256=hashlib.sha256(native_file.read_bytes()).hexdigest(),
                articles=articles, fourCartridgeKg=four, replacedFourEndTubesKg=old,
                retainedMiterAndTieReserveKg=reserve, upperAddedAllocatedKg=high,
                lowerAddedIfEntireSharedReserveSpentKg=high-reserve, clock=replay(general),
                scope="Conservative four-cartridge projection from the 504 native volumes. Installed material uses 8.0 g/cm³ for unspecified screw stock. The previous fully shared miter/tie allowance remains available for the six still-unmodeled joints. "
                      "The CAD term changes field mass and yaw inertia. Pocket machining and a future sensor/motor purchase revision receive no mass credit. Six-robot occupancy, actual motor harness, final bumpers, certification and measured field tracking remain open.")


if __name__=="__main__":
    result=run()
    (HERE/"terminal-mass-clock.json").write_text(json.dumps(result,indent=2)+"\n")
    print(result["fourCartridgeKg"], result["upperAddedAllocatedKg"])
    for name,case in result["clock"]["cases"].items():
        print(name,case["fieldKg"], min(item["marginN"] for item in case["traction"].values()))
