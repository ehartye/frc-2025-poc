"""Create the student optical replay from measured windows and first-party mount silhouettes."""

import argparse
import base64
from io import BytesIO
import json
from pathlib import Path
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent


def build(assets):
    from hashlib import sha256

    data = json.loads((HERE / "station.json").read_bytes())
    trials = json.loads((HERE / "obstruction.json").read_bytes())
    fields = json.loads((HERE / "sector.json").read_bytes())
    payload = {}
    for installation, model in data.items():
        register = model["register"]
        file = assets / "mounted-guide-vision" / (installation + "-labels.npz")
        if sha256(file.read_bytes()).hexdigest() != register["bodyLabelSha256"]:
            raise ValueError("Different sensor field")
        masks = np.load(file)
        eyes = []
        for eye in ("left", "right"):
            labels = masks[eye]
            rgb = np.full((*labels.shape, 3), [239, 242, 232], dtype=np.uint8)
            for code, body in register["bodyPerGray"].items():
                color = (
                    [215, 103, 53]
                    if body == "guide-L-687-mount-arm"
                    else [187, 143, 78]
                    if body == "guide-L-backing"
                    else [113, 139, 140]
                )
                rgb[labels == int(code)] = color
            png = BytesIO()
            Image.fromarray(rgb).save(png, "PNG")
            eyes.append(
                "data:image/png;base64," + base64.b64encode(png.getvalue()).decode()
            )
        rows = {}
        for r in trials:
            if r["path"] != "new-actual" or r["installation"] != installation:
                continue
            field = next(
                q
                for q in fields
                if q["installation"] == installation
                and q["wheelCase"] == r["wheelCase"]
                and q["mode"] == "nominal"
            )
            rows[r["wheelCase"]] = {
                "frames": r["samples"],
                "shortMm": -field["finalBodyErrorMmDeg"][0],
            }
        payload[installation] = {"eyes": eyes, "sides": rows}
    html = (
        (HERE / "viewer.template.html")
        .read_text(encoding="utf8")
        .replace("STUDY_DATA", json.dumps(payload, separators=(",", ":")))
    )
    (HERE / "viewer.html").write_text(html, encoding="utf8", newline="\n")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets", type=Path)
    build(p.parse_args().assets)
