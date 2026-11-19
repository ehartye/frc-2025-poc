"""Build the student stroke comparison from the reopened geometry and force fixture."""

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


def build():
    geometry = json.loads((HERE / "geometry.json").read_bytes())
    balance = json.loads((HERE / "balance.json").read_bytes())
    fixture = json.loads((HERE / "fixture.json").read_bytes())
    witness = json.loads((HERE / "dynamics-witness.json").read_bytes())
    edge = [row for row in witness["cases"] if row["localCenterMm"][1] == 42]
    case = next(iter(balance["cases"]))
    data = []
    for stroke in geometry["strokeMm"]:
        s = str(stroke)
        row = balance["cases"][case]["strokes"][s]
        f = fixture["cases"][case]["nominal"][s]
        data.append(
            dict(
                stroke=stroke,
                contacts=geometry["contacts"]["L"][s],
                button=geometry["fixedButtons"][s][0],
                load=row,
                pipe=balance["cases"][case]["pipeLocalXMm"] + stroke,
                tip=f["frontChangeFromStowMm"],
                middle=f["middle"]["rearReactionN"],
                inner=f["inner"]["rearReactionN"],
            )
        )
    template = (HERE / "viewer.template.html").read_text(encoding="utf8")
    (HERE / "index.html").write_text(
        template.replace(
            "__STUDY__", json.dumps(data, separators=(",", ":"), allow_nan=False)
        ).replace(
            "__EDGE__", json.dumps(edge, separators=(",", ":"), allow_nan=False)
        ),
        encoding="utf8",
        newline="\n",
    )


if __name__ == "__main__":
    build()
