"""Build the offline student comparison from the published contact and mass receipts."""

import json
from transfer import HERE, read


def build():
    transfer = read(HERE / "transfer.json")
    order = read(HERE / "order.json")
    labels = {
        "centered": "Centered remainder, unpaired parts",
        "twenty": "Unpaired parts, ±20 mm remainder",
        "paired-only": "Matched parts, unrestricted battery interior",
        "paired-and-packed": "Matched parts, ±10 mm remainder",
        "paired-and-packed-adverse": "Matched parts, ±30 mm remainder",
    }
    groups = {}
    for key, label in labels.items():
        r = transfer["cases"][key]
        groups[label] = {
            "side": r["minimumSideFraction"],
            "left": r["signed"]["leftSmall"]["lateral"]["wallS"],
            "right": r["signed"]["rightSmall"]["lateral"]["wallS"],
        }
    wheels = {
        label: order["cases"][key]["maximumEndFraction"]
        for key, label in (
            ("unsorted", "Unsorted wheels, 90–110% rates"),
            ("centers-largest", "Largest wheels centered, 90–110% rates"),
            ("wide-rates", "Largest wheels centered, 75–125% rates"),
        )
    }
    page = (
        (HERE / "viewer.template.html")
        .read_text(encoding="utf8")
        .replace("/*DATA*/null", json.dumps([groups, wheels], ensure_ascii=False))
    )
    (HERE / "index.html").write_text(page, encoding="utf8", newline="\n")


if __name__ == "__main__":
    build()
