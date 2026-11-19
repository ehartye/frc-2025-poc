"""Draw the signed field paths in an offline, student-readable HTML page."""

import argparse
import json
from pathlib import Path


def generate(source, out):
    register = json.loads(source.read_bytes())
    slim = {
        case: {
            name: dict(
                path=row["timedPath"],
                peer=row["field"]["minimumPeerMm"],
                stock=row["field"]["minimumStockMm"],
                wall=row["field"]["minimumBarrierMm"],
            )
            for name, row in item["routes"].items()
        }
        for case, item in register["cases"].items()
    }
    template = Path(__file__).with_name("plot-template.html").read_text(encoding="utf8")
    out.write_text(
        template.replace("/*REGISTER*/null", json.dumps(slim, separators=(",", ":"))),
        encoding="utf8",
        newline="\n",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("source", type=Path)
    p.add_argument("out", type=Path)
    a = p.parse_args()
    generate(a.source, a.out)
