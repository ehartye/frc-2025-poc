"""Repeat the support and frame comparison at the positions clear of the two front modules."""
import json
from perimeter_elastic import compare, p


def run():
    results = {}
    for height in (76.2, 101.6):
        for stiffness in (3000, 30000):
            for lost_attachment in (None, (100, 352.3)):
                name = f"h{height}-k{stiffness}-lost{lost_attachment}"
                results[name] = compare(height, [-100, 0, 100], stiffness, lost_attachment)
    return results


if __name__ == "__main__":
    target = p / "through-frame-elastic.json"
    target.write_text(json.dumps(run(), indent=2) + "\n")
