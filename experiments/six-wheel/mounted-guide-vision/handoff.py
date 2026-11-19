"""Check which signed observations are available at the first departure service."""

import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
CAPTURES = (5372, 5389, 5406, 5423, 5440)
DEPARTURE_MS = 5460


def application_ms(captured):
    return 5 * math.ceil((captured + 15) / 5)


def handoff(case, mode):
    fixed = {row[0]: row[1] for row in case["fixed"]}
    roof = {row[0]: row for row in case["boundedGaps"] if row[1] == mode}
    observed = []
    for stamp in CAPTURES:
        candidate = roof.get(stamp)
        angle = candidate[-1] if candidate else None
        observed.append(
            fixed.get(stamp) == "position-witness"
            and candidate is not None
            and candidate[2] == "inside"
            and angle is not None
            and max(abs(bound) for bound in angle) <= math.radians(2)
            and application_ms(stamp) <= DEPARTURE_MS
        )
    return {
        "guard": mode,
        "pipeTrajectorySha256": case["pipeSha256"],
        "wheelCase": case["wheelCase"],
        "source": case["source"],
        "availableCapturesMs": list(CAPTURES),
        "lastAppliedMs": application_ms(CAPTURES[-1]),
        "nextFrameAppliedMs": application_ms(5457),
        "accepted": all(observed),
        "observationsPassing": sum(observed),
    }


def run():
    results = {}
    for filename in ("station", "setting-station"):
        source = json.loads((HERE / (filename + ".json")).read_bytes())
        for key, configuration in source.items():
            results[key] = [
                handoff(case, mode)
                for case in configuration["cases"]
                for mode in ("nominal", "guarded", "matched-finish")
            ]
    return results


if __name__ == "__main__":
    (HERE / "handoff.json").write_text(
        json.dumps(run(), indent=2) + "\n", encoding="utf8", newline="\n"
    )
