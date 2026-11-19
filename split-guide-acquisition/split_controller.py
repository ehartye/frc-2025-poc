"""Build the tested station controller variant from the frozen desktop program."""

from pathlib import Path
import hashlib


def candidate(feed):
    source = feed.port_program(True)

    def change(old, new):
        nonlocal source
        if source.count(old) != 1:
            raise ValueError("Historical station controller changed")
        source = source.replace(old, new)

    change(
        "    private String reason",
        """    private int darkFrames;
    private boolean observedClearPair, returnedSeat, recovery;
    private String reason""",
    )
    change(
        "                if (now - entered >= 260 && incomingClear",
        """                if (incomingSeatOnly && !entry && !seat) {
                    darkFrames++;
                    observedClearPair |= darkFrames >= 2;
                } else {
                    darkFrames = 0;
                }
                if (observedClearPair && seat && !entry) returnedSeat = true;
                if (now - entered >= 260 && now - entered <= 360 && returnedSeat && seat && !entry) {
                    recovery = true;
                    move(State.PROBE, now);
                } else if (now - entered >= 260 && incomingClear""",
    )
    change(
        "else if (now - entered >= 1600)",
        "else if (now - entered >= (recovery ? 760 : 1600))",
    )
    return source


def digest(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()
