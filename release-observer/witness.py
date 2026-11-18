"""Consume pre-extracted synchronized observations offline. No device or network I/O."""
from math import hypot, isfinite
from stereo import clearance, bound_pair


def support_gap_upper(pipe_mark, shelf_mark, pixel_bound, registration_error_mm):
    """Upper bound between an observed piece-material point and a shelf-material point.

    Both labels must denote actual surfaces. An inferred center or empty bore cannot be used.
    Shared camera pose cancels in Euclidean distance. Pixel intervals remain independent.
    """
    a=bound_pair(pipe_mark,pixel_bound)
    b=bound_pair(shelf_mark,pixel_bound)
    differences=[max(abs(min(p[i] for p in a)-max(q[i] for q in b)),
                     abs(max(p[i] for p in a)-min(q[i] for q in b))) for i in range(3)]
    return hypot(*differences)+registration_error_mm


def witness(before,after,calibration,expected_attempt,piece_key,received_ms):
    """Return the interval covering the first crossing between two registered observations.

    Only applicable to a continuous non-reversing tracked release. Missing data or a
    contradictory frame resets the track. No fabricated positive for an unsupported camera.
    """
    for frame in (before,after):
        if frame["attempt"]!=expected_attempt or frame["piece"]!=piece_key:
            raise ValueError("unmatched piece or attempt")
        if (not frame["track_unambiguous"] or frame["calibration"]!=calibration["id"]
                or not frame["shelf_registered"]):
            raise ValueError("unregistered or ambiguous track")
    if after["source"]!="independent-stereo" or (before["source"] not in ("independent-stereo","lip-contact")):
        raise ValueError("wrong physical source")
    if after["track_epoch"]!=before["track_epoch"] or after["sequence"]<=before["sequence"]:
        raise ValueError("stale or interrupted tracking")
    if not calibration["physical_calibration"] or not calibration["id"]:
        raise ValueError("no verified physical calibration")
    err=calibration["pixel_error"]
    sync=calibration["clock_bound_ms"]
    reg=calibration["shelf_registration_mm"]
    size=calibration["pipe_size_bound_mm"]
    relative=calibration["surface_pair_bound_mm"]
    if not all(isfinite(v) and v>=0 for v in (err,sync,reg,size,relative)):
        raise ValueError("invalid bound")
    a,b=before["capture_ms"],after["capture_ms"]
    if not all(isfinite(v) and v>=0 for v in (a,b,received_ms)) or a+sync>=b-sync or b+sync>received_ms:
        raise ValueError("clock overlap, reversed frames or future capture")
    if before["source"]=="independent-stereo":
        if support_gap_upper(before["pipe_material"],before["shelf_material"],err,relative)>=1:
            raise ValueError("earlier frame cannot bound latest non-clear pose")
    elif not (before["healthy_loaded_edge"] and before["isolated_pipe_load"] and
              0 < calibration["installed_pad_max_mm"] < 1 and before["pad_id"] in calibration["pads"]):
        raise ValueError("unqualified lip contact")
    # The pad stack belongs to the shelf. Conservatively enclose its top everywhere.
    pad=calibration["installed_pad_max_mm"]
    if not isfinite(pad) or not 0 < pad < 1: raise ValueError("invalid installed pad envelope")
    gap=clearance(after["end_marks"],after["front_mm"],after["top_mm"]+pad,err,reg,size)
    if gap<1: raise ValueError("whole pipe clearance unresolved")
    return dict(clear=True,attempt=expected_attempt,pieceKey=piece_key,earliestCrossingMs=a-sync,
                latestCrossingMs=b+sync,lowerGapMm=gap,calibration=calibration["id"])
