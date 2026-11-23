"""Single source of the B'x pocket section geometry (robot datum, millimetres).

Robot datum = the iteration 002 chassis root: chassis centre on the factory-tire floor (Z = 0), +X toward the
reef (delivery) face, +Y port, +Z up. The pocket is a planar section in XZ extruded along Y; the pipe axis is
along Y (axis-across). build_layout.py (solids) and field_trials.py (free-piece physics) both import this file,
so the checked blocks and the physics trials use the same surfaces.

This design keeps every first-draft pocket surface and adds the flared hood lead-in (HOOD_FLARE). The drive, tread
band, sensors and structure records changed; see field_trials.py and structure_checks.py.

Revision of the evaluation's B geometry (evaluation/README.md, r2):
  The evaluation places a 70 mm fence under the lower roller with the nip throat at pipe-centre height 0.67 m
  and the seated pipe at 0.60 m. No two-roller nip can engage a pipe seated 70 mm below its throat and behind
  a wall taller than the pipe radius, so the evaluated "nip lifts it over the fence" has no contact that can do
  it (finding PD1 in design-findings.json). This package instead seats the pipe in the bite of the nip: the
  throat is at the pocket-centre height (0.60 m), the two treads are the front stop and a self-locking worm
  reducer held the rollers when unpowered (the first draft; this design holds them with the powered drive). The 15.74 deg pocket floor, the 10 mm hood, the one-way check flap
  200 mm behind the stop and the 0.55 m roll-off tip of the deployed lip are kept from the evaluation.
"""
import math

R = 57.15                  # CORAL outer radius (4.5 in OD, project constant)
PIPE_LENGTH = 301.625
PIPE_RI = 50.8             # inner radius, as the pan solver uses it
PIPE_MASS = 0.816466266    # upper bound (evaluation custody.coral.M)

FRAME_FRONT = 370.0        # frame perimeter face toward the reef
FRAME_REAR = -370.0
BUMPER_FACE = 445.5        # front bumper outer face (layout bumper ring, 75.5 mm stack)

# ---------------------------------------------------------------- floor and inlet (pipe-contact surfaces)
FLOOR_DEG = 15.74          # evaluation custody.B.feedSegmentAnglesDeg[1:], kept for the 2.76 m/s2 rollback screen
CHUTE_DEG = 52.31          # evaluation feedSegmentAnglesDeg[0]
ENTRY = (-340.0, 795.0)    # station-side inlet lip (pan-downselect feed[0])
POCKET_Z = 600.0           # pocket-centre (= throat) height, evaluation geometry

# ---------------------------------------------------------------- nip (front stop)
ROLLER_R = 25.4            # 2 in 35A compliant wheels
SQUEEZE = 6.0              # per-roller tread compression at the throat (12 mm total, evaluation bench article)
THROAT_X = 335.0           # tread front at 360.4 mm, 9.6 mm inside the frame face
ROLLER_DZ = R + ROLLER_R - SQUEEZE
LOWER_ROLLER = (THROAT_X, POCKET_Z - ROLLER_DZ)
UPPER_ROLLER = (THROAT_X, POCKET_Z + ROLLER_DZ)
CORE_R = ROLLER_R - SQUEEZE - 3.0   # rigid hub-and-core reserve; the outer 9 mm of tread is the designed squeeze + 3 mm
SHAFT_R = 6.35                       # 1/2 in hex shaft (across-flats half)

# Unloaded first-contact bite: centre where the pipe just touches both uncompressed treads.
BITE_X = THROAT_X - math.sqrt((R + ROLLER_R) ** 2 - ROLLER_DZ ** 2)

_t = math.tan(math.radians(FLOOR_DEG))
_c, _s = math.cos(math.radians(FLOOR_DEG)), math.sin(math.radians(FLOOR_DEG))
# Floor line: tangent R below the bite centre, along the 15.74 deg slope (falls toward +X).
_FLOOR_PT = (BITE_X - R * _s, POCKET_Z - R * _c)


def floor_z(x):
    return _FLOOR_PT[1] - _t * (x - _FLOOR_PT[0])


def _kink():
    # Intersection of the 52.31 deg chute from ENTRY with the floor line.
    tc = math.tan(math.radians(CHUTE_DEG))
    # ENTRY.z - tc (x - ENTRY.x) = floor_z(x)
    x = (ENTRY[1] + tc * ENTRY[0] - _FLOOR_PT[1] - _t * _FLOOR_PT[0]) / (tc - _t)
    return (x, floor_z(x))


KINK = _kink()
FLOOR_END_X = LOWER_ROLLER[0] - ROLLER_R - 3.0   # floor stops 3 mm short of the lower tread (roller slot)
INLET = [ENTRY, KINK, (FLOOR_END_X, floor_z(FLOOR_END_X))]   # pipe-contact polyline, station to roller slot

# ---------------------------------------------------------------- hood and check flap
HOOD_GAP = 10.0            # evaluation: hood 10 mm above the pipe
POCKET_LENGTH = 200.0      # evaluation: flap to stop, 200 mm pocket
FLAP_X = BITE_X + R - POCKET_LENGTH          # flap plane (hanging), 200 mm behind the bite front
HOOD_OFFSET = 2 * R + HOOD_GAP               # normal distance floor -> hood underside


def hood_z(x):
    return floor_z(x) + HOOD_OFFSET / _c


HOOD_END_X = UPPER_ROLLER[0] - ROLLER_R - 3.0
HOOD = [(FLAP_X - 40.0, hood_z(FLAP_X - 40.0)), (HOOD_END_X, hood_z(HOOD_END_X))]
# Flared hood lead-in. The station drop bounces (trials: centre up to about 765 mm at e 0.5) and can meet
# the hood's leading edge; a 90 mm flare 30 deg above the hood line turns a high pipe down into the pocket.
HOOD_FLARE_LEN = 90.0
HOOD_FLARE_DEG = 30.0
_fa = math.radians(FLOOR_DEG + HOOD_FLARE_DEG)
HOOD_FLARE = [(HOOD[0][0] - HOOD_FLARE_LEN * math.cos(_fa), HOOD[0][1] + HOOD_FLARE_LEN * math.sin(_fa)), HOOD[0]]
FLAP_HINGE = (FLAP_X, hood_z(FLAP_X) - 4.0)  # hinge pin 4 mm under the hood underside
FLAP_LENGTH = FLAP_HINGE[1] - floor_z(FLAP_X) - 6.0   # hangs to 6 mm above the floor
FLAP_OPEN_DEG = 90.0 - FLOOR_DEG   # opened parallel to the hood, 3.2 mm under it (the pipe passes 5.2 mm below)

# ---------------------------------------------------------------- deploy-once exit lip
LIP_HINGE = (366.0, 530.5)          # 3 mm knuckle: 3.3 mm outside the unloaded lower tread, 0.6 mm under the lip reserve top, stowed lip inside the frame face
LIP_ROOT_X = 367.5                  # ramp root ahead of the lower tread (the root's underside clears the unloaded tread)
LIP_RAMP_DEG = 30.0                 # deployed ramp angle: 20 and 30 deg capture; 40 deg stalls in the nip (root above the tread)
LIP_TIP_X = FRAME_FRONT + 100.0     # evaluation B'x: the pipe leaves 100 mm beyond the frame line
LIP_TIP_SURFACE_Z = 550.0 - R       # leaves with its centre at 0.55 m (evaluation screen)
LIP_LAND = 25.0                     # flat land at the tip so the pipe leaves horizontally
LIP_T = 3.175                       # 1/8 in polycarbonate


def lip_profile(ramp_deg=LIP_RAMP_DEG, land=LIP_LAND):
    """Deployed lip top surface: root -> ramp -> flat land -> tip (mm)."""
    tip = (LIP_TIP_X, LIP_TIP_SURFACE_Z)
    knee = (tip[0] - land, tip[1])
    a = math.radians(ramp_deg)
    root = (LIP_ROOT_X, knee[1] + (knee[0] - LIP_ROOT_X) * math.tan(a))
    return [root, knee, tip]


# Stowed, the lip folds UP and stands in front of the nip (ramp vertical, land leaning back toward the upper roller),
# 1.0 mm inside the frame face and >= 2.3 mm from both unloaded treads through the whole swing (layout-blocks.json
# treadClearance). Deploy is a +120 deg rotation about +Y (viewed +X right, +Z up: clockwise, forward and down). The
# stowed centre of mass sits about 3 mm behind the hinge, so the torsion spring alone starts the swing; gravity assists
# after the first few degrees. The pipe's weight on the land and a frontal tip push both load it INTO its deployed stop.
LIP_STOW_DEG = -120.0               # rotation about +Y from deployed to stowed
LIP_DEPLOY_DEG = -LIP_STOW_DEG

# ---------------------------------------------------------------- field: red L1 trough section
# FIRST's official trough at the docked sector (lip GE-25302, floor GE-25313, riser GE-25315). build_layout.py compares each
# value with the measurement on the placed official solids (evidence/official-fixtures.json) and stops beyond 0.05 mm.
TROUGH = dict(lipTop=457.2, lipDepth=12.7, floor0=404.9106592228664, slope=0.2679491924311204,
              riserX=247.8366565710712, riserTop=517.525461057565, riserT=6.35,
              sectorHalfWidth=233.5)   # half the docked floor panel's Y extent
NOMINAL_STANDOFF = 50.0               # bumper face to trough lip face at the shown dock (screen sweeps 0-100 mm)


def trough_x(standoff):
    return BUMPER_FACE + standoff
