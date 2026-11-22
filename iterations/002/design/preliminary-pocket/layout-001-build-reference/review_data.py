import json, sys, hashlib, math, re
T = sys.argv[1]
A = 'WORKSTATION/repos/frc-2026-poc-worktrees/preliminary-design-review'
B = '.'
R26 = A + '/cad/preliminary-design/releases/layout-001'
R25 = T + '/r25'
sha = lambda p: hashlib.sha256(open(p, 'rb').read()).hexdigest()
def pin(root, f): return dict(file=f, sha256=sha(root + '/' + f))
def sup(f):
    d = json.load(open(T + '/' + f, encoding='utf-8'))
    for v in d.values():
        if v['supplied'] and v['path'] and ' mounts on ' in v['path'][0]:
            v['path'][0] = 'Group datum: ' + v['path'][0]
    return d
def notsup(reason): return dict(supplied=False, reason=reason)
NOTE = ("Project-authored inputs for the standardized preliminary review scene. Values cite their source; "
        "anything without a source is recorded as not supplied.")

# ---------------- 2026
L = json.load(open(R26 + '/layout.json', encoding='utf-8')); g = L['groups']
circ = 1059.18 / 2
hexc = [[round(circ * math.cos(math.radians(90 + 60 * k)), 2), round(circ * math.sin(math.radians(90 + 60 * k)), 2), 1828.8] for k in range(6)]
timed26 = {}
for c in ['acquire-approach', 'acquire-climb', 'acquire-over', 'acquire-drop']:
    timed26[c] = dict(seconds=[0.0, 0.5254], trial='acquire-nominal', basis='first carpet contact to capture')
for c in ['launch-wrap', 'flight-1', 'flight-2', 'flight-3', 'flight-4']:
    timed26[c] = dict(seconds=[0.0, 0.9303], trial='deliver-nominal', basis='release from the hood exit to the HUB sensor-plane crossing')
timed26['climb-lift'] = dict(seconds=[0.0, 0.4548], trial='hang-nominal', basis='lift stroke complete; then a 3 s brake hold to 3.4548 s')
for c in list(L['stages']) + ['package-displacement-control']:
    timed26.setdefault(c, notsup('No trial or rehearsal times this prescribed case'))
d26 = dict(
    schema='frc-2026-poc/preliminary-review-data@1', release='layout-001', note=NOTE,
    runningAdditions=dict(
        source=pin(R26, 'budgets.json'),
        basis="budgets.json runningPayloadBasis: 'Bumpers 6.0-7.5 kg, battery and leads 5.0-6.9 kg and 24 FUEL at 0.203-0.227 kg: all carried in the match but outside the R103 weighing'",
        items=[dict(name='bumpers', kg=[6.0, 7.5], members=g['bumpers']['members'], basis='R103 excludes bumpers; 6.0-7.5 kg per budgets.json'),
               dict(name='battery and leads', kg=[5.0, 6.9], members=g['battery']['members'], basis='R103 excludes the battery and its connector half; 5.0-6.9 kg per budgets.json'),
               dict(name='24 FUEL', kg=[round(24 * 0.203, 3), round(24 * 0.227, 3)], members=g['stored FUEL']['members'] + g['tracked FUEL']['members'],
                    basis='24 FUEL at 0.203-0.227 kg each (budgets.json); the three placed FUEL represent the carried stock')],
        totalKg=[15.872, 19.848]),
    support=dict(source='layout.json mounts (group-level datums); structure-to-frame joints beyond these datums are not registered', members=sup('sup26.json')),
    modelNotes=[
        'What it is: the R3-PD2 whole-robot preliminary blockout (tall floor-fed hooded-flywheel shooter on a 680 x 714 mm four-module swerve base), layout-checked by Agent FRC. Solids are planning envelopes and simplified material blocks, not a Fusion robot.',
        'Simplified: swerve modules are body, motor and azimuth envelopes; electronics are two panel blocks; belts and rollers are rigid reserves (3 mm undeformed reserve for guided FUEL paths); the HUB and TOWER are V0-dimension fixtures, not the official field model.',
        'Prescribed, not simulated, in the package: every case motion, including the carried FUEL path. Free-piece contact physics is in the field-interaction trials (field-trials.json), whose contact stiffness, damping, friction and FUEL release speed and spin are estimates, not measurements.',
        'Unverified: verification gates G1-G6 (verification-gates.json) carry the physical tests; hopper capacity is a free-volume premise; dynamic BUMP crossing and robot-to-robot contact are not evaluated.',
        'Sources and cutoff: FRC REBUILT kickoff manual V0 with Team Update 00, cutoff 2026-01-10 23:59:59 America/New_York (construction-rules.json); field dimensions from the V0 description (geometry.py field_bodies).'],
    focus=dict(component='FUEL tracked', basis='The mission cases (acquire-drop, flight-4) follow this FUEL from the carpet into the HUB'),
    caseTimes=dict(source=pin(R26, 'field-trials.json'),
                   note='Package cases are prescribed 25-frame geometric paths. Times come from the nominal field trials; a case group shares its trial interval.',
                   cases=timed26),
    targetAperture=dict(
        name='HUB opening (top)',
        source=dict(rule="construction-rules.json hub: 'Section 5.4; front edge 72 in, 41.7 in hexagonal opening, 47 in footprint'",
                    geometry='geometry.py hexagon_prism: corners on +/-Y, flats facing +/-X', file=pin(R26, 'construction-rules.json')),
        interpretation='41.7 in (1059.18 mm) treated as across-corners, the smaller conservative inscribed opening (construction-rules.json)',
        frame='HUB-local millimetres: origin at the HUB centre on the carpet, +X/+Y parallel to the robot datum axes',
        cornersMm=hexc,
        placement=dict(shotPoseHubCentreMm=[-2700.0, 0.0], basis='build_package.py hub_shot: HUB centre 2700 mm aft of the robot centre at the delivery pose')))
d26['support']['members']['FUEL tracked'] = dict(path=[], supplied=False, reason=(
    "Not bound with transportedBy: the package path is a constructed waypoint path (build_package.py tracked_path and the "
    "ballistics flight arcs), not a trace of the acquire or deliver trials. The checker requires the pinned trial to record "
    "this exact case path, so binding it would assert evidence the trials do not provide. To close: generate the acquire, "
    "launch and flight case moves from the field_trials.py traces and write one transport record per case, as the 2025 "
    "package does."))
d26['openItems'] = [
    "No registered structure-to-frame joint: hopper walls (hopper front upper wall, hopper rear wall, hopper wall left, hopper wall right).",
    "No registered structure-to-frame joint: climber tubes and winch (climber tube left, climber tube right, climber winch) to the climber tube mounts.",
    "Frame-internal joints are not registered: frame rails, floor pan, floor pan front tongue, front lip, climber tube mounts and intake slide rails.",
    "Battery hold-down strap over the battery top is undesigned; the cleats capture it in X only.",
    "Group-level mount datums: right-hand members (climber hook right, climber inner tube right, intake carriage plate right, shooter side plate right) cite the left-hand datum."]
d26['modelNotes'].append("Open items: see openItems (unregistered structure joints and undesigned retention).")
open(A + '/cad/preliminary-design/review-data-layout-001.json', 'w', encoding='utf-8', newline='\n').write(json.dumps(d26, indent=1, ensure_ascii=False) + '\n')

# ---------------- 2025
L = json.load(open(R25 + '/layout.json', encoding='utf-8')); g = L['groups']
lip, riserX, riserTop, hw = 457.2, 247.8366565710712, 517.525461057565, 233.5
xl = 495.5; xr = xl + riserX
timed25 = {}
for c in ['receive-chute', 'receive-ramp', 'flap-open', 'receive-pocket', 'receive-seat']:
    timed25[c] = dict(seconds=[0.0, 0.5434], trial='receive-seat-time premise (G1 receive model, 40 N/mm tread)', basis='arrival to seated in the worm-locked bite')
for c in ['eject-throat', 'eject-over-roller', 'eject-lip-root', 'eject-ramp', 'eject-land', 'release-off-tip']:
    timed25[c] = dict(seconds=[0.0, 0.187], trial='eject-nominal (G2 worm 12 V, 50 mm stand-off)', basis='nip start to departure from the lip tip')
for c in ['release-flight', 'trough-landing', 'withdraw-park']:
    timed25[c] = dict(seconds=[0.187, 3.254], trial='eject-nominal', basis='departure to captured and supported 2 s after withdrawal')
for c in list(L['stages']) + ['package-displacement-control']:
    timed25.setdefault(c, notsup('No trial or rehearsal times this prescribed case'))
d25 = dict(
    schema='frc-2025-poc/preliminary-review-data@1', release='layout-001', note=NOTE,
    runningAdditions=dict(
        source=pin(R25, 'budgets.json'),
        basis="budgets.json runningPayloadBasis: 'Carried CORAL 0.816466 kg plus the R103-excluded battery 6.17 kg and one-colour bumpers 7.5 kg'",
        items=[dict(name='one-colour bumpers', kg=[7.5, 7.5], members=g['bumpers']['members'], basis='R103-excluded; 7.5 kg per budgets.json'),
               dict(name='battery', kg=[6.17, 6.17], members=g['battery']['members'], basis='R103-excluded battery 6.17 kg per budgets.json'),
               dict(name='carried CORAL', kg=[0.816466, 0.816466], members=g['pipe']['members'], basis='One carried CORAL 0.816466 kg per budgets.json')],
        totalKg=[14.486466, 14.486466]),
    support=dict(source='layout.json mounts (group-level datums) and load-paths.json', loadPaths=pin(R25, 'load-paths.json'), members=sup('sup25.json')),
    modelNotes=[
        "What it is: the Red 2 iteration 002 B'x preliminary blockout (fixed pocket, worm-held two-roller nip, deploy-once exit lip) on the carried-forward four-module swerve chassis, layout-checked by Agent FRC. B'x is held for review; the guarded-pan baseline A remains the fallback until bench gates G1-G4 pass.",
        'Simplified: the chassis, module housings and electronics are oriented-box unions or supplier-bounds boxes; the steer sweeps are 4 mm-stepped revolved envelopes; pipe-contact surfaces are reserved 2.5 mm below their contact line and the rollers by their rigid core (rights gate in custody-layout-001.json).',
        'Prescribed, not simulated, in the package: the pipe path; contact transport is declared per case and bound to the evidence/transport records. Free-piece receive, eject and retention physics is in field-trials.json, a planar compliant-contact model whose stiffness, friction and restitution are bracketed, not measured.',
        'Unverified: bench gates G1-G5 (bench-gates.json, withheld with the private package); the G412 eject interlock is open (rules-rulings.json); the L1 trough is a planning section from the 2025 field model extraction, not the official field model.',
        'Sources and cutoff: rule register Kickoff V0, 2025-01-04 release-day packet, V0 only (rules-rulings.json).'],
    focus=dict(component='CORAL pipe', basis='The mission cases (receive-seat, eject-land, trough-landing) follow the pipe from the station chute into the trough'),
    caseTimes=dict(source=pin(R25, 'evidence/field-trials.json'),
                   note='Package cases are prescribed 25-frame geometric paths. Times come from the nominal field trials and the receive-seat-time premise.',
                   cases=timed25),
    targetAperture=dict(
        name='L1 trough sector A opening',
        source=dict(geometry='pocket_geometry.py TROUGH (target.json section of the 2025 field model extraction) and trough_x(NOMINAL_STANDOFF = 50 mm)', file=pin(R25, 'pocket_geometry.py')),
        frame='Robot datum millimetres at the shown dock (lip face 50 mm ahead of the bumper face)',
        cornersMm=[[xl, -hw, lip], [xl, hw, lip], [round(xr, 3), hw, round(riserTop, 3)], [round(xr, 3), -hw, round(riserTop, 3)]],
        basis='Outer lip top edge to inner riser top edge across the 467 mm sector A panel'))
d25['openItems'] = [
    "Carried-forward chassis reserves (Chassis perimeter, cassettes and socket rails, pan trusses and stringers, sills, decks and crossmembers) keep their joints inside 2.1 mm partition shells; no joint is registered individually.",
    "Tray supports register one group datum (port floor post on the upper pan stringer) for Floor post port, Floor post starboard and both rear inlet uprights.",
    "Bumper ring mount to the chassis perimeter is undesigned.",
    "Electronics, wiring and the two sensors use proposed host-envelope attachment points; their brackets are undesigned.",
    "Group-level mount datums: starboard members (Starboard nip side plate, Rear inlet upright starboard, Floor post starboard) cite the port datum."]
d25['modelNotes'].append("Open items: see openItems (unregistered structure joints and undesigned brackets).")
s = json.dumps(d25, indent=1, ensure_ascii=False)
assert not re.search(r'iterations/|artifacts/|/Users/|\.f3d', s)
open(B + '/cad/preliminary-design/review-data-layout-001.json', 'w', encoding='utf-8', newline='\n').write(s + '\n')
print('ok', len(d26['support']['members']), len(d25['support']['members']))
