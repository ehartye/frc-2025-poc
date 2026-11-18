# 2025 REEFSCAPE kickoff narrative

Status: student draft for review.

## Release-day evidence

Kickoff · V0 — https://manuals.plus/m/e98a748723b248f34e890accab1c49c809276fe44de713321834e30736d1e04a.pdf

Field: 690.875 × 317 in. §5.1, p. 19. Rectangular field extents.

R101–R104. Rectangular robot perimeter excluding bumpers and height in starting configuration.

§5.3, pp. 23–24. Each reef is centered across the field. Its modeled center is 144 in plus half its width from the alliance wall. The rings mark elevations only.

L1 lip 18 in · L2 tip 31.875 in · L3 tip 47.625 in · L4 tip 72 in

Corners, barge, stations, game pieces, bumpers and deployed mechanisms are omitted. Field outlines and height rings support scale comparisons only.

## Student observations

OBS-1 — Analyst observation: 30 × 30 in uses the full rectangular perimeter budget. The 30-in box top is above the L1 lip, which gives no release or scoring evidence.
Box 30 × 30 × 30 in at (173, 106) in, heading 0°. Perimeter 120 in · 0 in remaining. Starting height 30 in · 12 in remaining. Box top is 12 in above L1 lip.

OBS-2 — Analyst observation: trading one inch of width for one inch of length keeps the 120-in perimeter while changing the approach footprint.
Box 31 × 29 × 30 in at (173, 106) in, heading 0°. Perimeter 120 in · 0 in remaining. Starting height 30 in · 12 in remaining. Box top is 12 in above L1 lip.

## Scoring opportunities and route requirements

OP-1 · Reef trough (L1) · TELEOP
V0 §§6.3.4, 6.4 and 6.5, Table 6-2. See G409 for simultaneous control, G412 for CORAL launch location and the scoring criteria for each target.
6 on field marks. Up to 6 robot preloads. The remainder is split between alliance stations (57–60 per alliance).
Distinct inventory 126. Opening allocation 4, plus 0 at 0 s. Window 15 s. Value 2 points/piece. Slots unspecified.
Assumptions: Analyst hypothesis: four distinct CORAL are available in sequence at the same station. One robot starts ready to receive. No late allocation. Candidate service counts pickup and release only. The separate route schedule accounts for each loaded and return transit.
Route: A (45, 42) in → B (122, 158.5) in · 139.647 in at assumed 60 in/s.
Traffic inference: High potential overlap · up to 2 other start slot(s) near one point (Partner slot 2, Partner slot 3). V0 §§5.1–5.6, 6.3 and Figure 6-2. One alliance frame is mirrored for the other. Start X derives from the starting line and reef dimensions. Three lateral starting slots, pickup and scoring approach coordinates are illustrative robot-center positions in inches. Opening AUTO routes can also use a preload. Actual team routes and arrival times are unknown.
Trial box 30 × 30 × 30 in. Candidate cycle 1.5 s/piece.
Supply and target ceiling: 4 pieces · goal 3.
Transit 2.327 s + other non-scoring time 1 s leaves 11.673 s.
Required throughput: 0.257 pieces/s (at most 3.891 s/piece).
Candidate: 0.667 pieces/s. At most 4 scored for 8 points. Meets the arithmetic screen.
Provisional scoring requirement: sustain this opportunity’s required throughput over its usable window. Resolve supply shortfalls before adopting the target.
Capture mechanism and exposed state: Station-side CORAL guides and retention path
Provisional contact requirement: assess the exposed capture mechanism along this route and verify retained function under the specified contact case.
Contact load case and basis: Possible side contact with the capture assembly in the carrying state. Direction, point, magnitude, duration and mount load path need evidence.
Retained function and verification: After the specified contact case, retain the piece, release on command, and complete the next acquisition within the separately timed service budget.

OP-2 · Processor · TELEOP
V0 §§6.3.4, 6.4 and 6.5, Table 6-2. See G409 for simultaneous control, G412 for CORAL launch location and the scoring criteria for each target.
12 on reef branches and 6 on field marks. Processor transfers can make a piece available for another scoring action.
Distinct inventory 18. Opening allocation 1, plus 0 at 0 s. Window 15 s. Value 6 points/piece. Slots unspecified.
Assumptions: Analyst hypothesis: one accessible reef ALGAE is allocated at the pickup pose. One robot starts ready to acquire. No follow-on supply or credited downstream NET event. Service includes acquisition and processor release.
Route: A (176.75, 101) in → B (236, 23) in · 97.952 in at assumed 60 in/s.
Traffic inference: High potential overlap · up to 2 other start slot(s) near one point (Partner slot 2, Partner slot 3). V0 §§5.1–5.6, 6.3 and Figure 6-2. One alliance frame is mirrored for the other. Start X derives from the starting line and reef dimensions. Three lateral starting slots, pickup and scoring approach coordinates are illustrative robot-center positions in inches. Opening AUTO routes can also use a preload. Actual team routes and arrival times are unknown.
Trial box 30 × 30 × 30 in. Candidate cycle 4 s/piece.
Supply and target ceiling: 1 pieces · goal 1.
Transit 1.633 s + other non-scoring time 1 s leaves 12.367 s.
Required throughput: 0.081 pieces/s (at most 12.367 s/piece).
Candidate: 0.25 pieces/s. At most 1 scored for 6 points. Meets the arithmetic screen.
Provisional scoring requirement: sustain this opportunity’s required throughput over its usable window. Resolve supply shortfalls before adopting the target.
Capture mechanism and exposed state: ALGAE capture and retention while traveling to the processor
Provisional contact requirement: assess the exposed capture mechanism along this route and verify retained function under the specified contact case.
Contact load case and basis: Possible side contact with the capture assembly in the carrying state. Direction, point, magnitude, duration and mount load path need evidence.
Retained function and verification: After the specified contact case, retain the piece, release on command, and complete the next acquisition within the separately timed service budget.

## Role and repeatable cycle

Candidate A: receive one CORAL, carry it to the L1 trough, place it, return and repeat if timing and access permit. No team selection recorded.

## Alternative or fallback

Candidate B: remove one accessible reef ALGAE and deliver it to the processor. Partner access, pickup feasibility and contact tolerance remain open.

## REQ-1 · Provisional capability

Time each pickup, loaded transit, release and empty return. Preserve retention during the selected contact case and record the next cycle outcome.

## TEST-1 · First experiment for REQ-1

Measure the travel and service components separately against the three-CORAL window. Compare the route and retained capture function before choosing a role.

Evidence for capability review: OBS-1, OBS-2, OP-1, OP-2.

Before technical design, record the team decision, then give the selected capability a measurable acceptance check.
