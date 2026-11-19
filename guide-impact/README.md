# Does the installed guide still seat the pipe?

## What changed at the station

The two stationary guides meet the ends of the incoming pipe before it reaches the shelf. Each has a 3 mm liner over a folded, 0.8 mm aluminum backing. Two arms attach each backing to the mast at heights of 687 and 787 mm. The liner runs from 625 to 813 mm. A native solid classification checks nine heights on each side, both face edges and both ends. The 260 inside/outside measurements match the surfaces used by the pipe model. [Guide assembly](../guide-mounts/README.md) · [Reopened STEP measurements](section.json)

The nominal 301.025 mm throat places each guide 0.300 mm inside the end of a centered, nominal 301.625 mm pipe. The separately fitted fastener position moves each guide outboard by 0.315 mm, giving 0.015 mm of clearance per end. The opening and contact timing depend on how the installer locates the two guides before tightening. [Fitted throat positions](../guide-mounts/datum.json)

## Load path and contact

Each sloping backing has 24 Euler–Bernoulli beam elements. Its two support springs combine arm bending, elastic root bending and tension, and bolt-couple rotation. The sloping beam spans the actual free lower and upper overhangs. The backing and liner move with their masses from the reopened STEP. The liner contributes mass only. A nominal 23.6% of each arm assembly's modeled mass participates at its nose with the full wedge, tapped bar and two face screws. This is a reduced mass assumption. A physical root may slip, twist or lift off, and the actual arm shape can carry different modal mass. [Beam equations and material inputs](beam.py) · [Underlying root calculation](../guide-mounts/loads.py)

At each integration step, the openly ended pipe contributes 512 candidate quadrature cells on each end. Cells outside the installed 40 mm face or its 188 mm height cannot push a guide. Each penetrating cell sees the local moving surface, its instantaneous normal and its relative sliding speed. Its compressive and friction forces advance the same pipe translation and rotation used by the shelf and belt plant. The opposite lateral force acts at that cell's height on the flexible backing. The actual force distribution advances both arms through the beam modes. The static calculation's single force and center of pressure provide an independent comparison. [Paired contact calculation](backing.py) · [Machined shelf and damper plant](../shelf-structure/README.md)

The input contact law keeps the earlier 10 N/mm and 200 N·s/m distributed coefficients, friction coefficient 0.1 and 0.02 m/s friction transition. The nominal guides use a modal damping ratio of 0.03. The pipe falls with its attained velocity, meets finite-preload belt cartridges, and drives the existing 20 ms Java acquisition program through two moving aperture rays. The first tick at the chute lip is Java 100 ms. A fault ends that case. A seated case continues until the scheduled Java 5,460 ms sample. The backing's retained modes advance exactly for a load held over one plant step. Contact forces, local normals and belt traction change each step. [Powered approach poses](../conditioned-match/elastic.json) · [Controller replay](replay.py)

## Reproduce the study

Use the project Python, native CAD and desktop JDK environments. Set the numerical library to one thread for a reproducible modal basis. The following commands run from the project root. Replace the angle-bracket paths with the local JDK and private artifact folders.

```shell
python guide-impact/section.py guide-impact/section.json
python guide-impact/replay.py <jdk> <raw> <raw>/folded.json --layouts folded --steps 62.5 31.25
python guide-impact/replay.py <jdk> <raw> <raw>/controls.json --layouts flat half-couple unribbed --steps 62.5 31.25
python guide-impact/replay.py <jdk> <raw> <raw>/rigid.json --layouts rigid --steps 31.25
python guide-impact/replay.py <jdk> <raw> <raw>/opening-gauge.json --layouts folded --outward .315 --steps 62.5 31.25
```

Each run preserves the pipe trajectory, its orientations and velocities, the shelf and plunger restart state, and a guide force/work history under its own configuration ID. The guide history has 15 columns in this order. Time is seconds from the lip, followed by both normal forces in N, both force-weighted heights in m, both lower-tip displacements in m, both upper-tip displacements in m, both peak absolute face displacements in m, both modal energies in J and both cumulative guide input works in J. Paired values always follow chassis −Y, then +Y. Peak travel uses a 1 mm height grid during possible pipe/guide overlap. The result includes one exact free decay to the final pipe clock. The installed modal damping, friction, preload, material law and receiver lash all require measurements.

## Shop measurements that decide transfer

Gauge both openings with the final hardware torqued. At both arm heights and both free ends, apply a measured transverse load and measure travel on loading and unloading. A displacement witness on the mast separates bracket/root motion from backing bending. Measure vibration decay after release and measure the actual liner force against the open pipe wall. Recheck the two Java port signals and both camera views with the working shelf, real piece, visible finish and signed approach offsets. A passing modeled port sequence supplies a candidate state for that optical replay. It grants no new match score. [Previously qualified optical and carry boundary](../elastic-carry/README.md)
