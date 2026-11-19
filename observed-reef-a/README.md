# Can the shop robot see scoring sector A while carrying coral-A?

The existing fixed head can register the A sector when it can see the surveyed face and a complete subset of its fasteners. At the terminal decision the left-small wheel case is inside an 8.53 mm position bound and the right-small case is inside 7.88 mm. Both heading bounds are 0.74°. These desktop results use the signed, powered station starts and the 85% loaded gyro schedule. The earlier [two-pipe match replay](../conditioned-match/replay.html) retains its original observers and stops.

## What the field camera can actually see

The close approach frequently exposes both fasteners to one pupil and only one to the other. The control uses a 36h11 ID10 for coarse registration, keeps each head's native occurrence and pixel identity, and checks a padded, robot-depth-tested window around its measured face before accepting a centroid. Two distinct identified fasteners can transport the moving estimate. The final gate additionally requires two complete features in one eye and a corroborating measurement in the other. The planar pose interval uses the pixel rays of the entire complete eye and adds separate survey and stopping allowances. No image uses an interpolated fastener for an occluded one. [Field trials](field-trials.json) · [January field and source geometry](../sector-alignment/scene.json)

The robot receives 35 ms captures after 15 ms and integrates shaft travel at 5 ms. The three fresh A permits arrive at wall 47.827 s on the left-small path and 47.792 s on the right-small path. Both chassis stop within about 1.5 mm of the intended center in the simulated field frame. Labelled dropout, duplicate candidate, 75 ms stale packet and wrong face each inhibit both signed approaches. A common 14 mm survey displacement admits a false positive on both. Before transferring the 10 mm / 1° gate, survey the installed field and the loaded camera independently.

At the A approach, accounting for 10 mm planar displacement and 1° heading error leaves at least 135.06 mm to the occupied reef ball. The bound uses the 991-body stowed register, two actual guide plates, the finite carried-pipe enclosure and 629 optional fixed-harness boxes. The student robot stays stowed throughout this approach. The common power calculation retains the two-camera host, rack hold and both grip controllers. It allows up to 8 A phase at each grip and 0.032 m/s surface speed throughout carry. [Native stock bounds](stock.json) · [Six-robot, installed stock and bus](field.json)

## Reproduce

Use the project Python environment, registered field assets, and the original two signed pickup and drive traces produced by the [conditioned match](../conditioned-match/README.md).

```sh
python observed-reef-a/trials.py "$ARTIFACTS"
python observed-reef-a/native_bounds.py "$ARTIFACTS" "$LAMP_STEP"
python observed-reef-a/stock.py "$ARTIFACTS"
python observed-reef-a/field_screen.py "$ARTIFACTS"
python observed-reef-a/retention.py "$ARTIFACTS"
python observed-reef-a/retention.py "$ARTIFACTS" --fine
```
