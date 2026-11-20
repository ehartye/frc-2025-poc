# Upper fixed USB guide · prebuild assembly handoff

The higher fixed-side guide now connects the forward-routed USB cable to the right mast. The former long tower and its end rib are superseded. Keep the remaining lower guide supports in their own engineering scope. All dimensions are in robot millimetres. The boom is part of the fixed structure.

## Drawings and material

| Article | Drawing or stock | Purpose |
| --- | --- | --- |
| Lower cradle and lift-off keeper | `roof-usb-guide-and-yoke.step` (4353b232…) | Nylon 6/6 with a 6.60 mm passage around the nominal 6.30 mm USB cable. The keeper bottoms on its two side lands, so the fasteners can seat without compressing the nominal jacket. |
| Socket, boom, flanged plugs, rear saddle and revised mast | `short-boom-keyed-saddle.step` (17e56aca…) | Machined 6061 components and a 114.865 mm length of drawn 6061-T6 tube. This version replaces the plain yoke in the guide drawing and the original mast. It replaces both earlier through-bolt and collar variants. |
| Shoulder pins | Accu/AMPG SSH-M2-3-16-A2, two | Ø3 shoulder, 16 mm bearing length, 1.5 mm hex drive. Capture with M2 washer and locking nut. Machine Ø3.05 through the socket, tube and flanged plug in register. |
| Mast screws | Two M3×25 socket-head screws | The front bar clears the screws. The threaded ears integral to the rear saddle provide 8 mm nominal engagement. Qualify the thread retention and allowable screw torque in the selected temper. |
| Keeper screws | Two M3×20 socket-head screws, M3 washers and locking nuts | Provides positive capture at this cable station and clearance for unplugging or lifting the cable during service. |

The 3/8 in OD × .035 in wall tube is Aircraft Spruce 03-35500-3. The drawing represents shoulder-bearing and thread envelopes. Source models and full thread geometry are separate from the project-drawn interfaces. [Tube catalog](https://www.aircraftspruce.com/catalog/pnpages/03-35500-3.php) · [Shoulder screw drawing](https://www.accu.co.uk/api/product-datasheet?id=436237) · [Alloy properties](https://online.kaiseraluminum.com/depot/PublicProductInformation/Document/1071/Kaiser_Aluminum_6061_Pipe_Tube.pdf)

## Assembly and service

1. Keep the original two-wall mast section. Drill one Ø3.20 locator hole through the 2 mm outboard wall at X−55 and Z595.5. The integral Ø3 key projects 1.8 mm into this hole. The two M3 clamp screws pass outside the mast. The rear lips have 0.30 mm nominal side clearance.
2. Cut and square the tube, then fit the flanged plug at each end to the actual stock bore. The design's nominal plug body is Ø7.70 mm against the stock's 7.747 mm nominal ID. Control the socket fits against the chosen tube's actual OD tolerance. The pin secures the tube axially and rotationally. Its mating socket reacts transverse bearing load. The flange prevents the plug from slipping down the bore. Support it until the pin is seated.
3. Assemble the two blind sockets and tube on the bench while both ends can move. Pin each end. Support the USB cradle, then bring the assembly in from the outboard side of the robot and register the locator. Attach the rear saddle and both clamp screws. Install the cable into the open guide, fit the keeper and seat its two screws, washers and locking nuts.
4. To remove the boom, support the cable and trough, release the two keeper screw stacks and the two mast screws, and support the loose rear shoe. Withdraw the pinned boom 205 mm toward −Y. The inboard face then reaches Y−440 beyond the bumper.
5. Remove the rear shoe separately. From its installed position slide it 32 mm down, then 14 mm toward +Y, 75 mm toward −X and 239 mm toward −Y. Reverse the sequence for installation. Direct inward withdrawal runs into the right grip gearbox. See `saddle-extraction.json` for the tested source revisions.

## Engineering status

The reopened node and boom have positive seated faces, pin-bearing controls and clear parked tool paths. The 22 guide articles clear 344 moving receiver bodies through the registered combined motions. The swept bounds are separated by at least 3.438 mm. The locator hole reduces net mast area by 2.254% and local bending second moment about X by 2.932% at its center.

The 53.809 g boom-and-socket subtotal is a mixture of catalog mass for the tube/pins and upper-density geometry for the remaining articles. The prior plain guide and keeper contribute a separate 13.682 g. The retired high tower and end rib must be credited only once. The outboard locator key and close-fitting sockets provide positive registration in the modeled assembly. Procurement tolerances, clamp and mast strength, fatigue, electrical dress, the remaining lower supports and the final integrated Fusion issue retain their own acceptance checks. Bare tube elastic compliance of 0.03255 mm/N assumes a perfectly rigid socket; actual socket backlash, mast motion and cable forces require their own load model.
