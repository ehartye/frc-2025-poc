# A linked receiver in the full robot

The full Team 1780 Fusion assembly now lets a student drive the scoring receiver with one slider. The 370 mm outer slide moves at twice the rate of the middle stage. Six native machining features produce two continuous underside lands from the existing insert and polymer facing. Open the project-local archive using the [student Fusion instructions](../FUSION.md).

| Commanded slide | Measured middle | What to inspect |
|---:|---:|---|
| 0 mm | 0 mm | The two absorber lands sit above their stowed contacts. |
| 185 mm | 92.5 mm | The moving camera stays with the pipe shelf. |
| 370 mm | 185 mm | Both stages reach their extension limits. |
| 0 mm | 0 mm | The moving bodies return to their initial coordinates. |

Fusion exported the stowed assembly, closed it and reopened the 142,839,502-byte archive in a fresh document. At each of the four positions, the test compared all six bounding-box coordinates for 278 outer-slide occurrences, 30 middle-stage occurrences, the front bumper and four supplier module assemblies with their predicted translations. The largest difference was below 0.000001 mm. The two facing cuts and four insert features survived the reopen, with volumes of 665,317.129 and 317,740.720 mm³ respectively. The linear draw-wire representation shows its stowed shape. The separate dynamic harness study supplies its changing path. [Native receipt](native.json) · [Motion groups](motion-groups.json) · [Continuous-land geometry and load assumptions](../../experiments/swerve-drive/score-stroke/study.md)

The full archive contains supplier geometry and stays in the local project artifacts. To rebuild from the issued full R4, run [the project Fusion script](fusion_receiver.py) from its checkout folder in Fusion and select the R4 archive. The script works on an imported copy, applies the six machining features, creates the two slider joints and the 1:2 motion link, checks all four positions, and leaves the model stowed for export. Selecting the issued articulated archive runs the travel checks on its existing joints. Both archive digests are in the receipt. For an openly downloadable native exercise, edit [the two cam solids](../../experiments/swerve-drive/score-stroke/cam-lands.f3d).

The [film](../../experiments/swerve-drive/score-stroke/receiver-motion.webm) uses a simulated station pickup and a separate scoring fixture. In the [loaded route study](../../experiments/swerve-drive/transit/README.md), the robot stopped with its first pipe aboard. Reaching this stroke in CAD adds no match delivery.
