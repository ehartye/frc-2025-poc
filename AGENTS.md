# Agent instructions

<!-- agent-frc:design-checks:start -->
## Design checks

What a coding agent runs for which change to a design. The agent's design tool wrote this block and keeps it current; write the project's own instructions outside it. The block is instruction only: the project runs none of it.

| The change touches | What the agent runs |
| --- | --- |
| Geometry, a motion, a tolerance or reserve, a fixture, a trial, a mass or voltage budget | The layout check, on the phases the change affects |
| The review page, a title, a colour, a name, a version label, a status word, a document | A view-only rebuild when the page shows it, then the publication check. Never the layout check |
| A draft being published | The publication check. A draft is published on its own layout check |
| A design being marked effective | The command that marks it effective. It runs the stage gate itself: the full layout check on the committed design, once. Commit the design's inputs first |
| The physics of an effective design | Do not edit it in place. Open the next minor version as a draft |

- The design tool belongs to the agent. The project never runs it: the project's CI is the project's own files, and a push only publishes.
- The project's CI scans for personal paths and secrets, checks that the published site is whole and publishes it. It runs no design check.
- Run the publication check before every push. It never runs the layout check.
- Time guidelines: about 1 minute while editing, 5 before merging, 15 at a stage gate.
- When a run is heading past its guideline, stop and tell the owner what is slow and why. Agree a budget before going on.
- Do not repeat a run whose inputs have not changed.
- Read what each command prints: it says what ran, what did not and what a push will do.
<!-- agent-frc:design-checks:end -->
