# CLAUDE.md — cajeta-robotica

Project-level guidance for **cajeta-robotica**: a Cajeta-native robotics stack — a family
of focused libraries under the package namespace `dev.cajeta.robotica.*`, built over a small
shared foundation (SE(3)/SO(3) transforms + core types). This is **not** part of the Cajeta
language stdlib; it is an external library family. The workspace-level conventions still
apply; this file adds repo-specific structure and imports the spec → plan → develop workflow.

@td-project-workflow.md

## Layout
- **Specs** live at `docs/specs/<package-path>/<name>-spec.md`, with the directory tree
  **mirroring the `dev.cajeta.robotica.*` package hierarchy** — e.g. `docs/specs/io/sensor/`
  ↔ `dev.cajeta.robotica.io.sensor`, `docs/specs/motor/` ↔ `dev.cajeta.robotica.motor`. The
  top-level family spec is `docs/specs/robotica-spec.md`.
- **Plans + work stacks** live in `agents/` (this repo).
- **Background research** (the landscape that grounds the specs) lives in `docs/research/`.

## Design pillars (carry into every spec)
- **Family over a shared foundation.** One `transform` foundation (SE(3)/SO(3), frame tree,
  core types); focused libs depend on it, never upward. Aggressive link-time DCE means breadth
  costs nothing at deployment, so unify *types* while partitioning *dependencies*.
- **One language from matmul kernel to FOC setpoint.** Reuse Cajeta's native numpy/tensor/
  linalg/FFT/GPU stack; no Python↔C++↔firmware seam.
- **Safety as a first-class differentiator.** Cajeta ownership/borrow + deterministic drop for
  e-stop/watchdog/fault paths (provable release, no GC pause).
