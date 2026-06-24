# `dev.cajeta.robotica.transform` — Specification

The Lie-group **foundation** of cajeta-robotica. Every other package depends on this; it
depends on nothing in the family (only on Cajeta's `numpy`/`linalg`/`tensor` substrate). It
fixes — once and forever — the rotation, twist, and frame conventions the whole family shares.
See the family spec [`../robotica-spec.md`](../robotica-spec.md) §2.a and the research
[`../../research/robotics-stack-part2-perception-to-ai.md`](../../research/robotics-stack-part2-perception-to-ai.md) §B.1.

---

## 1. Definition

`transform` provides exact, allocation-light **SO(3)/SE(3) Lie-group math**, rotation/quaternion
types, the closed-form **exp/log/adjoint/Jacobian** maps, a **time-stamped SE(3) frame tree**,
and the shared geometry/units/time types the rest of the family speaks in. It is pure
computation: fully unit-testable **without hardware** via algebraic identities.

### 1.a Capability summary
- 1.a.i — SO(3) rotations with conversions among quaternion / 3×3 matrix / axis-angle / RPY.
- 1.a.ii — SE(3) rigid transforms: compose, invert, act on points/vectors/poses.
- 1.a.iii — Lie operations: `hat`/`vee`, `exp`/`log`, `Adjoint`, left/right Jacobians, geodesic
  interpolation (slerp/screw).
- 1.a.iv — A time-stamped, interpolating **frame tree** (tf2-style buffered lookup).
- 1.a.v — Core types: timestamps/durations, units, twist (se(3) tangent) and wrench (its dual),
  spatial vectors.

### 1.b Conventions (NORMATIVE — fixed here, used family-wide)
> These are the single most important decisions in the family; mixing conventions silently
> corrupts every downstream Jacobian (the Sophus-vs-manif hazard, research §B.1). They are
> **decided here and never overridden** by a dependent package.
- 1.b.i — **Quaternions: Hamilton convention**, right-handed, **scalar-last storage**
  `[x, y, z, w]` (Eigen/ROS-compatible). Always normalized for rotations; the canonical sign
  has `w ≥ 0`.
- 1.b.ii — **Twist / se(3) tangent ordering: translation-first** `ξ = [ρ (3); θ (3)]` (ρ =
  linear, θ = angular). Sophus-compatible. The wrench (dual) matches: `[force; torque]`.
- 1.b.iii — **Transforms are active** (they move points within a fixed frame) and compose
  **left-to-right as parent←child**: `T_a_c = T_a_b * T_b_c`.
- 1.b.iv — **Angles in radians, lengths in meters, time in SI seconds** (see §2.e units).
- 1.b.v — Right-handed coordinate frames throughout.

### 1.c Non-goals
- 1.c.i — No sensor/hardware I/O (that is `io.*`), no estimation/SLAM (that is `estimation`/
  `fusion`), no dynamics (that is `dynamics`). `transform` is geometry only.
- 1.c.ii — No dynamic memory in the hot path beyond what the frame-tree buffers require.

---

## 2. Features

### 2.a SO(3) — rotations
Definition: a rotation type backed by a unit quaternion (§1.b.i) with lazy/explicit matrix form.
- 2.a.i (use case) — Construct from quaternion, 3×3 matrix, axis-angle, RPY (intrinsic
  Z-Y-X), or two vectors (shortest-arc), and convert losslessly back.
- 2.a.ii — Compose (`R1 * R2`) and invert (`Rᵀ`); rotate a 3-vector (`R * v`).
- 2.a.iii — Normalize/renormalize a drifted quaternion; canonicalize sign (`w ≥ 0`).
- 2.a.iv — Geodesic interpolation (slerp) between two rotations, `t ∈ [0,1]`.
- 2.a.v — Angular distance between two rotations (radians).

### 2.b SE(3) — rigid transforms
Definition: a pose = (SO(3) rotation, 3-vector translation), §1.b.iii composition.
- 2.b.i — Construct from (rotation, translation), 4×4 homogeneous matrix, or a twist via `exp`.
- 2.b.ii — Compose (`T_a_b * T_b_c`), invert (`T⁻¹`), act on a point, a free vector, and a pose.
- 2.b.iii — Extract rotation, translation, and the 4×4 / 3×4 matrix forms.
- 2.b.iv — Geodesic (screw) interpolation between two poses.
- 2.b.v — Relative transform `T_a_b⁻¹ * T_a_c` (pose of c in b's frame).

### 2.c Lie-group operations
Definition: the closed-form maps tying the group (SO(3)/SE(3)) to its algebra (so(3)/se(3)),
per micro-Lie-theory (arXiv:1812.01537).
- 2.c.i — `hat`/`vee` between a tangent vector and its matrix (skew for so(3); 4×4 for se(3)).
- 2.c.ii — `exp`: tangent → group (Rodrigues for SO(3); closed-form V-matrix for SE(3)),
  numerically stable near θ→0 (series fallback).
- 2.c.iii — `log`: group → tangent, the inverse of `exp`, branch-correct.
- 2.c.iv — `Adjoint(T)`: map a twist between frames; `adjoint(ξ)` (the algebra bracket).
- 2.c.v — Left/right Jacobians `J_l`, `J_r` and their inverses (for downstream estimation/
  dynamics derivatives).
- 2.c.vi — Box-plus / box-minus (`⊞`, `⊟`): retraction and its inverse on the manifold, the
  interface fusion/estimation filters consume.
- Acceptance: `exp(log(X)) == X` and `log(exp(ξ)) == ξ`, `Adjoint` and Jacobian identities, all
  to tolerance, as unit tests with no hardware.

### 2.d Frame tree (time-stamped transform graph)
Definition: a directed tree of frames whose edges are time-series of parent←child SE(3)
transforms, with temporal interpolation — the tf2 model (research §B.2).
- 2.d.i — Register/insert a timestamped transform on an edge (a per-edge ring buffer).
- 2.d.ii — Look up `T_target_source` at time *t*, **interpolating** (screw/slerp+lerp) between
  the two bracketing samples.
- 2.d.iii — Look up the latest available transform; query the valid time window of a frame.
- 2.d.iv — Walk a multi-edge path between arbitrary frames, composing along the way.
- 2.d.v — Configurable extrapolation policy (error vs. clamp vs. constant-velocity) and a
  buffer-horizon/eviction policy.
- 2.d.vi — Static (time-invariant) transforms as a distinct, cheap edge kind.
- Acceptance: round-trip and path-composition consistency; deterministic behavior at and
  outside buffer bounds.

### 2.e Core shared types
Definition: the value types the whole family passes around.
- 2.e.i — `Time` / `Duration` (nanosecond integer base, SI-second views), monotonic vs. wall.
- 2.e.ii — Units: typed `meters`, `radians`, `seconds` (and rates) to prevent unit-mix bugs at
  the API boundary; conversion helpers.
- 2.e.iii — `Twist` (se(3) tangent, §1.b.ii) and `Wrench` (its dual) as first-class types.
- 2.e.iv — Spatial 6-vectors / 6×6 spatial-inertia-compatible layout (the form `dynamics`
  consumes), consistent with the twist ordering.
- 2.e.v — Small fixed-size vec3/quat/mat3/mat4 aliases over Cajeta `tensor`/`linalg`.

---

## 3. Acceptance criteria (beyond per-feature tests)
- 3.a — **Convention conformance:** a test suite asserts §1.b conventions hold at every API
  boundary (quaternion storage order, twist order, compose direction).
- 3.b — **Hardware-free:** the entire suite runs in CI with no devices, on x86-64 and at least
  one RISC-V target (no non-portable deps).
- 3.c — **Numerical stability:** `exp`/`log`/Jacobians correct across the small-angle (θ→0) and
  near-π regimes (series-fallback paths exercised).
- 3.d — **No upward dependencies:** `transform` imports nothing else in `dev.cajeta.robotica.*`.
- 3.e — **Allocation discipline:** SO(3)/SE(3) ops allocate nothing on the hot path; only the
  frame-tree buffers allocate, bounded by the horizon policy.

## 4. Deliverables
- 4.a — `dev.cajeta.robotica.transform` implementing §2, with the §3 test suite.
- 4.b — A short conventions reference (§1.b) published as the family's canonical citation.
- 4.c — A plan at `agents/transform-plan.md` (authored when this spec is approved), TDD-ordered:
  core types → SO(3) → SE(3) → Lie ops → frame tree.

## 5. References (research-grounded)
- Micro-Lie theory (exp/log/Jacobians) — arXiv:1812.01537. **[verified]**
- Sophus SE(3) layout + translation-first twist; manif (opposite order — the hazard). **[verified]**
- ROS tf2 frame-tree / buffered-interpolated-lookup model. **[scope: secondary source]**
