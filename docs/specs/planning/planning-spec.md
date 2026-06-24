# `dev.cajeta.robotica.planning` — Specification

The **motion-planning & trajectory** math layer of cajeta-robotica. It turns a
configuration space + a goal into a collision-free path, retimes that path (or any
target state) into a jerk-limited time-optimal trajectory, and answers
distance/penetration queries between convex shapes. It is **math, not hardware**:
fully unit-testable without devices via identity, round-trip, and finite-difference
checks. It depends **only on** `transform` (and Cajeta's `numpy`/`linalg`/`tensor`
substrate) and **never upward** — planners consume user-supplied callbacks rather than
importing `kinematics`/`dynamics`. See the family spec
[`../robotica-spec.md`](../robotica-spec.md) §2.j and the research
[`../../research/robotics-stack-part2-perception-to-ai.md`](../../research/robotics-stack-part2-perception-to-ai.md)
§C.2 (OMPL RRT\*) [verified], §C.3 (Ruckig OTG) [verified], §C.4 (GJK/EPA) [scope].

---

## 1. Definition

`planning` provides three composable, allocation-disciplined capabilities over a generic
**configuration space (C-space)**:

1. **Sampling-based planners** — the RRT / RRT-Connect / RRT\* / PRM family, driven by
   a small set of C-space callbacks (sample / nearest / steer / interpolate / distance /
   state-validity), plus the tree-rewiring that makes RRT\* asymptotically optimal
   (research §C.2).
2. **Online trajectory generation (OTG)** — Ruckig-style, **time-optimal, jerk-limited**
   profiles that retarget every control tick, including the **non-zero target
   acceleration** case (research §C.3, arXiv:2105.04830).
3. **Convex collision** — **GJK** (distance / intersection) + **EPA** (penetration depth +
   contact normal) over convex support shapes, with a **broadphase** pair-culling stage
   (research §C.4).

It is pure computation: deterministic under a fixed RNG seed, board-agnostic (no native
deps), and verifiable on synthetic spaces and analytic shape pairs.

### 1.a Capability summary
- 1.a.i — A generic **C-space** interface (state vector + the planner callbacks) over which
  every planner is written, so a planner is independent of what the configuration *means*
  (joint angles, SE(3) pose, mobile-base pose, …).
- 1.a.ii — **RRT, RRT-Connect, RRT\*, PRM/PRM\*** planners returning a collision-free
  C-space path (or failure within a time/iteration budget).
- 1.a.iii — **Path simplification** (shortcutting + B-spline/linear smoothing) as a
  post-process on a returned path.
- 1.a.iv — **OTG**: from (current pos/vel/acc, target pos/vel/acc, kinematic limits) emit
  the next setpoint and/or the full closed-form profile; per-DoF **time-synchronized**.
- 1.a.v — **GJK/EPA** narrowphase over convex shapes via support functions, plus a
  **broadphase** (AABB sweep-and-prune / BVH) that returns candidate overlapping pairs.

### 1.b Conventions (INHERITED — not redefined here)
> This package **inherits, by reference, every normative convention fixed in**
> [`../transform/transform-spec.md`](../transform/transform-spec.md) **§1.b** and does not
> restate or override any of them. In particular: Hamilton scalar-last quaternions (§1.b.i),
> translation-first twist/wrench ordering (§1.b.ii), active left-to-right `parent←child`
> composition (§1.b.iii), and **SI units — radians / meters / seconds** (§1.b.iv) on every
> API boundary. Workspace collision geometry uses `transform`'s SE(3) poses and vec3 types.
- 1.b.i — A **C-space state** is an opaque vector the host defines; planning never assumes
  its dimension or metric — both arrive through the callbacks (§2.a). Where a state has
  rotational components, distance/interpolation are the host's responsibility (so SO(3)
  geodesics from `transform` plug in cleanly).
- 1.b.ii — OTG operates **per-DoF in the host's units** (rad or m), with time in SI seconds;
  velocity/acceleration/jerk limits are given in those derived units.

### 1.c Non-goals
- 1.c.i — **No optimization-based planning** (CHOMP/TrajOpt/MPC) and no QP solver — that is
  `control` (family §2.k). Planning here is sampling-based + analytic OTG only.
- 1.c.ii — **No dynamics/torque awareness.** Trajectories are kinematic (position-level,
  jerk-limited); torque-feasible retiming belongs to `dynamics`/`control`.
- 1.c.iii — **No non-convex collision meshes.** Narrowphase is convex-only; concave bodies
  must be supplied as a union of convex pieces (decomposition is the host's job).
- 1.c.iv — **No FK/IK, no URDF/collision-model loading, no estimation.** The collision-check
  and steer callbacks are provided by the host (typically wiring in `kinematics`), keeping
  the dependency arrow pointing only at `transform`.
- 1.c.v — No GPU path; the spine stays board-agnostic (family §3.c).

---

## 2. Features

### 2.a Configuration-space abstraction
Definition: the generic interface every planner is written against — a state type plus the
callback set OMPL formalizes (research §C.2: *sample / nearest-neighbor / steer /
collision-check*), extended with the metric and interpolation a planner needs.
- 2.a.i (use case) — Define a C-space by supplying: **dimension**, a **uniform/Gaussian
  sampler**, a **distance** metric, an **interpolate**(`a, b, t`) map, a **steer**(`from,
  toward, ε`) local extender, and a **state-validity** (collision-check) predicate.
- 2.a.ii — Wrap a **bounded box** C-space (per-DoF `[lo, hi]`, optionally wrapping at
  ±π for revolute joints) as a ready-made instance of §2.a.i.
- 2.a.iii — **Compound** C-spaces: concatenate sub-spaces (e.g. R³ × SO(3)) with a weighted
  product metric, so a planner can plan over an SE(3) pose using `transform`'s geodesics.
- 2.a.iv — A **motion-validity** check that samples the segment `a→b` at a resolution and
  rejects on the first invalid state (the discrete collision-checked local planner).
- 2.a.v — A **nearest-neighbor** structure (a kd-tree, falling back to linear for tiny sets)
  serving k-NN and radius queries over the chosen metric.
- Acceptance: on a synthetic space with an analytic obstacle, validity/motion-validity and
  the metric/interpolation round-trip (`interpolate(a,b,0)=a`, `=b` at 1) hold as unit tests.

### 2.b Sampling-based planners (RRT / RRT-Connect / RRT\* / PRM)
Definition: the canonical sampling family (research §C.2 [verified], OMPL `RRTstar`),
written over §2.a callbacks; RRT\* is the **asymptotically-optimal** variant via tree
rewiring.
- 2.b.i — **RRT**: grow a single tree (sample → nearest → steer → motion-check → add) until
  the goal region is reached or a time/iteration budget expires; return the path or failure.
- 2.b.ii — **RRT-Connect**: two trees (start/goal) grown toward each other with a greedy
  *connect* extension, for fast feasible queries.
- 2.b.iii — **RRT\***: maintain per-node cost-to-come; on each insert, **choose-parent** and
  **rewire** within a shrinking neighborhood of radius `r(n) = min(γ · (log n / n)^{1/d}, ε)`
  (γ a free constant, `d` the C-space dimension), so path cost converges toward the optimum
  as samples grow.
- 2.b.iv — **PRM / PRM\***: build a roadmap (sample valid nodes, connect within a radius via
  motion-checks), then graph-search (A\*/Dijkstra) between start/goal; reusable across
  multiple queries in a static space.
- 2.b.v — Pluggable **goal specification** (exact state, goal region/sampler, or goal-bias
  fraction) and a **stop condition** (first-solution vs. optimize-until-budget).
- 2.b.vi — Deterministic results under a fixed RNG seed; a returned path is a list of valid
  states with each consecutive segment motion-valid (§2.a.iv).
- Acceptance: probabilistic completeness on a synthetic maze (a path is found given enough
  samples); for RRT\*, solution cost is **non-increasing** with iterations and approaches a
  known analytic optimum to tolerance — all hardware-free unit tests on synthetic spaces.

### 2.c Path simplification & smoothing
Definition: the post-process OMPL's `PathSimplifier` performs on a raw (often jagged) tree
path before execution.
- 2.c.i — **Shortcutting**: repeatedly attempt to replace a sub-path between two states with
  a direct motion-valid segment, reducing length/cost.
- 2.c.ii — **Collision-aware smoothing**: fit a smoother (linear resampling or B-spline)
  through the waypoints, re-checking motion-validity and falling back on the original where a
  smoothed segment is invalid.
- 2.c.iii — **Resample** a path to a fixed waypoint spacing for handoff to OTG (§2.d).
- Acceptance: simplified paths remain motion-valid and have length ≤ the input path
  (shortcutting never increases cost); deterministic under a fixed seed.

### 2.d Online trajectory generation (jerk-limited, time-optimal)
Definition: the Ruckig OTG target (research §C.3 [verified], arXiv:2105.04830) — from a
current and target kinematic state plus limits, produce a **time-optimal, jerk-limited**
profile, retargetable **every control tick**, including a **non-zero target acceleration**.
- 2.d.i (use case) — Single-DoF: given `(p₀, v₀, a₀)`, `(p_t, v_t, a_t)`, and limits
  `(v_max, a_max, j_max)`, compute the closed-form **seven-segment** (constant-jerk /
  constant-accel / constant-vel) profile and its total duration `T`.
- 2.d.ii — **Non-zero target acceleration** `a_t ≠ 0` handled online (the capability the
  open-source "Community" Ruckig restricts — a genuine gap-filler, research §C.3).
- 2.d.iii — **Sample** the profile at time `τ ∈ [0, T]` → `(p, v, a)`; the per-tick API
  takes the current state + a (possibly moving) target and emits the **next setpoint** at the
  control period `Δt`.
- 2.d.iv — **Multi-DoF time synchronization**: compute each DoF's minimum time, take the
  slowest, and **stretch** the faster DoFs to that common `T` (phase-synchronized arrival).
- 2.d.v — **Limit/state validation**: reject infeasible inputs (e.g. a target velocity above
  `v_max`) with a typed error; guarantee the emitted profile **never exceeds** any limit.
- 2.d.vi — **Online retargeting**: feeding a new target mid-trajectory produces a
  continuous-up-to-jerk profile from the current `(p, v, a)` (no discontinuity in `p, v, a`).
- Acceptance: finite-difference of the sampled profile shows `|v|≤v_max`, `|a|≤a_max`,
  `|j|≤j_max` everywhere and the boundary state is reached to tolerance; the time `T` matches
  the analytic minimum for canonical rest-to-rest cases — all hardware-free.

### 2.e Convex collision — GJK distance/intersection + EPA penetration [scope]
> **[scope]** Research §C.4 is a *primary-source + scope note* (coal/FCL docs, secondary);
> §J lists "FCL/GJK-EPA specifics" as not independently verified. The algorithm *structure*
> below is well-documented; exact numerical tolerances/degenerate-simplex handling are to be
> validated against a reference during the plan.

Definition: the Minkowski-difference pair — **GJK** evolves a simplex toward the origin to
report distance/intersection; **EPA** expands the terminal simplex into a polytope to recover
penetration depth and contact normal — over shapes expressed only by a **support function**.
- 2.e.i (use case) — Define a convex shape by its **support**(`d`) → farthest point along
  `d`; ship supports for sphere, box, capsule, and convex point-hull, each transformable by an
  SE(3) pose from `transform`.
- 2.e.ii — **GJK distance**: minimum distance + witness points between two disjoint convex
  shapes; returns 0 / intersecting when the simplex encloses the origin.
- 2.e.iii — **GJK boolean intersection**: a cheaper yes/no early-out (no witness points).
- 2.e.iv — **EPA penetration**: for an intersecting pair, the **penetration depth** and
  **contact normal** (minimum translation vector), seeded from GJK's terminal simplex.
- 2.e.v — **Conservative-advancement / continuous** support is **out of scope** for the first
  cut (discrete queries only); flagged as a [scope] follow-on.
- Acceptance: GJK distance and EPA depth match the **analytic** answer for sphere-sphere,
  sphere-box, and box-box configurations across separated, touching, and penetrating cases,
  to tolerance — hardware-free unit tests.

### 2.f Broadphase & collision world
Definition: the pair-culling stage that precedes narrowphase (research §C.4: "plus
broadphase") — turn N shapes into the small set of candidate overlapping pairs.
- 2.f.i — Maintain a set of shapes each with a world **AABB** (from its support + pose).
- 2.f.ii — **Sweep-and-prune** (sorted-projection) and/or a **BVH/AABB-tree** producing the
  candidate-pair set whose AABBs overlap.
- 2.f.iii — Feed candidate pairs to §2.e narrowphase; a **collision-world** convenience query
  reports all penetrating pairs (or the nearest distance) in one call.
- 2.f.iv — Provide the host a **state-validity adaptor**: wrap a collision-world as a §2.a.i
  validity predicate so a robot model collision-checks a planner C-space directly.
- Acceptance: broadphase output is a **superset** of the truly-colliding pairs (no false
  negatives) on randomized scenes; the collision-world result equals the brute-force
  all-pairs narrowphase result.

---

## 3. Acceptance criteria (beyond per-feature tests)
- 3.a — **Convention conformance:** all geometry uses `transform` types and §1.b inherited
  conventions (units, quaternion/twist order); a test asserts no local redefinition.
- 3.b — **Hardware-free & deterministic:** the entire suite runs in CI with no devices, on
  x86-64 and at least one RISC-V target; every randomized planner/scene test is seeded and
  reproducible (family §3.b).
- 3.c — **OTG limit-safety:** a finite-difference sweep over generated trajectories proves no
  velocity/accel/jerk limit is ever exceeded, including the non-zero-target-accel path (§2.d.ii).
- 3.d — **Planner optimality trend:** RRT\* cost is monotonically non-increasing in iterations
  and converges toward the analytic optimum on a benchmark space (§2.b.iii).
- 3.e — **Collision correctness:** GJK/EPA match analytic shape-pair answers; broadphase has
  zero false negatives (§2.e, §2.f).
- 3.f — **No upward dependencies:** `planning` imports only `transform` (+ Cajeta substrate);
  planners reach `kinematics`/collision models exclusively through host-supplied callbacks
  (family §1.b.ii, §3.c).
- 3.g — **Allocation discipline:** OTG sampling and GJK/EPA queries allocate nothing on the
  hot path; only planner tree/roadmap growth allocates, bounded by the iteration budget.

## 4. Deliverables
- 4.a — `dev.cajeta.robotica.planning` implementing §2, with the §3 test suite.
- 4.b — Synthetic planning benchmarks (maze / narrow-passage spaces) and analytic
  shape-pair fixtures used as the regression corpus.
- 4.c — A plan at `agents/planning-plan.md` (authored when this spec is approved),
  TDD-ordered: C-space abstraction (§2.a) → convex collision GJK/EPA + broadphase
  (§2.e–§2.f) → sampling planners + simplification (§2.b–§2.c) → online trajectory
  generation (§2.d). Collision lands before the planners so the planner tests can use a real
  validity predicate; OTG is independent and may proceed in parallel.

## 5. References (research-grounded)
- **OMPL `RRT*`** — asymptotically-optimal sampling planner; C-space callback set
  (sample / nearest / steer / collision-check) + tree-rewiring. Research §C.2. **[verified]**
  (ompl.kavrakilab.org RRT\* class docs; optimal-planning literature.)
- **Ruckig OTG** — time-optimal, jerk-limited online trajectory generation, first to handle
  **non-zero target acceleration** online; Community-edition feature limits make a native
  implementation a gap-filler. Research §C.3, arXiv:2105.04830. **[verified]**
- **FCL / coal GJK + EPA** — convex distance/intersection (GJK) and penetration depth (EPA)
  via support functions + simplex evolution, plus broadphase. Research §C.4. **[scope]**
  (coal GJK/EPA docs — secondary; specifics flagged in research §J, validate during the plan.)
- **Foundation:** `transform` SE(3)/SO(3) types & conventions —
  [`../transform/transform-spec.md`](../transform/transform-spec.md) §1.b. **[verified]**
