# `dev.cajeta.robotica.dynamics` — Specification

The rigid-body **dynamics** layer of cajeta-robotica. A pure-math (`[layer: math]`) package
that turns a multibody model + joint state into joint torques, accelerations, the joint-space
inertia matrix, and the **closed-form analytical derivatives** of all three — the gradients
gradient-based control and trajectory optimization (MPC) consume. It depends on `transform`
for the SE(3)/spatial-vector foundation and on nothing in the family that sits above it. It is
fully unit-testable **without hardware** via algebraic identities and finite-difference checks.
See the family spec [`../robotica-spec.md`](../robotica-spec.md) §2.i and the research
[`../../research/robotics-stack-part2-perception-to-ai.md`](../../research/robotics-stack-part2-perception-to-ai.md) §C.1.

---

## 1. Definition

`dynamics` implements the classical **spatial-algebra** trio of rigid-body dynamics over an
articulated multibody model:

- **RNEA** — the Recursive Newton-Euler Algorithm: **inverse dynamics**, joint torques `τ` for
  a desired motion `(q, q̇, q̈)`, in `O(n)`.
- **ABA** — the Articulated-Body Algorithm: **forward dynamics**, joint acceleration `q̈` from
  applied torques `τ`, in `O(n)`.
- **CRBA** — the Composite-Rigid-Body Algorithm: the **joint-space inertia matrix** `M(q)`, in
  `O(n²)`, plus its Cholesky factor for solves.

Its distinguishing surface — the reason this package exists beyond a single torque call — is the
set of **closed-form ANALYTICAL derivatives** of RNEA and ABA (Carpentier/Mansard, RSS 2018):
`∂τ/∂q`, `∂τ/∂q̇`, `∂q̈/∂q`, `∂q̈/∂q̇`, computed recursively at a fraction of the cost of finite
differences, and **finite-difference-verifiable** in a unit test with no hardware (research §C.1).
The spatial-vector layout — motion vectors, force vectors, 6×6 spatial inertias — matches
`transform`'s twist/wrench ordering exactly (§1.b), so a wrench out of `dynamics` is the same
6-vector a `transform` adjoint moves between frames.

### 1.a Capability summary
- 1.a.i — Spatial algebra: motion vectors (twists), force vectors (wrenches), 6×6 spatial
  inertias, spatial (Plücker) transforms and their adjoints, and the spatial cross products
  (`v ×` on motion, `v ×*` on force) — laid out to `transform` conventions (§1.b).
- 1.a.ii — A **multibody model**: an ordered kinematic tree of joints/links with per-link
  spatial inertia, joint placements, and joint types (fixed, revolute, prismatic, spherical,
  free-flyer), with the `nq`/`nv` configuration/tangent split (§1.b.iv).
- 1.a.iii — **Inverse dynamics** (RNEA) — `τ = M(q)q̈ + C(q,q̇)q̇ + g(q)` — with the gravity
  term, the nonlinear (Coriolis/centrifugal + gravity) bias, and external wrenches as special
  cases.
- 1.a.iv — **Forward dynamics** (ABA), and the alternative `M⁻¹` route via CRBA + Cholesky.
- 1.a.v — **Joint-space inertia** (CRBA) `M(q)`, symmetric positive-definite, with its Cholesky.
- 1.a.vi — **Analytical derivatives** of RNEA and ABA w.r.t. `q`, `q̇`, `q̈`/`τ` (RSS 2018).
- 1.a.vii — Derived quantities for control: center of mass + CoM Jacobian, kinetic/potential
  energy, total/centroidal momentum and the centroidal momentum matrix `A_g` ([scope], §2.g).

### 1.b Conventions (INHERITED — not redefined here)
> `dynamics` adds **no new geometric conventions**. It **inherits `transform`'s normative
> conventions** ([`../transform/transform-spec.md`](../transform/transform-spec.md) §1.b) by
> reference and uses them unchanged. They are decided in `transform` and never overridden here.
- 1.b.i — **Twist / spatial-motion ordering = `transform` §1.b.ii: translation-first**
  `ξ = [ρ (3); θ (3)]` (ρ linear, θ angular). A spatial **motion vector** (link velocity,
  acceleration) is exactly this twist; a spatial **force vector** is the dual **wrench**
  `[force; torque]`. A 6×6 **spatial inertia** is block-laid-out to match this dual pairing
  (`force = I · motion`).
- 1.b.ii — **Quaternions, SE(3) composition, active transforms, right-handed frames** = `transform`
  §1.b.i / §1.b.iii / §1.b.v, used for joint placements and link poses (spherical and free-flyer
  joints store a Hamilton scalar-last quaternion).
- 1.b.iii — **Units = `transform` §1.b.iv:** radians, meters, SI seconds; hence N·m torques,
  kg·m² inertias. Gravity defaults to `g = 9.80665 m/s²` applied as a spatial acceleration at
  the model root along base `−z` (magnitude/direction configurable on the model).
- 1.b.iv — **`nq` vs `nv` (the configuration/tangent split):** a configuration `q ∈ ℝ^nq` may be
  larger than its velocity `q̇ ∈ ℝ^nv` because quaternion-bearing joints store 4 numbers but move
  on a 3-D tangent. Integration of `q̇` into `q` uses `transform`'s manifold retraction
  (box-plus, `transform` §2.c.vi) per joint; derivatives w.r.t. `q` are taken in the `nv`-D
  tangent, never in raw `nq` coordinates.
- 1.b.v — **Reference frame for spatial outputs:** Jacobians, derivatives, and reported spatial
  quantities are expressed in a named frame — `LOCAL` (body frame), `WORLD`, or
  `LOCAL_WORLD_ALIGNED` (origin at the body, axes of world) — converted between by `transform`'s
  adjoint. `LOCAL` is the default; the choice is an explicit argument, never implicit.

### 1.c Non-goals
- 1.c.i — **No model front-end.** URDF/SRDF/MJCF/USD parsing is `kinematics`/`sim`. `dynamics`
  consumes a programmatically-built or sibling-provided model; it does not read files. *(Boundary
  with `kinematics`: [scope], §2.b.)*
- 1.c.ii — **No IK / no planning / no control law.** Forward/inverse *kinematics*, Jacobian-based
  IK, motion planning, and the controllers/optimizers that *use* these derivatives live in
  `kinematics`, `planning`, and `control`. `dynamics` supplies the model and gradients they call.
- 1.c.iii — **No contact/constraint solver in the core.** Frictional contact, LCP/complementarity,
  and collision are out of the core; **constrained forward dynamics via the KKT system** (closed
  loops, fixed contacts) is an explicit [scope] extension (§2.h), not v1-core.
- 1.c.iv — **No hardware I/O, no actuator electrical model.** Joint-level effects only; rotor
  inertia / armature (a diagonal addition to `M`) and viscous/Coulomb joint friction are the only
  drivetrain terms modeled, and those are optional ([scope] for friction).
- 1.c.v — **No GPU/batched kernels in v1.** A batched, GPU-resident dynamics path over Cajeta's
  tensor stack is a deliberate forward target ([scope], §2.i), kept out of the hardware-free core.

---

## 2. Features

### 2.a Spatial algebra primitives
Definition: the 6-D motion/force vector calculus all three algorithms are built from, in
`transform`'s twist/wrench layout (§1.b.i). Grounded in Pinocchio's spatial types (research §C.1);
the algebra itself is Featherstone spatial-vector algebra ([scope] citation, §5).
- 2.a.i (use case) — Construct a **spatial inertia** from mass, center-of-mass offset, and a 3×3
  rotational inertia (about the CoM); read it back; assemble it as a symmetric 6×6 in the §1.b.i
  block layout.
- 2.a.ii — Apply a spatial inertia to a motion vector to get a force vector (`f = I · v`), and
  compute spatial momentum (`h = I · v`).
- 2.a.iii — Spatial cross products: `v ×` acting on a motion vector and the dual `v ×*` acting on
  a force vector (the bilinear terms that produce Coriolis/centrifugal forces).
- 2.a.iv — Transform a motion vector, a force vector, and a spatial inertia across a joint/link
  placement using `transform`'s SE(3) **adjoint** (§1.b.v) — no convention is redefined, the
  adjoint is imported.
- Acceptance: dual-pairing identity `vᵀ f` invariant under a shared adjoint; `I` symmetric and,
  for physical parameters, positive-definite — unit tests, no hardware.

### 2.b Multibody model & forward kinematics pass
Definition: the read-only **model** (topology + inertial/joint parameters) and a per-call
**workspace** holding the configuration-dependent quantities the algorithms cache (the
Pinocchio `Model`/`Data` split, research §C.1).
- 2.b.i — Build a model: append joints with a parent index, a joint type (fixed / revolute /
  prismatic / spherical / free-flyer), a placement transform, and the child link's spatial
  inertia; finalize to fix `nq`/`nv` and the topological joint order.
- 2.b.ii — Query model dimensions and per-joint `nq`/`nv` offsets; map a flat `q`/`q̇`/`q̈`/`τ`
  vector to/from per-joint blocks.
- 2.b.iii — Run the **forward-kinematics pass**: place every link (`oMi`) and compute each link's
  spatial velocity/acceleration from `(q, q̇, q̈)` — the shared first pass RNEA/ABA build on.
- 2.b.iv — Integrate `q̇` into `q` over `Δt` and difference two configurations
  (`q₂ ⊟ q₁ → q̇`) using `transform`'s per-joint retraction (§1.b.iv) — so quaternion joints
  stay on the manifold.
- 2.b.v — Optional per-joint **armature** (rotor inertia, a diagonal add to `M`) carried on the
  model; optional joint friction parameters ([scope], §1.c.iv).
- Acceptance: round-trip `q ⊞ (q₂ ⊟ q₁) == q₂` per joint; FK pass reproduces `transform`
  frame-tree poses for the same chain.

### 2.c Inverse dynamics — RNEA
Definition: joint torques for a prescribed motion, `τ = M(q)q̈ + C(q,q̇)q̇ + g(q)`, by the
two-pass recursive Newton-Euler algorithm (forward velocity/acceleration sweep, backward force
sweep). **[verified]** core (research §C.1).
- 2.c.i — Compute `τ` from `(q, q̇, q̈)` for a fixed-base or floating-base model.
- 2.c.ii — **Gravity term** `g(q) = RNEA(q, 0, 0)` (the static holding torque).
- 2.c.iii — **Nonlinear bias** `b(q,q̇) = C(q,q̇)q̇ + g(q) = RNEA(q, q̇, 0)` (gravity + Coriolis
  + centrifugal).
- 2.c.iv — Apply **external wrenches** at named links (e.g. an end-effector contact force) and
  fold them into `τ`.
- Acceptance: `RNEA(q, 0, a) − RNEA(q, 0, 0) == M(q) · a` (consistency with §2.e CRBA) to
  tolerance; linearity in `q̈`.

### 2.d Forward dynamics — ABA
Definition: the joint acceleration produced by applied torques, `q̈ = M(q)⁻¹(τ − b(q,q̇))`, by
the `O(n)` Articulated-Body Algorithm, with the dense `M⁻¹` route as the alternative.
**[verified]** core (research §C.1).
- 2.d.i — Compute `q̈` from `(q, q̇, τ)` via ABA, fixed- or floating-base, with external wrenches.
- 2.d.ii — Compute `q̈` via the **CRBA + Cholesky** route (factor `M` from §2.e, solve against
  `τ − b`) — the cross-check for ABA and the path that reuses a cached factor.
- 2.d.iii — Recover the explicit **`M(q)⁻¹`** (the minv operator) when the full inverse is needed
  (e.g. operational-space inertia downstream).
- Acceptance: **RNEA/ABA inverse identity** — `ABA(q, v, RNEA(q, v, a)) == a` — and ABA vs
  CRBA-route agreement, to tolerance, no hardware.

### 2.e Joint-space inertia — CRBA
Definition: the symmetric positive-definite joint-space inertia matrix `M(q)` by the
Composite-Rigid-Body Algorithm (`O(n²)`), plus its factorization. **[verified]** core (research §C.1).
- 2.e.i — Compute `M(q)` (upper triangle filled by the composite-inertia recursion, then
  symmetrized).
- 2.e.ii — Factor `M(q)` (Cholesky `L Lᵀ` on Cajeta `linalg`) and solve `M x = b` / apply `M⁻¹`
  without re-forming the matrix.
- 2.e.iii — Expose `M` (with optional armature, §2.b.v, added on the diagonal) to `control` (WBC)
  and `planning`.
- Acceptance: `M` symmetric to tolerance and positive-definite (Cholesky succeeds) for physical
  parameters; matches the §2.c.iv consistency relation.

### 2.f Analytical derivatives
Definition: the closed-form recursive derivatives of RNEA and ABA — the package's headline
deliverable for gradient-based control/optimization (Carpentier/Mansard, RSS 2018). **[verified]**
(research §C.1: "closed-form ANALYTICAL derivatives … finite-difference-verifiable without
hardware").
- 2.f.i — **Inverse-dynamics derivatives:** `∂τ/∂q` and `∂τ/∂q̇` (the `∂τ/∂q̈` term is `M(q)`),
  computed in one recursive sweep alongside RNEA, expressed in the §1.b.v reference frame.
- 2.f.ii — **Forward-dynamics derivatives:** `∂q̈/∂q`, `∂q̈/∂q̇`, and `∂q̈/∂τ = M(q)⁻¹`, derived
  from the inverse-dynamics derivatives + the CRBA factor.
- 2.f.iii — Derivatives taken in the **`nv`-D tangent** (§1.b.iv), so configuration perturbations
  respect the manifold (no raw-quaternion finite stepping).
- 2.f.iv — A reference **finite-difference** implementation (central differences via box-plus,
  §2.b.iv) shipped for testing/fallback, against which the analytical path is verified.
- Acceptance: analytical derivatives match central finite differences to `O(h²)` tolerance across
  randomized `(q, q̇, q̈/τ)` and randomized models — the core hardware-free acceptance test (§3.b).

### 2.g Derived quantities (energy, momentum, centroidal) — [scope]
Definition: the scalar/vector functionals control and estimation reuse; structurally part of
Pinocchio but **beyond the research's verified RNEA/ABA/CRBA+derivatives core** — flagged
[scope], to be confirmed against Pinocchio before its plan.
- 2.g.i — **Center of mass** of the whole model and the **CoM Jacobian** `J_com` (so `ẋ_com = J_com q̇`).
- 2.g.ii — **Kinetic energy** `½ q̇ᵀ M(q) q̇` and **potential energy** (gravity) — for energy/
  passivity checks (§3.c).
- 2.g.iii — Total spatial **momentum**, the **centroidal momentum matrix** `A_g` (CCRBA) and its
  time-variation bias — the centroidal-dynamics interface a WBC/MPC consumes.
- Acceptance: `½ q̇ᵀ M q̇` from §2.e matches the kinetic energy from this path; under no external
  force and exact integration, total energy is conserved (§3.c).

### 2.h Constrained forward dynamics (KKT) — [scope]
Definition: forward dynamics subject to bilateral constraints (closed kinematic loops, fixed
contacts) by solving the KKT/saddle-point system `[M Jᵀ; J 0][q̈; −λ] = [τ−b; −γ]`. Beyond the
verified core; included as a labeled [scope] extension, not v1.
- 2.h.i — Solve constrained `q̈` and the constraint forces `λ` for a set of contact/loop Jacobians.
- 2.h.ii — Proximal/regularized variant for redundant or near-singular constraint sets.
- 2.h.iii — Its analytical derivatives (extends §2.f through the KKT factor) — deferred behind
  §2.f.

### 2.i Batched / GPU dynamics — [scope]
Definition: a vectorized path evaluating RNEA/ABA/CRBA + derivatives across a batch of
`(q, q̇, …)` on Cajeta's tensor/GPU substrate (the sampling-based-MPC / RL-rollout use). A
deliberate forward target kept out of the hardware-free math core; structure to be re-researched.
- 2.i.i — Batched RNEA/ABA over `B` states sharing one model, results as a tensor.
- 2.i.ii — Batched analytical derivatives for parallel trajectory-optimization rollouts.
- *(Per the family's board-agnostic pillar, any GPU dependency stays optional and additive; the
  CPU core in §2.a–2.f remains native-dep-free.)*

---

## 3. Acceptance criteria (beyond per-feature tests)
- 3.a — **Convention conformance:** spatial motion = `transform` twist `[ρ; θ]`, spatial force =
  wrench `[force; torque]`, and a `dynamics` wrench is moved between frames by `transform`'s
  adjoint unchanged — asserted at the API boundary (§1.b.i, §1.b.v). No convention is redefined.
- 3.b — **Hardware-free verification (the headline test):** analytical derivatives (§2.f) match
  central finite differences to `O(h²)` across randomized models and states; runs in CI with no
  devices (research §C.1). The algorithm identities — `ABA∘RNEA == id` (§2.d), `RNEA(q,0,a)−g ==
  M·a` (§2.c/§2.e), `M = Mᵀ ≻ 0` (§2.e) — run alongside.
- 3.c — **Physical consistency:** under no external force and exact integration, total mechanical
  energy is conserved and the system is passive (the §2.g energy/momentum checks); gravity torque
  `g(q)` matches the static `RNEA(q,0,0)`.
- 3.d — **Numerical robustness:** RNEA/ABA/CRBA stable for ill-conditioned `M` (high mass ratios,
  near-singular configurations); Cholesky-failure surfaced explicitly, not silently NaN-propagated.
- 3.e — **No upward dependencies & board-agnostic:** `dynamics` imports only `transform` (plus
  Cajeta `numpy`/`linalg`/`tensor`); the §2.a–2.f core has **no native/heavy deps** and runs on
  x86-64 and at least one RISC-V target. Any GPU/batched path (§2.i) is additive and optional.
- 3.f — **Allocation discipline:** a steady-state RNEA/ABA/CRBA call reuses the model workspace
  (§2.b) and allocates nothing on the hot path; the model is built once, evaluated many times.

## 4. Deliverables
- 4.a — `dev.cajeta.robotica.dynamics` implementing §2.a–2.f (the verified core), with the §3
  test suite; §2.g–2.i delivered as labeled [scope] follow-ons after re-research.
- 4.b — A reference finite-difference derivative path (§2.f.iv) shipped as the test oracle and a
  documented fallback.
- 4.c — A plan at `agents/dynamics-plan.md` (authored when this spec is approved), TDD-ordered:
  spatial algebra (§2.a) → model + FK pass (§2.b) → RNEA (§2.c) → CRBA (§2.e) → ABA (§2.d) →
  analytical derivatives (§2.f), each unit of work landing its identity/finite-difference tests
  before its implementation. [scope] features (§2.g–2.i) are planned only after their re-research.

## 5. References (research-grounded)
- **Pinocchio** — RNEA (inverse), ABA (forward), CRBA (joint-space inertia) + the distinguishing
  **closed-form analytical derivatives**, finite-difference-verifiable without hardware.
  Research Part 2 §C.1; sources: Pinocchio README, https://github.com/stack-of-tasks/pinocchio. **[verified]**
- **Analytical derivatives of rigid-body dynamics** — Carpentier & Mansard, RSS 2018,
  https://laas.hal.science/hal-01866228/document (research §C.1, sources list). **[verified]**
- **`transform` conventions** (twist/wrench ordering, quaternion, units, adjoint, box-plus) —
  inherited by reference: [`../transform/transform-spec.md`](../transform/transform-spec.md) §1.b,
  §2.c. **[verified]** (the family's pinned convention, robotica-spec §3.a).
- **Featherstone spatial-vector algebra** — the textbook basis for the §2.a primitives and the
  RNEA/ABA/CRBA recursions; not independently cited in the research pass. **[scope]**
- **Centroidal dynamics / CCRBA (`A_g`), constrained-KKT dynamics, batched/GPU evaluation** —
  present in Pinocchio but beyond the research's verified RNEA/ABA/CRBA+derivatives core; §2.g–2.i.
  Re-research against Pinocchio before their plans. **[scope]**
- **KDL/orocos, Drake** — alternative dynamics engines; URDF/SRDF model front-end lives in
  `kinematics` (research §C.1 scope note). **[scope]**
