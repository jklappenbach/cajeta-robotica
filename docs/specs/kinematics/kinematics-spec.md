# `dev.cajeta.robotica.kinematics` — Specification

The **kinematics** package of cajeta-robotica (layer: **math**). It turns a robot's joint
configuration into the geometry the rest of the spine reasons about — link poses, end-effector
poses, and the velocity maps (Jacobians) between joint space and task space — and it parses the
standard **URDF/SRDF** robot-description front end into a chain expressed on the `transform`
frame model. It is **pure computation on top of `transform`**: fully unit-testable **without
hardware** via algebraic identities, finite-difference Jacobian checks, and recorded
model-file fixtures. See the family spec [`../robotica-spec.md`](../robotica-spec.md) §2.h and
the research [`../../research/robotics-stack-part2-perception-to-ai.md`](../../research/robotics-stack-part2-perception-to-ai.md)
§C.1 (KDL/Pinocchio kinematics; URDF) and §E.3 (robot-description formats).

---

## 1. Definition

`kinematics` provides a **kinematic model** (a tree of links connected by typed joints),
closed-form **forward kinematics** (configuration → link/end-effector poses), the **geometric
and analytic Jacobians** of that map, **inverse kinematics** (task pose → configuration), and a
**URDF/SRDF parser** that builds the model and lands its link frames directly onto the
`transform` time-stamped frame tree. Every rigid-body pose, twist, exp/log, and Adjoint it needs
comes from `transform`; this package adds the *articulated*-body layer on top, and nothing more.

### 1.a Capability summary
- 1.a.i — A **kinematic model**: links + typed joints (fixed, revolute, continuous, prismatic,
  planar, floating) forming a single-rooted tree, with a generalized-coordinate layout.
- 1.a.ii — **Forward kinematics**: place every link frame, compute any end-effector pose, and
  emit the result as a `transform` frame-tree snapshot.
- 1.a.iii — **Differential kinematics**: the geometric (spatial/body) Jacobian, the analytic
  Jacobian for a chosen orientation parameterization, and singularity/manipulability measures.
- 1.a.iv — **Inverse kinematics**: numerical (damped-least-squares) and, for specific
  structures, closed-form analytic solvers, with joint limits and nullspace objectives.
- 1.a.v — **Model front end**: URDF (geometry/topology) and SRDF (semantic groups, end-effectors,
  virtual joints, named states) parsed into the §1.a.i model.

### 1.b Conventions (INHERITED — not redefined here)
> This package **inherits `transform` §1.b wholesale and overrides nothing**: Hamilton
> quaternions, scalar-last `[x,y,z,w]`; **translation-first** twist `ξ = [ρ; θ]` (and the
> matching `[force; torque]` wrench); **active** transforms composing left-to-right as
> `T_a_c = T_a_b · T_b_c`; radians / meters / SI seconds; right-handed frames. Do **not**
> reread or restate the quaternion/twist/unit definitions — cite `transform` §1.b. The items
> below are only the *additional* articulated-body conventions this package pins.
- 1.b.i — **Generalized coordinates.** A configuration is a vector `q ∈ ℝ^nq`; a velocity is
  `v ∈ ℝ^nv`. Per-joint DoF: revolute/prismatic `nq=nv=1`, continuous `nq=1`, planar `nq=nv=3`,
  fixed `0`, **floating `nq=7` (translation + unit quaternion) but `nv=6`** (a `transform`
  twist). `nq ≠ nv` in general (the Pinocchio split, research §C.1). Coordinate **index order
  is assigned by a deterministic depth-first traversal** of the joint tree in child-declaration
  order; the model exposes name↔index lookup so callers never hard-code offsets.
- 1.b.ii — **Configuration integration is on the manifold.** `q ⊕ v` uses `transform` box-plus
  (§transform 2.c.vi) on the SO(2)/SE(3)-valued coordinates — never naive vector addition across
  the quaternion or continuous-joint components.
- 1.b.iii — **Joint geometry is URDF-style, not Denavit–Hartenberg.** Each joint carries a fixed
  `origin` transform (parent-link frame → joint frame at `q=0`) and a unit `axis` expressed in
  the joint frame; the joint variable applies a motion *about/along* that axis (revolute:
  rotation; prismatic: translation), realized via `transform` `exp`. (A DH-parameter ingest is a
  [scope] convenience, §2.a.)
- 1.b.iv — **Jacobian layout.** Columns are ordered to match `v` (§1.b.i); each column is a
  `transform` twist, **translation-first `[linear; angular]`** (§transform 1.b.ii). The frame in
  which a Jacobian is expressed (base/spatial vs. body/end-effector) is an explicit parameter,
  never implicit.

### 1.c Non-goals
- 1.c.i — **No dynamics.** Mass, inertia, forces, torques, RNEA/ABA/CRBA belong to `dynamics`.
  URDF `<inertial>` is parsed structurally and **passed through untyped**, not interpreted here.
- 1.c.ii — **No collision or planning.** Collision geometry (URDF `<collision>`) and SRDF
  `<disable_collisions>` pairs are **carried as opaque data** for `planning`; this package
  computes no distances and enforces no collision constraints.
- 1.c.iii — **No trajectory generation / time parameterization** (that is `planning`/`control`),
  and **no actuation or hardware I/O** (that is `motor`/`io`).
- 1.c.iv — **No Lie-group reimplementation.** All SE(3)/SO(3)/quaternion math is delegated to
  `transform`; this package never defines its own exp/log/Adjoint.
- 1.c.v — **No Xacro macro expansion and no URDF/SRDF *writing*** in this milestone (read-only
  front end; pre-expand Xacro with the upstream tool). See §2.e scope note.

---

## 2. Features

### 2.a Kinematic model & chain
Definition: the articulated-body data structure — a single-rooted tree of links joined by typed
joints, with a generalized-coordinate layout (§1.b.i) and subchain selection. Models KDL's
`Tree`/`Chain` and Pinocchio's `Model` (research §C.1).
- 2.a.i (use case) — Construct a model from links and joints: each joint typed (fixed, revolute,
  continuous, prismatic, planar, floating), with parent/child links, a fixed `origin` transform,
  a unit `axis`, and limits (position lower/upper, velocity, effort).
- 2.a.ii — Build the generalized-coordinate layout: assign each movable joint its `(nq, nv)`
  slice of `q` and `v` per §1.b.i, with bidirectional name↔index lookup and total `nq`/`nv`.
- 2.a.iii — Select a **subchain** between two links (e.g. base→tool) and name tip frames; expose
  the ordered joint path and the fixed/movable split along it.
- 2.a.iv — Produce the **neutral (zero) configuration**, and **clamp** an arbitrary `q` to joint
  limits (continuous joints wrap; floating/planar handled per their coordinate kind).
- 2.a.v — **Integrate** a configuration on the manifold: `q' = q ⊕ v·dt` using §1.b.ii box-plus;
  and its inverse, the **difference** `v = q_b ⊟ q_a` (the velocity that carries `q_a` to `q_b`).
- 2.a.vi — [scope] Ingest a **Denavit–Hartenberg** parameter table as an alternative joint-origin
  source, converting `(a, α, d, θ)` rows into §1.b.iii origins+axes. (Research §C.1 names DH only
  indirectly; provided as a convenience, not the primary path.)
- Acceptance: tree invariants (single root, acyclic, every non-root link has exactly one parent
  joint) are checked at construction; `nq`/`nv` totals match the joint inventory; `⊕`/`⊟`
  round-trip to tolerance.

### 2.b Forward kinematics (FK)
Definition: the map `q → {link poses}`, one tree sweep composing each joint's fixed origin with
its variable motion via `transform` `exp`. Lands on the `transform` frame tree (§transform 2.d).
- 2.b.i — Given `q`, compute the pose of **every** link frame relative to the model root
  (single forward sweep; revolute/prismatic apply `exp(axis·qᵢ)`, fixed apply origin only).
- 2.b.ii — Compute `T_base_tip` for a named end-effector, and the **relative** pose `T_a_b`
  between any two link frames (delegated to `transform` composition/inverse).
- 2.b.iii — **Emit a `transform` frame-tree snapshot** at a given timestamp: each movable joint
  becomes a timed parent←child edge, each fixed joint a static edge, so downstream consumers do
  tf-style buffered lookups (§transform 2.d) without knowing about joints.
- 2.b.iv — **Forward velocity kinematics**: given `(q, v)`, return the spatial twist of any link
  frame (the per-joint screw contributions accumulated along the path).
- Acceptance: FK is the identity at the neutral configuration up to fixed origins; FK agrees with
  closed-form poses on canonical chains (planar 2R/3R, a 6-DoF arm); the §2.b.iii snapshot path
  lookup reproduces the §2.b.ii direct composition to tolerance — all hardware-free.

### 2.c Differential kinematics — Jacobians
Definition: the linear map `v ↦ tip twist`, in geometric and analytic forms, plus the scalar
singularity/manipulability diagnostics. Layout per §1.b.iv. Models KDL `ChainJntToJacSolver`
and Pinocchio `computeJointJacobians` (research §C.1).
- 2.c.i — **Geometric Jacobian** `J(q)` for a tip frame: one column per velocity DoF, mapping `v`
  to the tip twist `[linear; angular]` in a stated frame; revolute column `[ω̂×r ; ω̂]`,
  prismatic column `[axis ; 0]` (screw form).
- 2.c.ii — Provide **both body and spatial** Jacobians and convert between them via the SE(3)
  **Adjoint** (§transform 2.c.iv) — never by re-deriving.
- 2.c.iii — **Analytic Jacobian** for a chosen orientation parameterization (RPY-rate or
  quaternion-rate): `J_a = E(φ)·J_geom`, where `E` is the representation map; document the
  parameterization's representation singularities (e.g. RPY gimbal at `pitch = ±π/2`).
- 2.c.iv — **Manipulability** (Yoshikawa `√det(JJᵀ)`) and **singularity detection** (rank /
  smallest singular value of `J`).
- 2.c.v — [scope] **Jacobian time-derivative** `J̇(q, v)` for acceleration-level / Coriolis-aware
  control. Marked scope: needed by `control`, lower-confidence to land closed-form initially.
- Acceptance: the analytical geometric Jacobian matches a **finite-difference of FK** —
  column `j ≈ log(T(q)⁻¹ · T(q ⊕ ε·eⱼ)) / ε` (a `transform` se(3) error twist) — to tolerance.
  This is the package's signature hardware-free verification (research §C.1: "finite-difference-
  verifiable without hardware").

### 2.d Inverse kinematics (IK)
> **[scope]** Research §C.1 explicitly scope-notes IK specifics — KDL and Drake are named as
> references, but **no single IK algorithm survived verification** this pass. The structures
> below are planning-grade (standard DLS / nullspace methods); re-confirm and bound ambition
> when authoring the plan. Avoid promising convergence guarantees the literature doesn't give.
Definition: the map `T_target → q` satisfying a task, numerically for general chains and in
closed form for specific well-known structures.
- 2.d.i — **Numerical IK by damped least squares** (Levenberg–Marquardt): iterate
  `q ← q ⊕ Jᵀ(JJᵀ + λ²I)⁻¹ e`, where `e = log(T_current⁻¹ · T_target)` is a `transform` se(3)
  error twist; converge on `‖e‖`. The damping `λ` keeps the step bounded near singularities.
- 2.d.ii — **Task masking / weighting**: position-only, orientation-only, or full 6-DoF tasks,
  with a task-space weighting matrix (the error metric is a weighted twist norm).
- 2.d.iii — **Joint limits + nullspace**: clamp/project iterates into limits; pursue a secondary
  objective (posture, joint-centering) in the Jacobian nullspace via `(I − J⁺J)`.
- 2.d.iv — [scope] **Closed-form analytic IK** for specific structures — a 6-DoF arm with a
  **spherical wrist** (position/orientation decoupling), and planar 2R/3R — provided
  per-structure, **not** as a general solver.
- 2.d.v — **Seed strategy, multi-solution handling, and deterministic failure**: accept a seed
  `q₀`, optionally enumerate/branch-select analytic solutions, and report non-convergence
  explicitly within a bounded iteration budget (no silent divergence).
- Acceptance: for poses sampled by FK from random reachable configurations, FK∘IK returns to the
  target within tolerance; non-convergence is reported, never faked; limit/nullspace projections
  keep iterates feasible.

### 2.e URDF parsing (model front end)
> **[scope]** URDF parsing is scope-noted in research §C.1/§E.3 ("URDF first"). The element set
> below is the standard URDF schema; treat wire-detail edge cases as plan-time work.
Definition: parse plain (pre-expanded) URDF XML into a §2.a model whose link frames map onto
`transform` frame ids. Models the ROS URDF schema (research §E.3).
- 2.e.i — Parse the URDF element model: `<robot>`, `<link>` (with optional `<inertial>`,
  `<visual>`, `<collision>`), `<joint>` with `type`, `<parent>`/`<child>`, `<origin xyz rpy>`,
  `<axis xyz>`, `<limit lower upper velocity effort>`, `<mimic>`, `<safety_controller>`.
- 2.e.ii — Map URDF joint types (revolute, continuous, prismatic, fixed, floating, planar) to the
  §2.a joint types and build the single-rooted link/joint tree. `<origin rpy>` is interpreted as
  **`transform` intrinsic Z-Y-X RPY** (§transform 2.a.i) so the convention is *inherited, not
  redefined*.
- 2.e.iii — Resolve `<mimic>` joints (`multiplier`/`offset`): a mimic joint follows its source
  and consumes **no** independent generalized coordinate.
- 2.e.iv — Produce a model whose link frames carry their URDF names as `transform` frame ids, so
  the §2.b.iii FK snapshot drops straight into a frame tree (no name remapping).
- 2.e.v — Deterministically reject malformed input (missing `parent`/`child`, cycles, multiple
  roots, unknown joint `type`); tolerate-but-flag unknown elements/attributes for forward-compat.
- [scope] Out of scope here (carried, not interpreted): `<inertial>` → opaque pass-through for
  `dynamics`; `<visual>`/`<collision>` geometry → opaque pass-through for `planning`; **Xacro**
  → pre-expand upstream (§1.c.v). Kinematics consumes only joint origins, axes, types, limits.
- Acceptance: parsing a recorded URDF fixture reproduces each joint's origin/axis as the expected
  `transform`, the tree topology, and the `(nq, nv)` layout — verified against text fixtures with
  no hardware (the model-file analog of the family's "recorded byte stream" parser tests, §3.b).

### 2.f SRDF parsing (semantic layer)
> **[scope]** SRDF detail is scope-noted with URDF (research §C.1/§E.3).
Definition: parse the SRDF semantic overlay onto an existing §2.e model — groups, end-effectors,
virtual joints, named states, disabled-collision pairs.
- 2.f.i — Parse `<group>` (joints / links / chains / sub-groups), `<end_effector>`,
  `<virtual_joint>` (fixed/floating/planar attaching the model root to a world frame),
  `<group_state>` (named configurations), and `<disable_collisions>`.
- 2.f.ii — Bind each group to a §2.a subchain and named tip frame, so `planning`/`control` select
  kinematic groups **by name** rather than by joint index.
- 2.f.iii — Expose `<group_state>` entries as named **configuration presets** (`q` values).
- 2.f.iv — Attach a `<virtual_joint>` (e.g. a floating base→world) onto the model root, extending
  `q`/`v` with the floating coordinates **consistently with the §1.b.i floating layout**.
- 2.f.v — Surface the `<disable_collisions>` set as **opaque data** for `planning` (§1.c.ii):
  this package stores it and acts on none of it.
- Acceptance: groups resolve to the correct ordered joint sets; a `<virtual_joint floating>` adds
  exactly `nq+=7 / nv+=6` to the model; named states round-trip through FK to expected poses.

---

## 3. Acceptance criteria (beyond per-feature tests)
- 3.a — **Convention conformance (inherited).** A test asserts this package overrides none of
  `transform` §1.b: Jacobian columns are translation-first twists (§1.b.iv), `q` ordering follows
  the §1.b.i traversal, and URDF `rpy` is the `transform` intrinsic Z-Y-X RPY (§2.e.ii).
- 3.b — **Hardware-free testability (math-layer pillar).** The full suite runs with no devices:
  FK identity/round-trip (§2.b), Jacobian-vs-finite-difference (§2.c.v acceptance), FK∘IK
  round-trip (§2.d), and URDF/SRDF parsed from **recorded model-file fixtures** (§2.e/§2.f) — the
  math-layer analog of the driver layers' recorded-byte-stream codec tests (family spec §3.b).
- 3.c — **No upward dependencies.** `kinematics` imports only `transform`, Cajeta's
  `numpy`/`linalg`/`tensor` substrate (for the IK linear solves / pseudo-inverse), and a Cajeta
  XML reader — and **nothing else in `dev.cajeta.robotica.*`** (family spec §1.b.ii).
- 3.d — **Board-agnostic, no native deps.** The package sits in the sense→…→actuate spine and
  pulls in no non-portable native library (family spec §3.c); the suite runs on x86-64 and at
  least one RISC-V target.
- 3.e — **Numerical robustness at singularities.** Jacobian diagnostics (§2.c.iv) and DLS IK
  (§2.d.i) behave deterministically at and near kinematic singularities (damping bounds the step;
  no NaN escapes); documented.
- 3.f — **Allocation & determinism.** FK and Jacobian hot paths are allocation-light against
  model storage pre-sized by `nq`/`nv`; IK iteration is bounded; parser error handling (§2.e.v)
  is deterministic on malformed input.

## 4. Deliverables
- 4.a — `dev.cajeta.robotica.kinematics` implementing §2, with the §3 test suite.
- 4.b — A **recorded URDF/SRDF fixture corpus** (canonical models — at minimum a 6-DoF arm with
  spherical wrist and a mobile base with a floating virtual joint) serving as the parser/FK/IK
  test vectors (the math/front-end layer's standing-in for "recorded byte streams").
- 4.c — A plan at `agents/kinematics-plan.md` (authored when this spec is approved), TDD-ordered
  along the dependency chain: model/chain (§2.a) → FK (§2.b) → Jacobians (§2.c) → URDF parse
  (§2.e) → SRDF (§2.f) → IK (§2.d, last — the scope-heaviest, depends on FK+Jacobian).

## 5. References (research-grounded)
- Pinocchio — spatial-algebra model, SE(3)-exp-map kinematics, and the **finite-difference-
  verifiable-without-hardware** property the §2.c Jacobian tests rely on (research §C.1).
  **[verified]** for the RBD/derivatives surface; the FK/Jacobian here shares its SE(3) basis.
- KDL/orocos (FK/IK + Jacobians) and Drake (multibody) — the named simpler-kinematics references
  for §2.a–§2.d (research §C.1). **[scope]**
- URDF / SRDF / Xacro robot-description model — the §2.e/§2.f front end; "URDF first" (research
  §C.1, §E.3). **[scope]**
- Inverse-kinematics methods — damped least squares / Levenberg–Marquardt, nullspace projection,
  analytic spherical-wrist decoupling (§2.d). General domain knowledge; research §C.1 explicitly
  scope-notes IK specifics. **[scope]**
- Micro-Lie theory exp/log/Jacobian basis (arXiv:1812.01537), consumed **via `transform` §B.1** —
  not reimplemented here. **[verified]** (inherited dependency).
