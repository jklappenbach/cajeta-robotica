# `dev.cajeta.robotica.control` — Specification

The **control-above-the-drive** package of cajeta-robotica: the feedback laws that turn a
desired motion/contact behavior into **joint/torque setpoints** handed down to `motor` (which
closes the FOC current loop). It sits on the **math** layer of the spine
(sense → estimate → plan → **control** → actuate) and is therefore **board-agnostic** — pure
computation over `transform` types and `dynamics` quantities, fully unit-testable without
hardware. See the family spec [`../robotica-spec.md`](../robotica-spec.md) §2.k and research
[`../../research/robotics-stack-part2-perception-to-ai.md`](../../research/robotics-stack-part2-perception-to-ai.md) §D.

---

## 1. Definition

`control` provides the feedback-control payload that rides **above** the drive's torque/current
loop and **below** the planner: scalar/vector **PID** (with tuning + anti-windup),
**impedance/admittance** control (a virtual mass-spring-damper rendered at the end effector),
**operational-/task-space** control, **model-predictive control (MPC)**, **whole-body control
(WBC)**, and the **shared quadratic-program (QP) solver** that WBC and MPC both reduce to
(dense for WBC, sparse for MPC). Every controller exposes a common per-tick interface and emits
**setpoints into `motor`**; none of them touch a bus, a sensor IC, or a non-portable native lib.

### 1.a Capability summary
- 1.a.i — A **QP solver** core in one fixed standard form, with a **dense active-set** path
  (small WBC problems, sub-millisecond) and a **sparse augmented-Lagrangian** path (banded MPC
  horizons), supporting warm-starting and infeasible-problem handling. **[verified core]**
- 1.a.ii — **PID** (scalar and per-joint vector) with anti-windup, derivative-on-measurement,
  feed-forward, output saturation, bumpless transfer, and offline/relay tuning. **[scope]**
- 1.a.iii — **Impedance / admittance** control: a Cartesian virtual mass-spring-damper at the
  end effector, with hybrid force/motion selection. **[scope]**
- 1.a.iv — **Operational-space (task-space) control** (Khatib OSC): map a task-space wrench to
  joint torques through the Jacobian, with nullspace posture projection. **[scope]**
- 1.a.v — **MPC**: discretize a (linearized) plant over a receding horizon and reduce tracking
  + constraints to the sparse QP of §1.a.i. **[scope]**
- 1.a.vi — **WBC**: map prioritized Cartesian task objectives + physical constraints (torque
  limits, friction cones, contacts, joint limits) to joint torques via the dense QP. **[scope]**
- 1.a.vii — A common **Controller interface** + **setpoint/limit contract** emitted into `motor`
  (position / velocity / torque setpoints, saturated and rate-limited, with a deadline hold).

### 1.b Conventions
> This package **inherits, by reference, all normative conventions fixed in
> [`../transform/transform-spec.md`](../transform/transform-spec.md) §1.b** — quaternion
> (Hamilton, scalar-last `[x,y,z,w]`), **twist ordering translation-first** `ξ = [ρ; θ]`, the
> matching **wrench** dual `[force; torque]`, active left-to-right `parent←child` composition,
> and **radians / meters / SI-seconds**. It does **not** redefine any of them. The items below
> are the **additional** package-local normative forms control fixes for its own surface; they
> never contradict transform.
- 1.b.i — **QP standard form (fixed):** `min_x ½ xᵀ H x + gᵀ x` subject to `A x = b` and
  `l ≤ C x ≤ u` (box constraints are the special case `C = I`). `H` is symmetric
  positive-(semi)definite. Every controller that reduces to a QP targets *this* form so a single
  solver serves all of them.
- 1.b.ii — **Torque/effort sign:** a joint command `τ_i > 0` produces motion in the **positive
  joint-coordinate direction** defined by the `dynamics`/`kinematics` model; wrenches/twists at
  the end effector follow transform §1.b.ii ordering. No package-local re-sign.
- 1.b.iii — **Controller tick contract:** every controller is a pure function
  `update(measured_state, reference, dt) → setpoint` plus bounded internal state (integrator,
  warm-start, filter); `dt` is an explicit SI-second `Duration`, never an implicit wall clock.
- 1.b.iv — **Stiffness/damping in SE(3):** Cartesian gains `K`, `D`, and virtual inertia `M`
  are expressed in the end-effector twist basis (§1.b ordering) so impedance gains compose with
  transform/dynamics quantities without reordering.

### 1.c Non-goals
- 1.c.i — **No drive/current loop.** FOC, commutation, and the wire protocol are `motor`'s job;
  control stops at the setpoint handed across that seam.
- 1.c.ii — **No motion planning / trajectory generation.** Global paths and jerk-limited OTG are
  `planning`; control consumes the *reference* a planner emits, it does not produce one.
- 1.c.iii — **No dynamics/kinematics models.** Mass matrix `M(q)`, bias `C(q,q̇)q̇ + g(q)`,
  Jacobians `J(q)`, RNEA/ABA/CRBA come from `dynamics`/`kinematics`; control consumes them.
- 1.c.iv — **No estimation.** The measured state is provided by `fusion`/`estimation`; control
  does not filter raw sensors (it may low-pass its own derivative term, §2.b).
- 1.c.v — **No hardware, no native deps.** Board-agnostic spine rule (family §3.c): the entire
  package is portable Cajeta over `numpy`/`linalg`/`transform`/`dynamics`.

---

## 2. Features

### 2.a QP solver — the shared high-frequency core **[verified core]**
Definition: a quadratic-program solver in the §1.b.i standard form, the common reduction target
of WBC and MPC. Research §D names the verified landscape: **ProxQP** (augmented-Lagrangian,
sparse, converges to the *closest feasible* QP when infeasible — a safety-relevant property),
and **eiquadprog / qpmad** (dense active-set, Goldfarb-Idnani, sub-millisecond on small WBC
problems). Both paths are heavy linalg on Cajeta's existing Cholesky/LDLᵀ surface.
- 2.a.i (use case) — Solve a **dense** QP (active-set / Goldfarb-Idnani) for a small WBC problem
  (tens of variables, tens of constraints) in **sub-millisecond** time, returning the primal `x`
  and the active set.
- 2.a.ii — Solve a **sparse / banded** QP (augmented-Lagrangian) for an MPC horizon (block-banded
  `H`, `C` from the lifted dynamics), exploiting the structure rather than densifying it.
- 2.a.iii — **Warm-start** from the previous tick's solution / active set (and, for MPC, a
  shifted horizon), so steady-state ticks converge in few iterations.
- 2.a.iv — **Infeasible handling:** detect infeasibility and return the **closest feasible**
  solution (augmented-Lagrangian path) with a status flag, rather than diverging or throwing —
  the property the research flags as safety-relevant for a control loop.
- 2.a.v — Report solver **status, iteration count, primal/dual residuals**, and a deterministic
  iteration cap so a real-time loop has a bounded worst-case.
- 2.a.vi — Expose **equality-only** and **box-only** fast paths (equality-constrained QP = one
  KKT solve; box-constrained = projected solve) used by the simpler controllers directly.
- Acceptance: against problems with **known closed-form KKT solutions** (equality-constrained,
  active-set-by-hand) the solver matches to tolerance; an **infeasible** fixture returns the
  closest-feasible point; warm-started vs. cold-started solutions agree; all hardware-free.
- [scope] note: the 2025 T-RO benchmark ranking favoring ProxQP comes from single studies (one
  possibly co-authored by the ProxQP group); treat the *ranking* as directional. The standard
  form (§1.b.i) and the two algorithm families are the [verified] surface to build to.

### 2.b PID control + tuning **[scope]**
Definition: the workhorse single-/multi-loop feedback law, in a numerically careful form. Research
§D lists PID(+tuning) as [scope] above the QP — structure known, not verified this pass.
- 2.b.i — **Scalar PID** with proportional, integral, and derivative terms over an explicit `dt`
  (§1.b.iii); output `u = Kp·e + Ki·∫e + Kd·de/dt`.
- 2.b.ii — **Per-joint vector PID** (a diagonal bank) producing a joint-space effort/position
  command vector in one call.
- 2.b.iii — **Anti-windup**: integrator clamping and back-calculation, so the integral term
  stops accumulating when the output saturates.
- 2.b.iv — **Derivative on measurement** (not on error) with a configurable low-pass on the
  derivative, to avoid setpoint-step kick and amplified noise.
- 2.b.v — **Feed-forward** term (velocity/accel/gravity ff supplied by the caller) summed into
  the output ahead of the feedback correction.
- 2.b.vi — **Bumpless transfer / reset**: re-initialize integrator and last-measurement on
  mode/gain change so the output does not jump.
- 2.b.vii — **Tuning helpers**: offline **Ziegler-Nichols** from a step/ultimate-gain
  characterization, and a **relay (Åström-Hägglund) auto-tune** producing gain triples.
  [scope] — verify the tuning formulas against a primary source before the plan.
- Acceptance: a unit-step on a known first-/second-order discrete plant yields the analytic
  closed-loop response to tolerance; anti-windup bounds the integrator under sustained
  saturation; derivative-on-measurement shows no setpoint-step spike. Hardware-free.

### 2.c Impedance / admittance control **[scope]**
Definition: render a **virtual mass-spring-damper** at the end effector — the essential
contact/manipulation primitive (research §D, [scope]). Target dynamics in the Cartesian twist
basis (§1.b.iv): `M ẍ + D ẋ + K x = F_ext`, with `x` the SE(3) pose error and `F_ext` a wrench.
- 2.c.i — **Impedance** mode: from measured Cartesian motion error (pose/twist) compute the
  commanded **wrench** `F = K·Δx + D·Δẋ + M·ẍ_des`, then map it to joint **torques** via the
  Jacobian transpose (`τ = Jᵀ F`); requires torque-capable drives.
- 2.c.ii — **Admittance** mode: from a measured external **wrench** (F/T sensor, supplied by the
  caller) integrate the virtual dynamics to a commanded **motion** (pose/twist) reference;
  requires a position/velocity-controlled inner loop.
- 2.c.iii — **Cartesian stiffness/damping** specified as 6×6 (or diagonal) `K`/`D` in the
  end-effector frame, decoupled into translational and rotational blocks per transform ordering.
- 2.c.iv — **Hybrid force/motion** via a **selection matrix** `S`: stiff (motion-controlled)
  directions and compliant (force-controlled) directions chosen per axis.
- 2.c.v — **Gravity/Coriolis compensation** feed-forward (from `dynamics`) added so the rendered
  impedance is about the desired equilibrium, not corrupted by the arm's own weight.
- Acceptance: with `F_ext = 0` the end effector holds equilibrium; a static virtual spring
  returns the analytic deflection `Δx = K⁻¹F` for a constant applied wrench; the
  impedance↔admittance duality round-trips on a 1-DoF fixture. Hardware-free.

### 2.d Operational-/task-space control (OSC) **[scope]**
Definition: Khatib operational-space control — regulate an end-effector task while exploiting
redundancy, mapping a task-space command to joint torques through the dynamics. Research §D
lists task-space control as [scope] above the QP.
- 2.d.i — Compute the **task-space (operational-space) inertia** `Λ = (J M⁻¹ Jᵀ)⁻¹` from the
  `dynamics` mass matrix and the task Jacobian.
- 2.d.ii — **Task wrench → joint torque:** `τ = Jᵀ (Λ ẍ_des + μ + p) + N τ₀`, where `μ`, `p` are
  the task-space Coriolis/gravity terms and `N = I − Jᵀ J̄ᵀ` is the dynamically-consistent
  **nullspace projector**.
- 2.d.iii — **Secondary (posture) task** in the nullspace `τ₀` (e.g. joint-limit avoidance,
  preferred posture) that does not disturb the primary task.
- 2.d.iv — **Multiple prioritized tasks** stacked by successive nullspace projection (strict
  priority) — the structural bridge to WBC's soft-priority QP form (§2.f).
- 2.d.v — **Singularity-robust** task inverse (damped least-squares `J̄`) near kinematic
  singularities, with a configurable damping schedule.
- Acceptance: on a redundant planar/3-link fixture with a known `M(q)`, the primary task tracks
  while a nullspace posture term provably leaves the task acceleration unchanged (finite-
  difference check); damped inverse stays bounded through a singularity. Hardware-free.

### 2.e Model-predictive control (MPC) **[scope]**
Definition: receding-horizon optimal control that **reduces to the sparse QP of §2.a**. Research
§D, [scope] — structure given, internals not verified this pass.
- 2.e.i — **Plant discretization:** from a (linearized/LTV) continuous model produce per-step
  `x_{k+1} = A_k x_k + B_k u_k` over a horizon `N`.
- 2.e.ii — **Build the QP:** assemble quadratic state/input tracking cost (`Q`, `R`, terminal
  `Q_N`) and constraints (state/input boxes, slew limits) into the §1.b.i standard form — either
  **sparse/lifted** (state kept as variables, banded `H`/`C`) or **condensed** (states
  eliminated, dense small `H`) — and solve via §2.a.
- 2.e.iii — **Receding horizon:** apply only `u_0`, shift the horizon, and **warm-start** the
  next solve from the shifted previous solution (§2.a.iii).
- 2.e.iv — **Reference preview:** accept a time-indexed reference trajectory (from `planning`)
  over the horizon, not just a single setpoint.
- 2.e.v — **Feasibility/recovery:** on an infeasible horizon, fall through to §2.a.iv's
  closest-feasible solution and surface the status (soft-constraint slack as the configured
  recovery).
- Acceptance: on a linear double-integrator with box input limits, the unconstrained MPC matches
  the analytic LQR gain over a long horizon, and the constrained MPC respects the input box at
  every step; condensed and sparse builds give the same `u_0` to tolerance. Hardware-free.

### 2.f Whole-body control (WBC) **[scope]**
Definition: a per-tick **dense QP** that maps prioritized Cartesian task objectives plus physical
constraints to joint torques (research §D, [scope]) — the OSC stack (§2.d) recast as one
optimization so constraints are handled exactly.
- 2.f.i — **Decision variables:** joint accelerations `q̈` (and optionally contact wrenches `λ`,
  joint torques `τ`) in one stacked vector `x`.
- 2.f.ii — **Dynamics as equality constraint:** `M q̈ + h = Sᵀ τ + Jcᵀ λ` (from `dynamics`)
  enters as `A x = b` (§1.b.i).
- 2.f.iii — **Task objectives** (Cartesian motion tracking `J q̈ + J̇ q̇ = ẍ_des`, posture,
  momentum) as **weighted cost terms** in `H`, `g` — soft priorities via weights.
- 2.f.iv — **Strict priorities** (optional): a hierarchical/cascaded QP where higher levels are
  solved first and lower levels operate in their nullspace.
- 2.f.v — **Inequality constraints:** joint-position/velocity/torque limits, **friction cones**
  (linearized as `l ≤ C λ ≤ u`), and unilateral contact (`λ_normal ≥ 0`) as box/general
  inequalities for §2.a.
- 2.f.vi — Emit the resulting **joint torques** as the setpoint (§2.g), sub-millisecond per tick
  via the dense path (§2.a.i).
- Acceptance: a floating-/fixed-base fixture with a known `M(q)` and one Cartesian task solves to
  a `q̈` that satisfies the dynamics equality and respects the torque box; removing a constraint
  recovers the unconstrained OSC solution (§2.d) to tolerance. Hardware-free.

### 2.g Controller interface & setpoint contract
Definition: the common shape every controller presents and the **typed setpoint** it hands to
`motor`, so controllers compose and the spine seam stays clean (board-agnostic, family §3.c).
- 2.g.i — A **`Controller` interface**: `update(state, reference, dt) → Setpoint` (§1.b.iii)
  with explicit `reset()` and bounded internal state; controllers are values, not threads.
- 2.g.ii — **Setpoint kinds** emitted into `motor`: position, velocity, and torque/effort
  setpoints (per joint), tagged so `motor` selects the matching drive mode (e.g. CiA 402
  CSP/CSV/CST, family §2.e).
- 2.g.iii — **Saturation & rate-limiting** of the setpoint against supplied joint
  position/velocity/torque limits, *inside* control, before it crosses the seam.
- 2.g.iv — **Deadline / staleness hold:** if `dt` exceeds a configured bound or the reference is
  stale, emit a defined safe setpoint (hold / zero-torque) and flag it — the hook `safety`
  (family §2.n) builds its watchdog reaction on; control owns the saturation, `safety` owns the
  policy.
- 2.g.v — **Composition:** controllers chain (e.g. impedance outer → PID inner, or OSC →
  per-joint torque) through this one interface without bespoke glue.
- Acceptance: setpoints never exceed supplied limits (property test over random references); a
  forced stale/oversized `dt` produces the defined safe setpoint with the status flag set.
  Hardware-free.

---

## 3. Acceptance criteria (beyond per-feature tests)
- 3.a — **One QP form, one solver.** Every reduction-based controller (§2.d–§2.f) targets the
  §1.b.i standard form and is solved by §2.a; a test asserts each builds a well-formed QP (symmetric
  PSD `H`, consistent constraint dimensions).
- 3.b — **Convention conformance.** Inherited transform conventions (§1.b) hold at every boundary
  — twist/wrench ordering into Jacobian maps, torque sign — asserted by tests; control redefines
  none of them.
- 3.c — **Hardware-free.** The whole suite runs in CI with no devices, on x86-64 and at least one
  RISC-V target, using `identity / round-trip / finite-difference` checks (family §3.b) — analytic
  KKT/LQR baselines, virtual-spring deflection, nullspace-invariance finite differences.
- 3.d — **Strict dependency direction.** `control` imports only `transform`, `dynamics`
  (and through it `kinematics`), and Cajeta `numpy`/`linalg` — **never upward** (no `planning`,
  `motor`, `safety`, `fusion`); emits setpoints to `motor` purely as a typed value (family §1.b.ii).
- 3.e — **Real-time discipline.** Solver and controller ticks have a **bounded worst-case**
  (deterministic iteration cap, §2.a.v) and allocate nothing on the hot path beyond solver
  working buffers sized once at construction; the dense WBC path meets a sub-millisecond budget
  on the small-problem fixture.
- 3.f — **Infeasibility is graceful.** No controller throws or diverges on an infeasible/
  ill-conditioned tick; it returns a flagged closest-feasible / safe result (§2.a.iv, §2.g.iv).

## 4. Deliverables
- 4.a — `dev.cajeta.robotica.control` implementing §2, with the §3 test suite.
- 4.b — A plan at **`agents/control-plan.md`** (authored when this spec is approved), TDD-ordered
  bottom-up: **QP solver core (§2.a)** → PID (§2.b) → impedance/admittance (§2.c) → OSC (§2.d) →
  MPC (§2.e) → WBC (§2.f), with the controller interface/setpoint contract (§2.g) threaded
  through each. The QP solver is the first deliverable because every other unit reduces to it.
- 4.c — A short note documenting the §1.b.i **QP standard form** and the §1.b.iii **tick
  contract** as the package's local citation, alongside the inherited transform conventions.

## 5. References (research-grounded)
- **QP solvers** — ProxQP (augmented-Lagrangian, closest-feasible on infeasibility); eiquadprog /
  qpmad (dense active-set, Goldfarb-Idnani, sub-ms small WBC). T-RO 2025 benchmark
  (https://scaron.info/publications/tro-2025.html) + 2025 solver review
  (https://arxiv.org/html/2510.21773v1). Research §D. **[verified core]** *(ranking directional —
  single studies, one possibly co-authored by the ProxQP group).*
- **Impedance / admittance, OSC, MPC, WBC** — the algorithmic payload above the QP per research
  §D. **[scope]** — structures known (virtual MSD at the end effector; Khatib operational-space
  inertia + nullspace; receding-horizon QP; constraint-exact whole-body QP), **not verified this
  pass**; verify against primary sources (Khatib OSC; standard MPC/WBC texts) before the plan.
- **Dynamics quantities consumed** (`M`, `C`, `g`, `J`, RNEA/ABA/CRBA) — Pinocchio model,
  research §C.1 / family §2.i. **[verified]** (provided by `dynamics`, not reimplemented here).
- **Transform conventions inherited** — [`../transform/transform-spec.md`](../transform/transform-spec.md)
  §1.b; micro-Lie theory arXiv:1812.01537 / Sophus twist ordering, research §B.1. **[verified]**
- **Safety hook target** (deadline/watchdog reaction, PFL constraints injected into control) —
  research §F / family §2.n. **[scope]** — control exposes the saturation/hold hooks; the policy
  lives in `safety`.
