# `dev.cajeta.robotica.fusion` — Specification

The attitude- & state-**fusion** library of cajeta-robotica (layer: **math**). It turns raw
inertial samples (and optional aiding measurements) into a drift-corrected orientation or a
full INS-style navigation state. It depends on
[`transform`](../transform/transform-spec.md) for the SO(3)/SE(3) manifold (box-plus /
box-minus, exp/log, quaternion algebra) and on Cajeta's `linalg` for the matrix work; it
depends on **nothing else** in the family and never upward. See the family spec
[`../robotica-spec.md`](../robotica-spec.md) §2.f and the research
[`../../research/robotics-stack-part2-perception-to-ai.md`](../../research/robotics-stack-part2-perception-to-ai.md) §A.1.

---

## 1. Definition

`fusion` is **pure math over sensor samples**: feed it recorded accel/gyro/mag (and, for the
probabilistic tier, aiding observations) and it emits an orientation quaternion or a fused
state estimate with covariance. It hosts four tiers of increasing capability and cost:
a **complementary** baseline, the two classic single-gain AHRS filters — **Madgwick**
(gradient-descent quaternion) and **Mahony** (PI controller on SO(3)) — and the
probabilistic **EKF / UKF / error-state-KF** tier (a 15-state INS-style filter with an
IMU-driven predict and aiding updates). Every estimator is fully unit-testable **without
hardware** via algebraic identities, round-trips, finite-difference checks, and synthetic
sample streams.

### 1.a Capability summary
- 1.a.i — A common **fusion interface**: an IMU-sample input type, an estimator trait with
  `predict(imu, dt)` / `update(measurement)` / `state()`, and a shared noise/gain config.
- 1.a.ii — **Complementary** orientation filter (gyro high-pass + accel/mag low-pass) as the
  cheapest baseline and reference oracle.
- 1.a.iii — **Madgwick AHRS**: one gradient-descent step per sample fused with gyro
  integration by a single tunable gain β; IMU (6-axis) and MARG (9-axis) variants.
- 1.a.iv — **Mahony AHRS**: orientation error drives a proportional + integral correction of
  the gyro bias on SO(3); IMU and MARG variants.
- 1.a.v — **Error-state KF tier**: a 15-DoF INS error state (orientation, velocity, position,
  gyro bias, accel bias) with IMU **predict** and pluggable aiding **updates** (mag / GPS /
  odometry / vision), plus EKF and UKF realizations over the same manifold.

### 1.b Conventions (NORMATIVE by inheritance)
> This library **inherits, in full and without override**, the normative conventions fixed in
> [`transform` §1.b](../transform/transform-spec.md): **Hamilton quaternions, scalar-last
> storage `[x, y, z, w]`, `w ≥ 0` canonical sign**; **translation-first twist ordering**
> `ξ = [ρ; θ]`; **active, parent←child left-to-right composition**; **radians / meters /
> SI seconds**; right-handed frames. The quaternion-convention hazard (research §A.1, §B.1)
> is resolved exactly once, there. This spec **does not redefine** any of them.
- 1.b.i — **Body / world (NED-or-ENU) frame split is a config choice, fixed per estimator
  instance, not a new convention.** The estimated quaternion is `q_world_body` (rotates a
  body-frame vector into the world frame), consistent with transform's active-rotation rule.
  The gravity reference vector and magnetic reference are expressed in the chosen world frame
  and supplied at construction.
- 1.b.ii — **Manifold updates go through transform.** Every orientation increment is applied
  with transform's box-plus (`q ⊞ δθ`, right-multiplicative SO(3) retraction) and every
  orientation residual with box-minus (`q_a ⊟ q_b ∈ ℝ³`); `fusion` introduces **no** private
  retraction. Error covariances are therefore expressed in the 3-DoF SO(3) tangent, never in
  raw 4-vector quaternion space.

### 1.c Non-goals
- 1.c.i — **No sensor I/O or driver code.** Reading an IMU/mag/GPS off a bus is `io.sensor`;
  on-chip-fused IMUs (BNO055/BNO085) bypass this library entirely (research §A.2).
- 1.c.ii — **No SLAM / factor-graph / loop-closure estimation** (that is `estimation`); no
  pose-graph optimization. `fusion` is a recursive/online filter layer, not a smoother.
- 1.c.iii — **No magnetometer hard-/soft-iron calibration, accel/gyro intrinsic calibration,
  or temperature compensation.** `fusion` consumes already-calibrated samples; it exposes
  online **bias** estimation only (a filter state, not a factory calibration). [scope]
- 1.c.iv — **No measurement transport, time-sync, or buffering policy.** Samples arrive
  pre-timestamped; out-of-order/delayed-measurement handling beyond a simple latest-timestamp
  guard is out of scope for the first milestone. [scope]

---

## 2. Features

### 2.a Common fusion interface & sample types
Definition: the shared vocabulary every estimator speaks, so they are interchangeable behind
one trait and testable with one synthetic-stream harness.
- 2.a.i (use case) — Construct an `ImuSample` from accel (m/s²), gyro (rad/s), optional mag
  (unit-agnostic, normalized internally), and a transform `Time` stamp; validate finiteness.
- 2.a.ii — Drive any estimator through the uniform trait: `predict(imu, dt)` advances on a new
  sample, `update(measurement)` folds in an aiding observation (no-op for the pure-AHRS
  filters), `orientation() -> SO(3)` and (where applicable) `state()` read the estimate.
- 2.a.iii — Configure per-estimator gains/noises from one typed config struct (β, k_P/k_I,
  process/measurement noise densities) with documented SI units and sane defaults.
- 2.a.iv — Seed initial orientation from a single accel(+mag) sample (TRIAD/“align to
  gravity”) so a filter starts near-correct instead of at identity.
- 2.a.v — Report a per-estimate **quality/validity** signal (e.g. gyro-only dead-reckoning vs.
  aided, innovation magnitude, covariance trace) for downstream gating.

### 2.b Complementary filter (baseline)
Definition: the cheapest orientation estimator — fuse a gyro-integrated quaternion with an
accel(+mag)-derived absolute orientation by a single blend factor α; serves as the reference
oracle the AHRS filters are checked against.
- 2.b.i — Integrate gyro into orientation over `dt` via box-plus (`q ⊞ (ω·dt)`).
- 2.b.ii — Derive an absolute tilt from the accelerometer (gravity direction) and, in MARG
  mode, heading from the magnetometer; blend `q ← slerp(q_gyro, q_abs, 1−α)`.
- 2.b.iii — Expose α (or an equivalent cutoff time-constant τ given a sample rate) and degrade
  gracefully to gyro-only when accel magnitude departs from 1 g (linear-acceleration reject).
- Acceptance: with zero gyro noise and a static accel the estimate converges to the
  gravity-aligned attitude; gyro-only mode round-trips against transform `exp`/`log`.

### 2.c Madgwick AHRS — gradient-descent quaternion [verified]
Definition: orientation as the minimizer of the error between the measured field directions
and those predicted by the current quaternion; one gradient-descent step per sample fused with
gyro integration by a single gain β (research §A.1, x-io source).
- 2.c.i — **IMU (6-axis) variant.** Build the objective `f(q, ĝ, â)` measuring the mismatch
  between predicted gravity (`q⁻¹` applied to world-down) and the normalized accelerometer
  reading; compute the gradient `∇f = Jᵀ f` analytically from the closed-form Jacobian.
- 2.c.ii — **Gyro fusion.** Form the quaternion rate `q̇_ω = ½ q ⊗ ω`, subtract the
  normalized gradient scaled by β: `q̇ = q̇_ω − β · ∇f/‖∇f‖`; integrate over `dt` and
  renormalize.
- 2.c.iii — **MARG (9-axis) variant.** Add the magnetometer term; compute the earth-frame
  reference direction `b` from the measured field and **constrain magnetic distortion to the
  horizontal plane** (the Madgwick `b_x, b_z` projection) so magnetic disturbance corrupts
  heading only, never tilt.
- 2.c.iv — **Gain semantics.** Document β as the gyro-measurement-error rate (rad/s) and
  provide the published default; optionally derive β from a gyro noise spec.
- Acceptance: zero-noise gyro-only stream reproduces the analytic attitude (round-trip);
  static-attitude convergence to the gravity(+north) reference within tolerance; MARG heading
  is invariant to a synthetic horizontal-plane magnetic disturbance.

### 2.d Mahony AHRS — PI controller on SO(3) [verified]
Definition: the complementary/DCM filter posed as a **proportional+integral controller on
SO(3)** — the orientation error (measured vs. predicted field directions) drives a P
correction of the rate and an I correction of the gyro bias (research §A.1, x-io source).
- 2.d.i — **Error term.** Compute the orientation error `e = Σ vᵢ × v̂ᵢ` as the sum of cross
  products between each measured direction (gravity from accel; north from mag in MARG) and
  its estimate predicted from `q` — i.e. the `vex(·)` of the antisymmetric part of `R̂ᵀR`.
- 2.d.ii — **PI correction.** Integrate the bias estimate `ḃ = −k_I · e`; form the corrected
  rate `ω_c = ω − b̂ + k_P · e`; propagate `q̇ = ½ q ⊗ ω_c` over `dt` (via box-plus) and
  renormalize.
- 2.d.iii — **IMU and MARG variants** mirroring §2.c (accel-only vs. accel+mag direction set),
  with the same horizontal-only magnetic handling for heading.
- 2.d.iv — Expose `k_P`, `k_I`, published defaults, and an optional bias-clamp.
- Acceptance: with an injected constant gyro bias on an otherwise static stream the estimated
  bias converges to the injected value (the integral term's defining property); gyro-only and
  static-attitude checks as in §2.c.

### 2.e Error-state Kalman filter tier — 15-state INS [scope]
Definition: the probabilistic tier the family charter asked for. Structure to reproduce
(research §A.1 — *not verified this pass; build from this algorithm skeleton, re-research
`robot_localization` internals before the plan*): a 15-DoF **error-state** formulation that
keeps the filter linear around the current estimate and sidesteps quaternion-normalization
singularities, with an IMU-driven predict and sensor-agnostic aiding updates.
- 2.e.i — **Nominal state & error state.** Nominal `x = (q_world_body, v_world, p_world, b_g,
  b_a)` (16 stored params); error state `δx = [δθ, δv, δp, δb_g, δb_a] ∈ ℝ¹⁵`, with `δθ` the
  SO(3) tangent. Injection of `δx` into `x` uses transform box-plus (multiplicative on `q`,
  additive on the vector parts) followed by an **error-state reset** (the reset Jacobian `G`
  applied to `P`).
- 2.e.ii — **IMU predict.** Propagate the nominal state by integrating bias-corrected
  accel/gyro (gravity removed in the world frame); propagate the error covariance
  `P ← F P Fᵀ + G_w Q G_wᵀ` with the continuous-to-discrete error-state transition `F` and the
  process-noise mapping for gyro/accel white noise and bias random-walk. Bias states are
  modeled as random walks.
- 2.e.iii — **Aiding update (generic).** For an aiding measurement `z = h(x) + ν`,
  `ν ~ N(0, R)`: innovation `y = z ⊟ h(x)` (box-minus on any manifold-valued part), gain
  `K = P Hᵀ (H P Hᵀ + R)⁻¹` solved via Cajeta `linalg` (Cholesky, never an explicit inverse),
  `δx = K y`, `P ← (I − K H) P` (Joseph form for stability), then inject + reset per §2.e.i.
- 2.e.iv — **Concrete aiding models.** Provide at least: a **gravity/levelling** update
  (accel as a tilt observation), a **magnetometer/heading** update, a **velocity** update
  (wheel/visual odometry), a **position** update (GPS/vision), each as an `(h, H, R)` triple.
  Updates are pluggable and sensor-agnostic (the `robot_localization` model: many heterogeneous
  aiding sources into one `odom→base` state).
- 2.e.v — **UKF realization.** The same nominal state and aiding models driven by an
  **unscented** transform: sigma points generated on the manifold with box-plus, propagated
  through `f`/`h`, and recombined with box-minus weighted means/covariances — sharing §2.e.i's
  state definition and §2.e.iv's measurement models, differing only in propagation.
- 2.e.vi — **EKF degenerate / pure-attitude config.** A reduced 6-DoF (orientation + gyro
  bias) error-state attitude filter as the smallest member, so the tier subsumes the AHRS
  use case under the same math.
- Acceptance: (a) **linear-Gaussian sanity** — on a synthetic linear/no-rotation problem the
  ESKF output matches a textbook linear KF to tolerance; (b) **bias observability** — an
  injected constant gyro/accel bias is recovered under sufficient excitation; (c)
  **predict/update consistency** — Jacobians `F`, `H` agree with finite-difference of the
  nominal `f`, `h` (the hardware-free verification hook); (d) **filter consistency** — NEES
  on a synthetic Monte-Carlo trajectory stays within the χ² bounds; (e) **EKF≈UKF** — both
  realizations agree on a mildly-nonlinear synthetic run.

---

## 3. Acceptance criteria (beyond per-feature tests)
- 3.a — **Convention conformance:** every estimator emits `q_world_body` in transform's
  Hamilton/scalar-last form with `w ≥ 0`; all orientation increments/residuals route through
  transform box-plus/box-minus (asserted by a test that swaps in a transform spy).
- 3.b — **Hardware-free:** the whole suite runs in CI with no devices — recorded/synthetic
  accel/gyro/mag streams and analytic ground truth — on x86-64 and at least one RISC-V target;
  no non-portable deps (the math layer stays board-agnostic, family pillar §3.c).
- 3.c — **Finite-difference verification:** all analytic Jacobians (Madgwick `∇f`, ESKF/UKF
  `F`/`H`) are checked against numerical differentiation of their generating functions.
- 3.d — **Round-trip / identity:** gyro-only integration of any estimator is exactly
  transform `exp`-integration to tolerance; a zero-input static stream holds the seeded
  attitude.
- 3.e — **No upward dependencies:** `fusion` imports only `transform` and Cajeta
  `linalg`/`tensor`/`numpy`; nothing else in `dev.cajeta.robotica.*`.
- 3.f — **Allocation discipline:** the AHRS filters (§2.b–2.d) allocate nothing on the
  per-sample hot path; the KF tier's allocation is bounded by fixed state/measurement
  dimensions (no per-step heap growth).
- 3.g — **Numerical robustness:** covariance stays symmetric positive-definite across a long
  run (Joseph-form / Cholesky), and orientation stays normalized (no quaternion drift) under
  extended integration.

## 4. Deliverables
- 4.a — `dev.cajeta.robotica.fusion` implementing §2, with the §3 test suite (synthetic IMU
  stream generators + analytic oracles checked in as fixtures).
- 4.b — A small set of reusable **synthetic-trajectory fixtures** (static, pure-rotation,
  constant-bias, excited-INS) usable by downstream estimation/control tests.
- 4.c — A plan at `agents/fusion-plan.md` (authored when this spec is approved), TDD-ordered:
  interface & sample types → complementary baseline → Madgwick (IMU then MARG) → Mahony
  (IMU then MARG) → ESKF (state+predict, then generic update, then concrete aiding models) →
  UKF realization. The KF tier (§2.e) is gated on a `robot_localization`/ESKF re-research pass
  per family-spec §5 before its plan section is finalized.

## 5. References (research-grounded)
- **Madgwick** gradient-descent quaternion AHRS — x-io.co.uk open-source IMU/AHRS algorithms.
  **[verified]** (research §A.1).
- **Mahony** complementary/DCM filter as a PI controller on SO(3) — x-io.co.uk; Mahony et al.,
  *Nonlinear Complementary Filters on SO(3)*. **[verified]** (research §A.1).
- **EKF / UKF / error-state (indirect) KF**, 15–16-state INS structure (orientation, velocity,
  position, gyro bias, accel bias), IMU predict + aiding updates; `robot_localization` (ROS)
  as the sensor-agnostic reference node. **[scope — not verified this pass; re-research before
  the §2.e plan]** (research §A.1, gap list §J).
- **Manifold retraction (box-plus / box-minus), SO(3) exp/log/Jacobians** consumed from
  `transform`; micro-Lie theory (arXiv:1812.01537). **[verified]** (research §B.1, via
  `transform`).
- **Quaternion-convention hazard** (Hamilton vs. JPL, scalar order) — resolved once in
  `transform` §1.b. **[verified]** (research §A.1, §B.1).
