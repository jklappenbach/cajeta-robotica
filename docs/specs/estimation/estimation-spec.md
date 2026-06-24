# `dev.cajeta.robotica.estimation` — Specification

The **SLAM & localization** layer of cajeta-robotica: factor-graph estimation, pose-graph
optimization, odometry, scan matching, and particle-filter localization. A **math layer** —
pure computation over recorded measurements, fully unit-testable **without hardware**. It
depends on `transform` (SE(3)/SO(3) exp-maps, the manifold retraction, core types) and on
Cajeta's `numpy`/`linalg` sparse-solver substrate; it depends on nothing else in the family
and **never upward**. See the family spec [`../robotica-spec.md`](../robotica-spec.md) §2.g and
the research [`../../research/robotics-stack-part2-perception-to-ai.md`](../../research/robotics-stack-part2-perception-to-ai.md)
§B.3.

---

## 1. Definition

`estimation` builds **probabilistic state estimates of a robot and its world** by posing them
as **MAP inference on a factor graph**: a bipartite graph of *variable* nodes (poses,
landmarks, biases, calibration) and *factor* nodes (each a measurement's negative-log-
likelihood term). Maximizing the **product of factors** reduces to a **sparse nonlinear least-
squares solve** (Gauss-Newton / Levenberg-Marquardt over a sparse Cholesky or QR
factorization) — squarely on Cajeta's existing sparse-linalg surface. On top of that engine it
ships the standard SLAM/localization building blocks: **pose-graph optimization**, **odometry**
front-ends, **scan matching / ICP**, and **particle-filter (Monte-Carlo) localization**. The
modeling reference for the graph engine is **GTSAM** (research §B.3 **[verified]**); the
front-end building blocks model **g2o/Ceres**, **ICP**, and **AMCL** (research §B.3 **[scope]**).

### 1.a Capability summary
- 1.a.i — A **factor graph** over manifold variables: add variables, add factors (priors,
  between-factors, projection/range/bearing factors, custom residuals), evaluate the joint cost.
- 1.a.ii — A **sparse nonlinear least-squares optimizer** (Gauss-Newton, Levenberg-Marquardt,
  dogleg) solving the linearized normal equations via sparse Cholesky/QR with fill-reducing
  ordering and robust (M-estimator) kernels.
- 1.a.iii — **Pose-graph optimization (PGO):** SE(3)/SE(2) poses linked by relative-pose
  measurements and loop closures, optimized to a globally consistent trajectory.
- 1.a.iv — **Odometry & incremental estimation:** accumulate relative motion into the graph;
  fixed-lag smoothing / incremental re-solve so online cost stays bounded.
- 1.a.v — **Scan matching / ICP:** register two point clouds into a relative SE(3) transform
  (with covariance) to feed the graph front-end.
- 1.a.vi — **Particle-filter localization:** Monte-Carlo localization of a pose against a known
  map from a motion model + a measurement model, with adaptive sample size and resampling.

### 1.b Conventions (INHERITED — not redefined here)
> This package **inherits, by reference, the normative conventions fixed in `transform` §1.b**
> and never overrides them. Quaternion convention (Hamilton, scalar-last `[x,y,z,w]`), twist /
> se(3) tangent ordering (**translation-first** `ξ = [ρ; θ]`), active left-to-right
> `parent←child` composition, and SI units (radians / meters / seconds) are **defined there**
> and used verbatim here. In particular:
- 1.b.i — All on-manifold optimization uses `transform`'s retraction pair **`⊞`/`⊟`** (box-plus
  / box-minus, transform §2.c.vi): a variable update is `x ⊞ δ` and a residual between manifold
  values is `a ⊟ b`, so the solver's increment `δ` always lives in the tangent space with the
  family-standard ordering. Jacobians are the left/right Jacobians of transform §2.c.v.
- 1.b.ii — Measurement uncertainty is expressed as an **information (inverse-covariance) form**
  in the same tangent basis; a factor's whitened residual is `Σ^{-1/2}(h(x) ⊟ z)`.
- 1.b.iii — Poses, twists, and wrenches are the `transform` types; this package adds **no new
  rotation/twist representation.**

### 1.c Non-goals
- 1.c.i — **Not the frame tree.** `transform` §2.d owns the time-stamped SE(3) frame graph and
  buffered lookup. `estimation` *complements* it: it **consumes** odometry/sensor transforms and
  **produces** a correction (the `map→odom` / `odom→map` estimate of family §2.g.ii) that a
  consumer publishes back **into** the frame tree. It does not re-implement buffered interpolation.
- 1.c.ii — **No sensor I/O or feature extraction.** Raw IMU/encoder/LiDAR/camera access is
  `io.*`; image feature detection/description and depth front-ends are `io.vision`. `estimation`
  starts from already-associated measurements (point clouds, relative transforms, range/bearing).
- 1.c.iii — **No attitude AHRS / low-level inertial fusion** (Madgwick/Mahony/error-state-KF) —
  that is `fusion` (family §2.f). `estimation` may *ingest* a fused state as a prior/odometry
  factor but does not host the AHRS filters.
- 1.c.iv — **No dynamics, kinematics, planning, or control.** No dense-volumetric mapping,
  surfel/TSDF reconstruction, or learned front-ends (those are out of layer / `ai`).
- 1.c.v — **No hidden global state**; graphs and filters are explicit owned values, freed
  deterministically (Cajeta ownership) — relevant for the bounded-latency online path.

---

## 2. Features

### 2.a Factor graph — variables and factors
Definition: a bipartite graph whose **variable** nodes hold manifold-valued unknowns (keyed by a
typed symbol, e.g. `x0`, `l3`, `b1`) and whose **factor** nodes each contribute a cost
`φ_i(X_i) = ½‖ h_i(X_i) ⊟ z_i ‖²_Σ` over the subset `X_i` of variables they touch. The joint
posterior is the **product of factors**; its negative log is the sum of those costs (GTSAM
model, research §B.3 [verified]).
- 2.a.i (use case) — Insert a variable of a given manifold type (SE(3)/SE(2)/SO(3) pose, R^n
  point/landmark, bias/calibration vector) under a unique key; read/initialize its value.
- 2.a.ii — Add a **prior factor** pinning a variable to a measured value with an information
  matrix (anchors gauge freedom — at least one prior is required for a well-posed solve).
- 2.a.iii — Add a **between factor** `z = a⁻¹ b` constraining the relative transform of two
  pose variables (the PGO edge and the odometry edge).
- 2.a.iv — Add **measurement factors** parameterized by a residual function `h` and its
  Jacobians: range, bearing, range-bearing, projection (landmark→pixel), and a generic
  user-supplied residual (custom factor) over an arbitrary variable subset.
- 2.a.v — Evaluate total graph error at the current values; enumerate per-factor residuals and
  whitened errors (for χ² gating / diagnostics).
- Acceptance: a graph with a single between-factor and matching prior has **zero error** at the
  ground-truth values; analytic factor Jacobians match finite differences to tolerance (no
  hardware).

### 2.b Sparse nonlinear least-squares optimizer
Definition: the engine that minimizes the §2.a cost. Each iteration **linearizes** every factor
about the current estimate to form the sparse system `A δ = b` (equivalently the normal
equations `(AᵀΣ⁻¹A) δ = AᵀΣ⁻¹b`, an information matrix `Λ`), solves for the tangent increment
`δ`, and updates `x ← x ⊞ δ`. Models GTSAM/g2o/Ceres (research §B.3 — GTSAM [verified],
g2o/Ceres [scope]).
- 2.b.i — **Gauss-Newton** step: assemble the sparse Jacobian/Hessian and solve via Cajeta
  `linalg` **sparse Cholesky** (`LL^T`/`LDL^T`); fall back to **sparse QR** on `A` directly when
  the system is rank-deficient/ill-conditioned (QR is the more stable factor-graph default).
- 2.b.ii — **Levenberg-Marquardt** and **Powell's dogleg** trust-region variants for robustness
  far from the optimum (damping `λ` adaptation on the diagonal).
- 2.b.iii — **Fill-reducing variable ordering** (COLAMD/AMD-style) chosen before factorization so
  the Cholesky factor stays sparse; expose the elimination ordering as a tunable.
- 2.b.iv — **Robust kernels** (M-estimators: Huber, Cauchy/Lorentzian, Tukey, Dynamic Covariance
  Scaling) applied per-factor to down-weight outliers / spurious loop closures via iteratively
  re-weighted residuals.
- 2.b.v — Convergence control: absolute/relative error and increment-norm thresholds, max
  iterations, and a recoverable **non-convergence / indeterminate-system** result (never a hard
  crash on a singular graph).
- 2.b.vi — Recover the **marginal covariance** of selected variables from the factorization
  (back-substitution over the Cholesky factor) for uncertainty reporting and χ² loop-closure
  gating.
- Acceptance: on a synthetic graph with known optimum, GN/LM converge to it from a perturbed
  seed; the recovered marginal covariance matches a dense brute-force inverse on a small problem.

### 2.c Manifold variables & retraction
Definition: the glue making the §2.b linear-algebra increment respect curved state spaces, using
`transform`'s `⊞`/`⊟` (§1.b.i). Each variable type declares its tangent dimension, its retraction
`⊞`, its local-coordinate `⊟`, and the chain-rule Jacobian blocks.
- 2.c.i — SE(3) and SE(2) pose variables: 6- and 3-dimensional tangents using transform `exp`/`log`.
- 2.c.ii — SO(3)/SO(2) rotation-only and R^n Euclidean (point/landmark/bias) variables.
- 2.c.iii — Product/composite variables (e.g. pose + velocity + bias) with block-diagonal
  retraction, for the navigation-state node a VIO/INS graph needs.
- 2.c.iv — Correct **left vs. right** Jacobian convention threaded consistently from the variable
  type into each factor's Jacobian (the manif-vs-Sophus hazard, transform §1.b / research §B.1).
- Acceptance: `(x ⊞ δ) ⊟ x == δ` and `x ⊞ (y ⊟ x) == y` to tolerance for every variable type
  (round-trip), exercised at and near the small-tangent (`δ→0`) regime.

### 2.d Pose-graph optimization (PGO)
Definition: the canonical SLAM back-end — a graph whose variables are robot poses along a
trajectory and whose factors are **odometry between-factors** (consecutive poses) plus
**loop-closure between-factors** (re-visited places), solved by §2.b. (research §B.3: GTSAM
[verified] backbone; loop-closure/front-end [scope].)
- 2.d.i — Build a trajectory of pose variables from sequential odometry edges (a chain), with an
  anchoring prior on the first pose.
- 2.d.ii — Insert a loop-closure edge (a measured relative transform between two non-adjacent
  poses) and re-optimize to redistribute accumulated drift.
- 2.d.iii — **Outlier-robust loop closures:** χ²/robust-kernel gating (§2.b.iv/§2.b.vi) so a
  single wrong closure does not corrupt the global solution.
- 2.d.iv — 2-D (SE(2)) and 3-D (SE(3)) pose graphs share the engine; both are exercised on
  standard synthetic benchmark topologies (e.g. sphere/torus/Manhattan-style loops).
- Acceptance: on a synthetic loop with injected odometry drift and a correct closure, optimized
  poses return to ground truth within tolerance; an injected bad closure is rejected by the robust
  kernel without destabilizing the solve.

### 2.e Odometry & incremental estimation
Definition: keeping the estimate **online** — folding new measurements into the graph as they
arrive and bounding the per-update cost so it does not grow with trajectory length. Models the
incremental-smoothing idea (iSAM-style) and fixed-lag smoothing (research §B.3 [scope]).
- 2.e.i — Accumulate a relative-motion measurement (wheel/visual/inertial odometry, or a §2.f scan
  match) as a new pose + between-factor in one update step.
- 2.e.ii — **Fixed-lag smoothing:** maintain only the last *N* poses in the active window;
  **marginalize** older variables into a summarizing prior (Schur complement) so the active graph
  size is bounded.
- 2.e.iii — Incremental re-solve that re-linearizes/relinearization-thresholds only the affected
  sub-graph rather than the whole graph each step (bounded online latency).
- 2.e.iv — Emit the **`map→odom` correction** (family §2.g.ii): the current best estimate minus
  the dead-reckoned odometry, as an SE(3) transform a consumer publishes into the `transform`
  frame tree (§1.c.i).
- Acceptance: a fixed-lag run over a recorded measurement stream produces (within tolerance) the
  same windowed estimate as a from-scratch batch solve over that window; marginalization conserves
  information (no double-counting).

### 2.f Scan matching / ICP
Definition: register a **source** point cloud to a **target** (a prior scan or a map) and return
the relative SE(3) transform plus a covariance — the geometric front-end that manufactures
between-factors for §2.d/§2.e. Models Iterative Closest Point (research §B.3 [scope]).
- 2.f.i — **Point-to-point ICP** (Besl-McKay): nearest-neighbor correspondence (kd-tree /
  spatial hash) → closed-form SE(3) alignment of the correspondence set (SVD/quaternion solution)
  → iterate to convergence.
- 2.f.ii — **Point-to-plane ICP** (target surface normals; linearized SE(3) least-squares per
  iteration) for faster convergence on structured scenes.
- 2.f.iii — Correspondence rejection (max-distance / normal-compatibility / trimmed-ICP) for
  partial overlap and outliers.
- 2.f.iv — Convergence + quality reporting: fitness (inlier RMS), overlap ratio, and an estimated
  covariance of the resulting transform (feeding the factor's information matrix).
- 2.f.v — Initial-guess seeding from odometry / the §2.d graph so ICP starts inside its basin of
  convergence.
- Acceptance: aligning a cloud to a known-rotated/translated copy of itself recovers the inverse
  transform to tolerance; with partial overlap + injected outliers, trimmed/rejected ICP still
  converges to the correct pose. **[scope]** — ICP variant specifics re-research before the plan.

### 2.g Particle-filter localization (MCL / AMCL)
Definition: estimate a pose against a **known map** as a set of weighted samples (particles)
approximating the posterior — Monte-Carlo Localization, with adaptive sample count (AMCL).
Models AMCL (research §B.3 [scope]).
- 2.g.i — **Motion update (predict):** propagate each particle through a probabilistic motion
  model driven by odometry (sampled noise on the relative SE(2)/SE(3) increment).
- 2.g.ii — **Measurement update (correct):** re-weight particles by a sensor model — a
  **likelihood-field** or **beam** range model for laser scans against an occupancy map (the
  measurement-model interface is pluggable so non-laser sensors can supply a weight).
- 2.g.iii — **Resampling:** low-variance / systematic resampling triggered by effective-sample-
  size collapse; optional random-particle injection for recovery from localization failure
  (kidnapped robot).
- 2.g.iv — **Adaptive sample size (KLD-sampling):** grow/shrink the particle count to bound the
  KL error between the sample-based and true posterior (the "A" in AMCL).
- 2.g.v — Extract a point estimate + covariance (weighted mean on the manifold / dominant cluster)
  and emit it as the localization correction (as §2.e.iv).
- Acceptance: on a synthetic map with a recorded odometry+range stream, the filter converges from
  a dispersed prior to the ground-truth track and recovers after an injected kidnap; results are
  **seed-deterministic** given a fixed RNG seed. **[scope]** — beam/likelihood-field and KLD
  parameters re-research before the plan.

### 2.h Graph & measurement I/O (test substrate)
Definition: load/save graphs and recorded measurement streams so the whole layer is exercised
against fixtures with **no hardware** (math-layer testing pillar, family §3.b).
- 2.h.i — Read/write a pose-graph in a standard interchange text form (g2o `VERTEX_SE3`/`EDGE_SE3`
  / `VERTEX_SE2`/`EDGE_SE2`) for round-tripping public benchmark datasets.
- 2.h.ii — Replay a recorded measurement stream (odometry + scans/ranges) deterministically into a
  graph/filter for regression tests.
- 2.h.iii — Serialize an optimized trajectory + per-variable marginal covariance for downstream
  consumers (`log`) and for golden-file comparison in CI.
- Acceptance: a g2o benchmark file round-trips byte-for-byte through read→write; a replayed stream
  yields a bit-stable (within tolerance) estimate across runs.

---

## 3. Acceptance criteria (beyond per-feature tests)
- 3.a — **Convention conformance:** every variable/factor uses `transform`'s `⊞`/`⊟`, twist
  ordering, and quaternion convention (§1.b); a test asserts increments and residuals live in the
  translation-first tangent basis at the API boundary.
- 3.b — **Hardware-free, math-layer testing (family §3.b):** the entire suite runs in CI with no
  devices, on x86-64 and ≥1 RISC-V target, using **identity, round-trip, and finite-difference**
  checks — `exp(log)` round-trips for variables (§2.c), analytic-vs-numeric Jacobians for factors
  (§2.a), and known-optimum convergence for the optimizer (§2.b), plus self-alignment for ICP
  (§2.f) and seed-deterministic convergence for the particle filter (§2.g).
- 3.c — **Sparse-solver correctness:** GN/LM solutions and recovered marginal covariances match a
  dense reference on small problems; fill-reducing ordering keeps the Cholesky factor sparse on
  benchmark topologies.
- 3.d — **Robustness:** a single outlier loop closure is rejected by the robust kernel without
  diverging; singular/indeterminate graphs return a recoverable result, never a crash.
- 3.e — **No upward dependencies:** imports only `transform` and Cajeta `numpy`/`linalg`; nothing
  else in `dev.cajeta.robotica.*`. No native/heavy deps (board-agnostic core, family §3.c) — the
  layer is pure linalg.
- 3.f — **Bounded online cost & ownership:** fixed-lag/incremental updates keep per-step cost
  bounded by the window, and graphs/filters release deterministically (no hidden global state, no
  GC pause on the online path).
- 3.g — **Complementary, not duplicative:** `estimation` emits a `map→odom`-style correction and
  consumes/produces transforms, but contains **no** buffered-interpolation frame tree (that stays
  in `transform`, §1.c.i).

## 4. Deliverables
- 4.a — `dev.cajeta.robotica.estimation` implementing §2, with the §3 test suite (fixtures:
  synthetic graphs, g2o benchmark files, recorded odometry+scan streams).
- 4.b — A reference note mapping each factor/variable type to its `transform` retraction and
  Jacobian, so downstream packages add custom factors consistently with §1.b.
- 4.c — A plan at **`agents/estimation-plan.md`** (authored when this spec is approved),
  TDD-ordered along the dependency spine: factor-graph + manifold variables (§2.a/§2.c) → sparse
  NLS optimizer (§2.b) → pose-graph optimization (§2.d) → odometry/incremental (§2.e) → scan
  matching/ICP (§2.f) → particle-filter localization (§2.g), with graph/measurement I/O (§2.h)
  landed early as the test substrate.

## 5. References (research-grounded)
- **GTSAM** — factor graphs; MAP = product of factors → **sparse Cholesky/QR**; SE(2)/SE(3)
  exp-maps; backbone for pose-graph SLAM / VIO / calibration. Research §B.3; gtsam.org/tutorials/intro.html. **[verified]**
- **SE(3)/SO(3) Lie foundation** — exp/log/adjoint/Jacobians and the `⊞`/`⊟` retraction this
  layer optimizes over. Research §B.1 (arXiv:1812.01537; Sophus); consumed via `transform`. **[verified]**
- **g2o / Ceres** — alternative factor-graph / sparse-NLLS engines (Gauss-Newton/LM, robust
  kernels, fill-reducing ordering); g2o file format as the benchmark interchange. Research §B.3. **[scope]**
- **ICP / scan matching** — point-to-point (Besl-McKay) and point-to-plane registration as the
  graph front-end producing relative-pose constraints. Research §B.3. **[scope]**
- **AMCL / Monte-Carlo Localization** — particle filter with motion + likelihood-field/beam
  measurement models, low-variance resampling, KLD-adaptive sample size. Research §B.3. **[scope]**
- **Incremental smoothing (iSAM-style) / fixed-lag smoothing & marginalization** — bounded-cost
  online re-solve; structure noted for the §2.e online path. Research §B.3 (connective). **[scope]**

> **[scope] note:** per research §B.3 and the gap list §J, only the **GTSAM factor-graph /
> sparse-solve backbone** survived the verification pass. ICP variants, AMCL beam/likelihood-field
> and KLD parameters, g2o/Ceres engine specifics, and incremental-smoothing internals are
> **planning-grade** here — re-research against primary sources before writing `agents/estimation-plan.md`,
> rather than inventing false precision in this spec.
