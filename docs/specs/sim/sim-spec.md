# `dev.cajeta.robotica.sim` — Specification

The **simulation-interface** edge of cajeta-robotica. It parses robot **models** (MuJoCo
**MJCF** first; URDF/SDF/USD as relatives) and **drives/observes a simulator**, exposing
simulated sensors and actuators through the **same `io.sensor` / command interfaces** the
real-hardware drivers implement — so a controller cannot tell sim from real (sim/real
parity). It is a **heavy-dependency EDGE package**: the native simulator binding lives
*only* here, never on the sense→estimate→plan→control→actuate spine. See the family spec
[`../robotica-spec.md`](../robotica-spec.md) §2.l and the research
[`../../research/robotics-stack-part2-perception-to-ai.md`](../../research/robotics-stack-part2-perception-to-ai.md) §E.2.

---

## 1. Definition

`sim` is the bridge between a robot description and a running physics engine. It has two
cleanly separable halves: (1) a **pure, native-free model codec** that parses MJCF (and,
scoped, URDF/SDF/USD) into an in-memory model the family understands — a `transform` frame
tree plus joint/geom/actuator/sensor tables; and (2) a **backend bridge** that loads that
model into a simulator (MuJoCo first), steps it, applies controls, and reads state back —
publishing simulated measurements through the identical `io.sensor` interfaces and consuming
commands through the identical actuator/command interfaces used on real hardware. The codec
half is fully unit-testable without a simulator or hardware; the backend half is verified
against **recorded golden state traces** (the sim analogue of the family's "protocol-codec
tests against recorded byte streams" pillar).

### 1.a Capability summary
- 1.a.i — Parse an **MJCF** XML model (`<worldbody>` body/joint/geom/site/actuator/sensor
  tree, `<default>` inheritance, `<compiler>` options) into a family model IR.
- 1.a.ii — Build a time-stamped `transform` **frame tree** from the model's body hierarchy
  and joint kinematics, so simulated frames are queryable exactly like real ones.
- 1.a.iii — **Load → step → observe** a simulator through an abstract `SimBackend`
  (concrete: a native MuJoCo binding), with a fixed-timestep, deterministic stepping loop.
- 1.a.iv — Apply controls/torques to simulated **actuators** through the same command
  interfaces real drives use (`motor` / `control` setpoints), and read simulated
  **sensors** through the same `io.sensor` interfaces real sensors use — **sim/real parity**.
- 1.a.v — **Record and replay** golden trajectories (seed + model + control stream → exact
  state trace) for regression and for hardware-free CI of downstream packages.
- 1.a.vi — *(scope)* Ingest **URDF / SDF / USD** models into the same model IR.

### 1.b Conventions (INHERITED — not redefined here)
> This package **inherits, by reference, every normative convention fixed in
> `transform` §1.b** — Hamilton quaternions, scalar-last `[x,y,z,w]` storage; translation-first
> twist `[ρ; θ]`; active, parent←child left-composed transforms; radians/meters/SI-seconds;
> right-handed frames. `sim` does **not** redefine any of them. It only specifies the
> **codec boundary** where an external format disagrees, and converts *at parse/marshal time*
> so that everything the rest of the family sees is already in `transform`'s convention.
- 1.b.i — **Quaternion-storage boundary [verified hazard].** MJCF stores orientation
  quaternions **scalar-first `(w, x, y, z)`** and MuJoCo's runtime `qpos`/sensor quaternions
  are likewise scalar-first; `transform` is scalar-last `[x,y,z,w]`. The codec and the
  backend marshaller **reorder at the boundary** (and re-canonicalize sign, `w ≥ 0`); no
  scalar-first quaternion escapes this package. This is exactly the convention-mismatch bug
  class the research flags (Part 2 §A.1, §B.1) — contained, never propagated.
- 1.b.ii — **Angle/units boundary.** MJCF `<compiler angle="radian|degree">` defaults to
  radians but may be degrees, and `eulerseq` selects Euler order; the codec normalizes all
  angles to **radians** and all Euler/axisangle/xyaxes/zaxis orientation specifiers to a
  `transform` SO(3) at parse time. Lengths are meters, time SI seconds (MuJoCo's defaults
  already match).
- 1.b.iii — **Coordinate boundary.** MJCF body poses default to **`coordinate="local"`**
  (child-relative); the codec composes them into the `transform` parent←child frame tree.
  Gravity/up-axis is read from the model, not assumed.
- 1.b.iv — **Time boundary.** Simulated `Time` originates from the backend's accumulated
  `model.opt.timestep` (a monotonic sim clock), surfaced as `transform.Time`; it is the
  stamp on every published simulated measurement so downstream fusion/estimation treats sim
  and real timestamps identically.

### 1.c Non-goals
- 1.c.i — **Not a physics engine.** `sim` does not reimplement a contact solver, integrator,
  or collision pipeline; it *binds* an existing simulator. A native Cajeta physics engine is
  explicitly out of scope (research §E.2: "a native sim is a much larger effort").
- 1.c.ii — **Not the kinematics/dynamics math.** FK/IK, Jacobians, and RBD live in
  `kinematics` / `dynamics`. `sim`'s model IR may *feed* those packages, but does not compute
  them. (See open question 5.a on where the shared model IR lives.)
- 1.c.iii — **No rendering / GUI.** Visualization is `log` (MCAP/Foxglove) and `io.vision`;
  `sim` may expose simulated camera buffers through `io.vision`'s interface but renders
  nothing itself.
- 1.c.iv — **No training loop.** RL/imitation training and policy inference are `ai`; `sim`
  only provides the steppable environment they drive.
- 1.c.v — **No upward dependencies.** `sim` depends on `transform` (and, for the parity
  interfaces it implements, on the *interface* surfaces of `io.sensor` and `motor`/`control`);
  nothing in the spine depends on `sim`.

---

## 2. Features

### 2.a MJCF model codec [primary — research §E.2]
Definition: a pure, native-free reader (and minimal writer) for MuJoCo's MJCF XML, producing
the family model IR. MuJoCo is **Apache-2.0** and MJCF is fully documented (bodies / joints /
geoms / sites / actuators / sensors tree).
- 2.a.i (use case) — Parse `<worldbody>` into a body tree: each `<body>` with its pose
  (`pos` + one of `quat`/`euler`/`axisangle`/`xyaxes`/`zaxis`), `<inertial>`, child bodies,
  `<geom>`s, and `<site>`s.
- 2.a.ii — Parse the **joint** set per body — `free` / `ball` / `slide` / `hinge` — with axis,
  range, and the `qpos`/`qvel` address layout each implies (a `free` joint = 7 `qpos` /
  6 `qvel`; a `hinge` = 1/1), recording the **index map** the backend will use.
- 2.a.iii — Resolve **`<default>` class inheritance** (nested defaults, `childclass`) so
  every element's effective attributes are materialized.
- 2.a.iv — Parse `<actuator>` entries (`motor`/`position`/`velocity`/general) with their
  `gear`, control range, and the joint/site they drive — the targets §2.d commands.
- 2.a.v — Parse `<sensor>` entries (e.g. `accelerometer`, `gyro`, `velocimeter`,
  `framepos`/`framequat`, `force`/`torque`, `jointpos`/`jointvel`, `rangefinder`,
  `touch`) with their `site`/object attachment and their slot in the flat `sensordata`
  vector — the source map §2.c publishes.
- 2.a.vi — Read `<compiler>` (`angle`, `eulerseq`, `coordinate`, `meshdir`/`assetdir`) and
  `<option>` (`timestep`, `gravity`, integrator) and apply the §1.b boundary normalizations.
- 2.a.vii — Resolve `<include>` and report `<asset>` references (meshes/heightfields) as
  named, lazily-loaded resources (not parsed geometry — that is the backend's job).
- 2.a.viii — Surface **diagnostics** with source line/column on malformed or unsupported
  constructs, and a clearly-enumerated **unsupported-element list** rather than silent drop.
- Acceptance: round-trip a curated fixture corpus (a humanoid, an arm, a quadruped from the
  MuJoCo Menagerie-style set) → IR → frame tree with **no hardware and no simulator**;
  body poses, joint index maps, and quaternion reordering (§1.b.i) checked against
  hand-computed expected values.

### 2.b Backend bridge — load / step / observe [primary; native edge]
Definition: an abstract `SimBackend` interface and a concrete native MuJoCo binding. This is
the **only native dependency** in the package and is confined behind the interface.
- 2.b.i — `load(model)` — instantiate the parsed IR in the backend, returning a handle plus
  the resolved `qpos`/`qvel`/`ctrl`/`sensordata` address tables (validated against §2.a's
  predicted layout).
- 2.b.ii — `step(n = 1)` — advance the simulation by `n` fixed `timestep`s; **deterministic**
  for a fixed model + seed + control stream (no wall-clock coupling).
- 2.b.iii — `setState` / `getState` — read and write `qpos`/`qvel` (and `act` for stateful
  actuators) as `transform`-convention values; reset to a named keyframe.
- 2.b.iv — `setControl(ctrl)` / `applyExternal(wrench)` — write the actuator control vector
  and optional external body wrenches (cartesian perturbations) for the tick.
- 2.b.v — `readSensors()` — pull the flat `sensordata` vector and the body/site kinematic
  state for §2.c to demultiplex.
- 2.b.vi — A `SimClock` driving §1.b.iv, and an optional real-time-factor pacing mode
  (free-run for CI, paced for human-in-the-loop).
- 2.b.vii — Backend capability/version query so a model using features the linked backend
  lacks fails **loudly at load**, not mid-run.
- Acceptance: a recorded **golden trajectory** test — `(model, seed, control stream)` →
  exact `qpos`/`qvel`/`sensordata` trace — replays bit-stable (or within a documented
  tolerance) on the pinned backend version; the abstract interface is exercised by a
  **deterministic stub backend** so downstream packages test against `sim` with **no native
  MuJoCo present**.

### 2.c Simulated sensors via `io.sensor` (sim/real parity)
Definition: each MJCF `<sensor>` (and each derivable kinematic quantity) is published through
the **identical `io.sensor` interface** a real driver implements, so a fusion/estimation
consumer is byte-for-byte agnostic to the source.
- 2.c.i — Map `accelerometer` + `gyro` (+ `magnetometer`, if modeled) at a site to the
  **IMU** interface `io.sensor` defines (the same one a BNO08x/MPU driver implements),
  stamped on the `SimClock`.
- 2.c.ii — Map `jointpos`/`jointvel` (and `framequat`/`framepos`) to the **encoder** /
  pose interfaces (quadrature/absolute-encoder parity).
- 2.c.iii — Map `force`/`torque` at a site to the **6-axis F/T** wrench interface (already a
  `transform.Wrench`, §1.b twist-dual convention).
- 2.c.iv — Map `rangefinder`/`touch` to the rangefinder/contact interfaces.
- 2.c.v — *(scope)* Surface a simulated camera/depth buffer through the `io.vision`
  interface, **if** the linked backend provides offscreen rendering (else report unsupported,
  §2.b.vii) — flagged because rendering support and pixel/format fidelity are backend- and
  build-dependent, not guaranteed.
- 2.c.vi — Optional **measurement-noise / bias / latency injection** so simulated sensors can
  emulate real-sensor imperfection (configurable; default = ideal/noiseless) — the bridge that
  makes sim a useful estimator test rig, not just a kinematics echo.
- Acceptance: a **parity test** — the same downstream fusion/estimation code, run once against
  a real-driver recording and once against a sim publisher of the same nominal motion,
  consumes both through one interface with no source-specific branches; sim output validated
  against the analytic value (e.g. an `accelerometer` at rest reads `-g` in body frame).

### 2.d Simulated actuators / command sink (sim/real parity)
Definition: the inbound mirror of §2.c — controllers issue commands through the **same
`motor`/`control` command interfaces** real drives implement, and `sim` marshals them onto
the backend `ctrl` vector.
- 2.d.i — Accept a position/velocity/torque setpoint through the family command interface and
  route it to the matching MJCF `<actuator>` via §2.a.iv's target map, applying `gear` and
  control-range clamps.
- 2.d.ii — Accept a **sync/group write** (the bus-servo idiom: many joints in one
  transaction) and apply it atomically before the next `step`.
- 2.d.iii — Report actuator feedback (commanded vs. realized force, saturation flags) back
  through the same feedback channel a real drive exposes.
- 2.d.iv — Honor command staleness/timeout the way a real watchdog-guarded drive would
  (a missed command leaves the actuator in its defined safe/hold state) — the hook `safety`
  exercises in sim.
- Acceptance: a closed-loop test — a `control` PID/impedance loop drives a simulated joint to
  a setpoint through the command interface with **no sim-specific code in the controller**;
  identical controller binary would drive a real `motor` driver.

### 2.e Determinism, recording & replay
Definition: the reproducibility substrate that makes `sim` a CI fixture generator for the
whole family.
- 2.e.i — Single **seed** governs all stochasticity (noise injection, any randomized
  reset/domain-randomization); same seed + inputs ⇒ same trace.
- 2.e.ii — Record a run as a portable trace — `(model hash, seed, control stream, state +
  sensordata per tick)` — and replay it without the backend (pure data) for downstream
  regression.
- 2.e.iii — Emit the trace as a `log` (MCAP) channel set so a sim run is inspectable in
  Foxglove with the same tooling as a real run (depends only on `log`'s writer interface,
  not its internals).
- 2.e.iv — **Domain randomization** hooks (mass/friction/sensor-noise sampling from the seed)
  for sim-to-real robustness studies — *(scope: API shape only; the randomization policy
  library is `ai`/research territory, not specified here).*
- Acceptance: a golden replay run is bit-stable across two processes on the same backend
  version; a recorded trace replays with no native dependency.

### 2.f Model-format relatives — URDF / SDF / USD [scope]
Definition: additional model front-ends feeding the **same** model IR (§2.a). The research
ranks these below MJCF and notes the detail is unverified (Part 2 §E.3 scope note, §J gap
list: "sim formats (MJCF/USD detail); URDF/SRDF parsing").
- 2.f.i — *(scope)* Parse **URDF** (link/joint tree, `<visual>`/`<collision>`/`<inertial>`)
  into the IR; **URDF first** among the relatives. Coordinate with `kinematics` §2.h.ii,
  which also consumes URDF — shared parser vs. duplicated is open question 5.a.
- 2.f.ii — *(scope)* Parse **SDF** (Gazebo) — world + model + nested models.
- 2.f.iii — *(scope)* Ingest **OpenUSD** stages (Isaac-Sim-style) — flagged lowest-confidence;
  USD is a large external dependency and its robotics schema is still settling.
- 2.f.iv — A format-agnostic **IR conformance** layer so a model loaded from any front-end
  drives the §2.b backend identically.
- Acceptance (when promoted from scope): a URDF and an equivalent MJCF of the same robot
  produce frame trees that agree to tolerance on every body pose.

---

## 3. Acceptance criteria (beyond per-feature tests)
- 3.a — **Codec is native-free.** The MJCF parser, IR, frame-tree build, and trace replay
  (§2.a, §2.e.ii) compile and test with **no MuJoCo and no hardware**, on x86-64 and at least
  one RISC-V target — the same hardware-free bar the math layers meet.
- 3.b — **Native dependency is contained.** Only the §2.b MuJoCo binding links libmujoco;
  it sits behind `SimBackend`, and a deterministic stub backend lets every downstream package
  test against `sim` without it. No `sim` symbol leaks the native dep onto the spine.
- 3.c — **Convention conformance at the boundary.** Tests assert §1.b.i–iv: no scalar-first
  quaternion, degree angle, or local-coordinate pose escapes the package; everything emitted
  is in `transform` convention.
- 3.d — **Sim/real parity is structural.** A downstream consumer compiles against the
  `io.sensor` / `motor` / `control` interfaces with **zero source branches** distinguishing
  sim from real (verified by §2.c / §2.d parity tests sharing one consumer binary).
- 3.e — **Determinism.** Golden-trajectory replay is reproducible across processes on the
  pinned backend version; all stochasticity flows from one seed (§2.e.i).
- 3.f — **No upward dependencies.** `sim` imports only `transform` and the *interface*
  surfaces of `io.sensor` / `motor` / `control` (and, optionally, `log`'s writer); nothing in
  the sense→…→actuate spine imports `sim`.

## 4. Deliverables
- 4.a — `dev.cajeta.robotica.sim` implementing §2, with the §3 test suite (codec + stub-backend
  tests in default CI; native-MuJoCo golden-trace tests gated on a build flag).
- 4.b — A curated **MJCF fixture corpus** + recorded **golden trajectories** committed as test
  assets (the sim analogue of recorded byte streams).
- 4.c — A short **parity contract** doc: the exact `io.sensor` / command interface mapping
  table (MJCF sensor/actuator type → family interface), published so real-driver and sim
  authors implement the same surface.
- 4.d — A plan at `agents/sim-plan.md` (authored when this spec is approved), TDD-ordered:
  MJCF codec → IR + frame-tree build → stub `SimBackend` + determinism/replay → native MuJoCo
  binding → sensor parity (`io.sensor`) → actuator/command sink → *(scope)* URDF relative.

## 5. References (research-grounded)
- MuJoCo **Apache-2.0** + **MJCF** modeling format (bodies/joints/geoms/actuators/sensors
  tree) — research Part 2 §E.2; https://mujoco.readthedocs.io/en/stable/modeling.html.
  **[primary source]**
- "Parse MJCF and speak to a MuJoCo instance; a native sim is a much larger effort." —
  research Part 2 §E.2 (the load-step-observe scope, §1.c.i). **[primary source]**
- **Quaternion-convention hazard** (pin one convention; mismatch is a top source of silent
  orientation bugs) — research Part 2 §A.1 hazard box + §B.1 Sophus/manif note; basis for the
  §1.b.i scalar-first→scalar-last boundary. **[verified]**
- **Gazebo/Ignition (SDF), Isaac Sim (USD), PyBullet** as alternatives; robot-description
  formats **URDF/SRDF/Xacro, MJCF, OpenUSD**, "URDF first" — research Part 2 §E.2 and §E.3
  scope note. **[scope]**
- Sim-format / URDF-SRDF parsing detail listed as an explicit **gap to re-research** before
  this plan — research Part 2 §J. **[scope]**
- `io.sensor` / `motor` parity interfaces this package implements — family spec
  [`../robotica-spec.md`](../robotica-spec.md) §2.c, §2.e, §3.c (board-agnostic core; heavy
  deps confined to `io.vision`/`sim`/`ai`). **[verified — family pillar]**
- `transform` conventions inherited wholesale — [`../transform/transform-spec.md`](../transform/transform-spec.md) §1.b. **[verified]**
