# `dev.cajeta.robotica.ai` — Specification

The **embodied-AI inference edge** of cajeta-robotica: it runs a *learned policy*
(vision + proprioception → action) **on-device** on Cajeta's own GPU/tensor stack, loads
policies from the LeRobot / VLA ecosystem, and emits the resulting actions into `control`.
This is the package that cashes the family's headline payoff — **one language and one type
system from the matmul kernel that runs the policy down to the FOC setpoint on the wire**,
with no Python↔C++↔firmware seam (research Part 2 §I). It is a **heavy, forward-looking
EDGE** package: GPU-bound and board-specific by nature, it sits at the rim of the layer map
so importing it never taxes a deployment that only wants `transform` + a fusion filter.
See the family spec [`../robotica-spec.md`](../robotica-spec.md) §2.o and the research
[`../../research/robotics-stack-part2-perception-to-ai.md`](../../research/robotics-stack-part2-perception-to-ai.md) §I.

---

## 1. Definition

`ai` is an **on-device policy-inference runtime**. Given a trained policy artifact and a
stream of observations, it assembles the model's input, executes the forward pass on
Cajeta's GPU/tensor substrate, decodes the output into a typed action expressed in the
family's conventions, and hands that action to `control`. It is **inference only** — it does
not train, fine-tune, capture sensors, or close a control loop. Its differentiator is that
the *whole* path (image preprocessing → matmul kernels → action denormalization → setpoint)
lives in one type system, so there is no serialization tax across the inference→control seam.

### 1.a Capability summary
- 1.a.i — Load a trained policy artifact (weights + config + dataset statistics) from the
  LeRobot / VLA ecosystem into Cajeta tensors.
- 1.a.ii — Assemble a model observation from time-aligned vision tensors, a proprioceptive
  state vector, and (for VLA) a language instruction.
- 1.a.iii — Execute the policy forward pass on Cajeta's GPU/tensor stack, including
  action-chunking / temporal-ensembling policies and an async inference cadence.
- 1.a.iv — Decode and denormalize the output into a typed action (joint position/velocity/
  torque, or an end-effector twist/pose) in `transform`/`control` units, and emit it into
  `control`.
- 1.a.v — Run reproducibly and offline from a local artifact, with optional inference-I/O
  capture to `log` for replay.

### 1.b Conventions (INHERITED — not redefined here)
> This package **inherits, verbatim and by reference, the normative conventions fixed in
> `transform` §1.b** (Hamilton scalar-last quaternions `[x,y,z,w]`; translation-first
> twist/wrench ordering `[ρ; θ]`; active left-to-right `parent←child` composition; radians /
> meters / SI-seconds; right-handed frames). It defines **no** new geometric or unit
> convention. The two clauses below are *bindings* of those conventions to this package's
> boundaries, not new conventions.
- 1.b.i — **Every action `ai` emits is already in `transform`/`control` units and ordering.**
  An end-effector action is a `transform.Twist` (translation-first) or `SE(3)` pose; a joint
  action is a vector in the joint order `control`/`kinematics` publish. All
  normalize/denormalize happens *inside* `ai` so no downstream package sees model-space units.
- 1.b.ii — **Every observation and action carries a `transform.Time` stamp**; time-alignment
  of multi-camera + proprioception inputs uses `transform`'s timestamp model (the frame-tree
  clock), never wall-clock guesses.

### 1.c Non-goals
- 1.c.i — **No training, fine-tuning, gradient computation, or dataset curation.** `ai`
  consumes a *frozen* artifact. (Backprop, optimizers, and dataset tooling are out of family
  scope; the LeRobot training pipeline stays in Python.)
- 1.c.ii — **No sensor or camera capture** (that is `io.sensor` / `io.vision`); `ai` accepts
  observations as tensors so a deployment can feed it without dragging in a camera stack.
- 1.c.iii — **No control law** (that is `control`); `ai` produces a setpoint/action and hands
  it off — it does not run PID/impedance/WBC.
- 1.c.iv — **No safety reaction.** `ai` exposes a validity/clamp gate and a liveness signal,
  but the watchdog, e-stop, and fault reaction live in `safety` (research Part 2 §F). A
  GPU-bound async inference loop is explicitly *not* the real-time safety path.
- 1.c.v — **No upward dependencies and no claim of real-time determinism for the forward
  pass.** Inference is best-effort throughput; the deterministic guarantee belongs to
  `control` + `safety` below it.

---

## 2. Features

> **Confidence framing.** Per the research, the *existence and shape* of the LeRobot / VLA
> ecosystem rests on **primary sources** and is tagged **[verified]** below; the *internal
> architecture and wire details* of specific policies (graph format, exact tensor layouts,
> chunking constants) did **not** pass the research's adversarial verification and are tagged
> **[scope]** — to be re-researched against the upstream repos before this package's plan
> (research §I, §J). The whole package is **forward-looking** (family spec §2.o).

### 2.a Policy artifact loading
Definition: parse a trained policy distributed in the LeRobot / VLA ecosystem into Cajeta
tensors plus a typed I/O descriptor. **[verified]** LeRobot (HuggingFace) is the emerging
open framework for real-world imitation/RL policy sharing — datasets, pretrained policies,
and a deployment path (research §I).
- 2.a.i (use case) — Load policy **weights** from a local artifact (e.g. a `safetensors`/
  tensor-dump file) into Cajeta GPU/host tensors, offline, with no network access in the hot
  path. **[scope: exact container format to confirm against LeRobot before plan.]**
- 2.a.ii — Parse the policy **config** into a typed *I/O descriptor*: observation keys and
  shapes (image streams + proprioceptive state dim), action dim and semantics, image input
  resolution, and the policy family (e.g. ACT/diffusion/VLA). **[scope]**
- 2.a.iii — Load the **dataset normalization statistics** (per-key mean/std or min/max) that
  the policy was trained against, as first-class tensors used by §2.b/§2.d. **[scope]**
- 2.a.iv — Validate the artifact against the runtime: fail loudly if a required observation
  key, action dim, or stat is missing, *before* any inference runs (no silent shape coercion).
- Acceptance: a recorded artifact loads to a descriptor whose declared shapes match the loaded
  tensors; a deliberately-corrupted/missing-key artifact is rejected at load, not at first tick.

### 2.b Observation assembly
Definition: turn the robot's heterogeneous, asynchronously-sampled inputs into the single
time-consistent tensor bundle the policy expects.
- 2.b.i — Assemble an **observation** from N camera image tensors + a proprioceptive state
  vector (joint positions/velocities and/or end-effector pose in `transform` units) +,
  optionally, a language instruction (§2.e).
- 2.b.ii — **Time-align** the inputs: gather the per-stream samples bracketing the target
  tick and select/interpolate to one `transform.Time`, so all camera frames and the
  proprioceptive vector belong to the same instant (uses the §1.b.ii clock).
- 2.b.iii — **Preprocess images** to the model's input spec: resize/crop to the configured
  resolution and apply the §2.a.iii image normalization — entirely on the GPU/tensor stack,
  no host round-trip.
- 2.b.iv — **Normalize proprioception** with the loaded stats into model space; map joint
  order from the `control`/`kinematics` publication order into the policy's expected order.
- 2.b.v — Accept observations either as **plain Cajeta tensors** (default — keeps `ai`
  camera-free) or, when present, from `io.vision`/`io.sensor` directly (turnkey path).
- Acceptance: identical raw inputs + stats produce a bit-reproducible observation bundle;
  a missing or stale stream is reported (not silently zero-filled).

### 2.c Inference execution
Definition: the forward pass on Cajeta's GPU/tensor substrate — the matmul-kernel end of the
"one language to the FOC setpoint" story (research §I).
- 2.c.i — Run a **single forward pass**: observation bundle → raw action (or action *chunk*),
  executing the policy's compute graph as Cajeta GPU kernels.
- 2.c.ii — **Action chunking / temporal ensembling.** For chunk-emitting policies (ACT-style:
  one inference yields a short horizon of future actions) maintain the chunk buffer and the
  ensembling/blending rule that combines overlapping predictions across ticks. **[scope:
  chunk length and ensembling weights are per-policy; confirm against the policy family.]**
- 2.c.iii — **Decouple inference cadence from control cadence.** Run inference on an async
  worker that publishes the latest action(chunk) into a buffer; `control` consumes at its own
  (higher) rate via §2.d. Expose the measured inference latency and a staleness age.
- 2.c.iv — **Precision selection** (fp32 / fp16 / bf16, and int8 where supported) to fit the
  policy on the target GPU within a latency budget; precision is a load-time choice recorded
  in the descriptor.
- 2.c.v — **Liveness signal.** Publish a heartbeat/last-success timestamp that `safety` can
  watchdog (the hook for §1.c.iv) — `ai` never decides the fault reaction itself.
- Acceptance: with a recorded observation and a fixed artifact + seed + precision, the forward
  pass is deterministic and reproduces a recorded reference action to tolerance.

### 2.d Action decoding & emission into `control`
Definition: convert raw model output into a typed, in-convention action and hand it to the
control layer (family spec §2.o.ii, layer map §1.c).
- 2.d.i — **Denormalize** the model output with the §2.a.iii action stats back into physical
  units, and pack it into the typed action the descriptor declares: a joint vector
  (position/velocity/torque) **or** an end-effector `transform.Twist`/`SE(3)` pose (§1.b.i).
- 2.d.ii — **Rate-adapt** the (possibly chunked, possibly stale) policy output to `control`'s
  tick rate: hold/interpolate within the current chunk and advance the chunk index per
  control tick, so `control` always reads a fresh in-bounds setpoint.
- 2.d.iii — **Validity / clamp gate.** Saturate the action to the configured joint/twist
  limits and flag out-of-range or stale-beyond-horizon output as *invalid*, surfacing the
  flag to `control`/`safety` rather than emitting a wild setpoint. (The *reaction* to an
  invalid action is `safety`'s, per §1.c.iv.)
- 2.d.iv — **Emit** the validated action into the `control` package's command interface; `ai`
  depends *downward* on `control`, never the reverse.
- Acceptance: a known model output denormalizes to the expected physical action within
  tolerance; an out-of-limit output is clamped and flagged, never passed through raw.

### 2.e Vision-Language-Action (VLA) conditioning
Definition: the language-conditioned policy path. **[verified]** VLA models map
camera + instruction → actions; **SmolVLA** is a compact, consumer-hardware-targeted VLA, and
the space is moving fast (research §I; arXiv:2511.05642). **[scope]** the concrete tokenizer,
context format, and model internals are fast-moving and unverified this pass — re-research
against the upstream model card before the plan.
- 2.e.i — Accept a **natural-language instruction** as part of the observation (§2.b.i) and
  condition the policy on it (tokenize / embed per the model's contract). **[scope]**
- 2.e.ii — Target the **compact, consumer-hardware** regime (SmolVLA-class) so the forward
  pass fits the kind of GPU an edge robot actually carries — the precision/latency knobs of
  §2.c.iv are the levers. **[verified existence; scope on the sizing details.]**
- 2.e.iii — Treat the VLA architecture surface as **versioned and replaceable**: the I/O
  descriptor (§2.a.ii) is the stable contract; a new VLA generation is a new descriptor +
  weights, not a `control`-facing API change. **[scope]**

### 2.f Deployment, provenance & reproducibility
Definition: the lifecycle guarantees that make on-device inference trustworthy and debuggable.
- 2.f.i — **Offline-first:** a policy runs from a local artifact with no network dependency in
  the hot path; any fetch is an explicit, out-of-loop deploy step.
- 2.f.ii — **Provenance:** record the artifact identity (hash/version), precision, and stats
  set actually loaded, so a logged run is traceable to exact weights.
- 2.f.iii — **Inference-I/O capture (optional):** record observation→action pairs (with
  timestamps and the provenance of §2.f.ii) to `log` (MCAP) for offline replay and
  regression. Dependency on `log` is optional/sparing, never required to run.
- 2.f.iv — **Replay/regression:** a recorded observation stream re-fed through the same
  artifact reproduces the recorded action stream to tolerance — the package's hardware-free
  test mode.

---

## 3. Acceptance criteria (beyond per-feature tests)
- 3.a — **Hardware-free regression where it can be.** Although the forward pass needs a GPU,
  the package's *contract* tests run without a robot: artifact-load/validation (§2.a),
  observation assembly + stat normalization round-trips (§2.b), action denormalize/clamp
  (§2.d), and recorded-stream replay (§2.f.iv) all run against recorded tensors. The
  GPU-dependent forward pass is exercised separately on a GPU target.
- 3.b — **Convention conformance:** every emitted action is asserted to be in `transform`/
  `control` units and ordering (§1.b.i), and every observation/action carries a
  `transform.Time` (§1.b.ii) — no model-space units cross the package boundary.
- 3.c — **Determinism on a fixed artifact:** given fixed weights + seed + precision, inference
  reproduces a reference action to tolerance (§2.c Acceptance), so regressions are detectable.
- 3.d — **Edge-isolation:** `ai`'s heavy/GPU dependencies are confined to this package;
  importing `transform`, `fusion`, or `control` must not transitively pull in the inference/
  GPU stack (family pillar §3.c, §1.b.ii of the family spec). `ai` depends only **downward**
  (on `transform`, on `control`, on Cajeta's GPU/tensor/numpy substrate; optionally on
  `io.vision`/`log`), **never upward**.
- 3.e — **Fail-loud, fail-safe boundaries:** missing keys/stats fail at load (§2.a.iv); stale
  or out-of-limit actions are flagged and clamped, never emitted raw (§2.d.iii); and the
  liveness signal (§2.c.v) is always published for `safety` to watchdog.

## 4. Deliverables
- 4.a — `dev.cajeta.robotica.ai` implementing §2, with the §3 test suite (recorded-tensor
  contract tests + a GPU-target forward-pass test).
- 4.b — A small set of **recorded reference fixtures** (artifact descriptor + observation
  stream + expected action stream) that drive the hardware-free replay/regression tests
  (§2.f.iv), analogous to the family's "protocol-codec tests against recorded byte streams"
  pillar applied to learned-policy I/O.
- 4.c — A plan at `agents/ai-plan.md` (authored when this spec is approved), dependency-ordered
  TDD: artifact loading + I/O descriptor → observation assembly/normalization → action
  decode/clamp/emit (contract-testable, no GPU) → forward-pass execution on the GPU stack →
  action chunking/async cadence → VLA conditioning. Each unit is one commit with TDD/Coding/
  Acceptance line items.

## 5. References (research-grounded)
- **LeRobot** (HuggingFace) — open imitation/RL framework: datasets, pretrained policies,
  deployment path. Source: github.com/huggingface/lerobot (research §I). **[verified]**
- **SmolVLA** — compact, consumer-hardware-targeted Vision-Language-Action model. Source:
  huggingface.co/blog/smolvla (research §I). **[verified]**
- **VLA survey** — vision-language-action models map camera + instruction → actions; the field
  is moving fast. Source: arXiv:2511.05642 (research §I). **[verified existence; scope on
  internals]**
- **"One language from matmul kernel to FOC setpoint"** payoff — the single-type-system,
  GPU-to-actuator story that motivates this package. Source: research Part 2 §I (synthesis).
  **[verified-synthesis]**
- **Policy artifact format, tensor layouts, chunking constants, VLA tokenizer/context** — the
  internal wire/architecture details. Research §I/§J explicitly did **not** verify these;
  re-research against the upstream LeRobot/SmolVLA repos before the plan. **[scope]**
- **Inherited conventions** — `transform` §1.b (quaternion/twist/units/composition). The
  normative source this package binds to (§1.b above). **[verified — family foundation]**
