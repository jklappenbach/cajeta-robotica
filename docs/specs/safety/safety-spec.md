# `dev.cajeta.robotica.safety` — Specification

The **safety plane** of cajeta-robotica: a cross-cutting package that owns e-stop chains,
hardware/software watchdogs, deterministic fault reaction, and the power-&-force-limiting
constraints injected into control. It depends only on the `transform` foundation (for
units, `Twist`, `Wrench`, and `Time`/`Duration`) and on the Cajeta language runtime; it
**never** depends upward on `control`, `motor`, or any sense→…→actuate spine package.
Instead it **defines the contracts** those packages consume (a safe-state command, a PFL
constraint set, a deadline/heartbeat port). See the family spec
[`../robotica-spec.md`](../robotica-spec.md) §2.n and the research
[`../../research/robotics-stack-part2-perception-to-ai.md`](../../research/robotics-stack-part2-perception-to-ai.md)
§F (safety plane) plus
[`../../research/robotics-software-landscape.md`](../../research/robotics-software-landscape.md)
§2 (the CiA 402 FAULT/QUICK-STOP states that are the drive-side hook).

---

## 1. Definition

`safety` is the layer that **detects a hazardous condition and drives the machine to a
defined safe state within a bounded latency** — and proves, at compile time, that the
resources held on that fault path are released without a GC pause. It contributes three
kinds of artifact: **monitors** (e-stop aggregators, watchdogs, deadline checkers) that
observe the running system; **reactions** (a deterministic fault state machine that emits a
safe-state command) that act on it; and **constraints** (power-&-force-limiting bounds) that
it hands to `control` to keep the nominal loop inside biomechanical limits. It is pure
coordination logic over `transform` types — fully unit-testable **without hardware** by
driving its monitors with synthetic event/clock streams and asserting the emitted reaction
and latency.

### 1.a Capability summary
- 1.a.i — A **dual-channel e-stop chain**: aggregate N latching/momentary safety inputs into
  one monotonic "demand to stop", with cross-channel discrepancy detection.
- 1.a.ii — **Watchdogs**: a software deadline/heartbeat monitor and a hardware-watchdog
  binding that force the drive to a safe CiA 402 state on a missed deadline.
- 1.a.iii — A **deterministic fault-reaction state machine**: fault class → reaction →
  safe-state command, with a bounded, measured worst-case reaction latency.
- 1.a.iv — **Power-&-force-limiting (PFL) constraints** (ISO/TS 15066 model) expressed in
  `transform` units and injected into `control` as additional QP/limit constraints.
- 1.a.v — The **Cajeta safety differentiator**: ownership/borrow + deterministic drop give
  provable, allocation-free, GC-pause-free resource release on the fault path (§2.f).

### 1.b Conventions (NORMATIVE)
> This package **inherits `transform` §1.b in full and overrides nothing.** Quaternion
> (Hamilton, scalar-last), twist/wrench ordering (translation-first `[ρ; θ]` / `[force;
> torque]`), active left-to-right composition, and SI units (radians, meters, seconds) are
> defined there and used here verbatim. Do **not** redefine them. The conventions added
> below are *safety-local* and do not touch geometry.
- 1.b.i — **Fail-safe / de-energize-to-trip.** The safe state is the *absence* of a
  positive "run permit"; loss of signal, loss of heartbeat, and uninitialized state all
  resolve to **STOP**. A monitor that has never been fed is in demand-to-stop, never "OK".
- 1.b.ii — **Monotonic stop demand.** A latched stop demand never auto-clears; clearing
  requires an explicit, edge-triggered reset (mirrors CiA 402 Fault Reset, rising edge,
  Controlword bit 7). [verified: landscape §2]
- 1.b.iii — **One clock.** Deadlines and latencies are measured against `transform`'s
  **monotonic** `Time` (never wall-clock); all durations are `Duration` (ns base).
- 1.b.iv — **Severity ordering.** Reactions form a total order `NONE < WARN < SLOW
  (limit/derate) < QUICK_STOP < STOP (disable voltage) < STO (safe torque off)`; the
  supervisor always applies the **most severe** demand currently asserted (priority merge,
  never averaged).
- 1.b.v — **No allocation, no fallible call on the fault path** (§2.f acceptance): once a
  fault is latched, reaching the safe-state command must not allocate, lock, or call a
  function that can fail.

### 1.c Non-goals
- 1.c.i — **No certification claim.** A pure-Cajeta library is not a certified safety
  product; `safety` provides *conformance hooks and an auditable structure* (§2.g), not a
  SIL/PL rating. Hardware safety functions (a certified safety PLC, STO wiring, a category-4
  e-stop relay) remain external. [scope: IEC 61508 / ISO 13849]
- 1.c.ii — **No transport.** Talking to a CAN/EtherCAT bus or a safety fieldbus is `io`'s
  job; `safety` emits *commands and frames as values* and consumes *events as values* across
  ports that the driver layers implement.
- 1.c.iii — **No control law.** `safety` emits constraints and a safe-state command; the
  actual QP solve / torque computation stays in `control`, the CiA 402 wire write stays in
  `motor`.
- 1.c.iv — **No motion planning.** Speed-and-separation distances come *in* as a sensed
  scalar; computing them from a point cloud is `io.vision`/`planning` work.

---

## 2. Features

### 2.a E-stop chain (dual-channel aggregation & latch)
Definition: a deterministic combinational+latch network turning many safety inputs into one
stop demand, modeled on a dual-channel (category-3/4) e-stop loop. [scope: ISO 13849
architecture]
- 2.a.i (use case) — Register N inputs, each tagged **momentary** or **latching** and
  **normally-closed** (de-energize-to-trip, §1.b.i); evaluate the aggregate demand.
- 2.a.ii — **Dual-channel discrepancy:** two redundant channels for the same physical button
  must agree within a configured time window; a persistent mismatch is itself a fault
  (channel-fault), not a silent pass.
- 2.a.iii — **Latch + reset:** a latching input, once tripped, holds the stop demand until an
  explicit edge-triggered reset *and* all inputs read safe (§1.b.ii); reset while any input
  is still tripped is rejected.
- 2.a.iv — Compose sub-chains hierarchically (a cell e-stop ⊇ a robot e-stop) so a parent
  trip propagates to every child but not vice-versa.
- Acceptance: pure function of (input vector, clock); no hidden state beyond the documented
  latch; discrepancy and "never-fed ⇒ STOP" behaviors covered by table-driven tests.

### 2.b Watchdogs (software deadline + hardware binding)
Definition: liveness monitors that assert a stop demand when the control loop misses its
deadline or stops feeding the dog. The drive-side safe state is a **CiA 402** transition.
[verified: landscape §2 for the CiA 402 hook; the watchdog policy itself is scope]
- 2.b.i — **Heartbeat/petting watchdog:** the control loop calls `pet()` each cycle; if no
  pet arrives within `timeout` (monotonic clock, §1.b.iii), the dog trips to a configured
  reaction (default `QUICK_STOP`).
- 2.b.ii — **Deadline monitor:** given an expected period (e.g. the ros2_control-style
  read→update→write loop, **default 100 Hz** [verified: landscape §3.1]), flag both *missed*
  and *late* cycles, with a tolerated-jitter band and an overrun counter (N consecutive
  misses ⇒ escalate severity).
- 2.b.iii — **Hardware-watchdog port:** an interface (`HwWatchdog`) the `io` layer binds to a
  real WDT device node; `safety` owns the feed/disarm logic, the board owns the silicon. On
  process death the hardware dog fires independently — the software dog is not the last line.
- 2.b.iv — **Drive-safe-state mapping:** a missed deadline emits a `SafeStateCommand`
  (§2.e) — by default **Quick Stop** (Controlword `xxxxxxxx0xxxx01x`) escalating to
  **Disable Voltage** (`xxxxxxxx0xxxxx0x`) if QUICK STOP ACTIVE is not confirmed within a
  second deadline. [verified: landscape §2]
- Acceptance: driven by a synthetic monotonic clock; asserts trip at exactly `timeout`,
  correct reaction emitted, and idempotent re-trip; no wall-clock dependence.

### 2.c Deterministic fault-reaction state machine
Definition: the supervisor mapping a classified fault to a reaction and a safe-state command,
with a **bounded, measured** worst-case latency — the software analogue of the CiA 402
FAULT-REACTION-ACTIVE → FAULT path. [verified: landscape §2 for the drive states; reaction
policy is scope]
- 2.c.i — A **reaction table**: fault class (deadline-miss, e-stop, channel-fault,
  range/limit violation, sensor-stale, over-force) → reaction severity (§1.b.iv). The table
  is data, auditable, and exhaustively matched (no default-fallthrough to "ignore").
- 2.c.ii — **Priority merge:** with several faults asserted, the supervisor commands the
  single most-severe reaction (§1.b.iv), never an average; de-escalation requires every
  contributing fault to clear *and* an explicit reset.
- 2.c.iii — **Bounded reaction latency:** the path from `fault_latched` to
  `SafeStateCommand emitted` is straight-line, allocation-free code (§2.f) whose worst-case
  cycle count is asserted by a test, giving a deterministic latency claim at a stated tick
  rate.
- 2.c.iv — Mirror the CiA 402 states the drive will actually traverse (OPERATION ENABLED →
  QUICK STOP ACTIVE → FAULT REACTION ACTIVE → FAULT → SWITCH ON DISABLED) so the supervisor's
  view and the drive's statusword (`0x6041`) stay reconciled; a statusword that doesn't reach
  the expected safe state within deadline is itself escalated. [verified: landscape §2]
- Acceptance: deterministic transition table tested as a pure FSA; worst-case-path test
  pins the latency bound; reconciliation-timeout path covered.

### 2.d Power-&-force-limiting (PFL) constraints
Definition: ISO/TS 15066-style collaborative limits expressed as `transform` `Twist`/`Wrench`
bounds and handed to `control` as additional constraints. [scope: ISO/TS 15066 — paywalled,
planning-grade]
- 2.d.i — **Force/pressure cap:** a per-body-region maximum contact `Wrench` (force/torque,
  §1.b inherited ordering); `safety` exposes it as a constraint object `control` adds to its
  QP / clamps its commanded wrench against. The biomechanical limit *values* are a
  configurable table (a `[scope]` placeholder pending the standard's Annex A numbers).
- 2.d.ii — **Speed limit / derate:** a maximum Cartesian or joint `Twist` magnitude, optionally
  a function of a sensed human-separation distance (the **SSM**, speed-and-separation-
  monitoring mode) — distance in, speed-scale out; distance is sensed elsewhere (§1.c.iv).
  [scope: SSM is the 15066 companion mode, not verified this pass]
- 2.d.iii — **Energy/momentum cap:** a transferred-energy or momentum bound (the PFL
  transient-vs-quasi-static distinction) expressed over mass + `Twist`; emitted as a
  derate factor when approached.
- 2.d.iv — **Safety zones:** named workspace volumes (in a `transform` frame) each carrying a
  speed/force policy; entering a zone swaps the active constraint set. Zone membership is a
  geometric predicate over a pose from the frame tree.
- 2.d.v — **Constraint contract:** `safety` defines the `Constraint` value type; `control`
  *depends on `safety`* to consume it (the one sanctioned cross-package edge). `safety` does
  **not** import `control`.
- Acceptance: constraints are pure values, unit-checked at the boundary (newtons, m/s,
  rad/s); a reference clamp applied to synthetic commands stays within bounds; zone-swap is
  deterministic given a pose stream.

### 2.e Safe-state command & drive-reaction contract
Definition: the package's outward command type — an abstraction over the CiA 402 transitions
and STO that `motor` implements, so `safety` stays below `motor` in the dependency graph.
[verified: landscape §2 for the bit patterns]
- 2.e.i — `SafeStateCommand` enumerates the reactions of §1.b.iv mapped to concrete CiA 402
  intents: `QUICK_STOP` → Controlword `xxxxxxxx0xxxx01x`, `STOP`/disable → Disable Voltage
  `xxxxxxxx0xxxxx0x`, `SHUTDOWN` → `xxxxxxxx0xxxx110`, `FAULT_RESET` → rising edge of bit 7
  (`xxxxxxxx1xxxxxxx`). [verified: landscape §2]
- 2.e.ii — A `SafeStatePort` interface (emit a command, read back a statusword `0x6041`) that
  a `motor`/`io` driver implements; `safety` holds only the port, never a concrete drive —
  dependency inversion keeps the edge pointing downward.
- 2.e.iii — **STO (Safe Torque Off):** a distinct, highest-severity command modeling the
  hardware STO function (de-energize the output stage independently of the control loop);
  `safety` treats it as terminal — no software path re-enables torque, only an external arm.
  [scope: STO is IEC 61800-5-2, not in the verified set]
- 2.e.iv — **Confirmation/timeout:** every command carries an expected resulting statusword
  and a deadline; non-confirmation escalates per §2.c.iv.
- Acceptance: command→bit-pattern mapping tested against the recorded CiA 402 table
  (driver-layer-style codec test against known bytes); port is mockable for hardware-free runs.

### 2.f Deterministic resource release on the fault path (the Cajeta differentiator)
Definition: the marquee feature — use Cajeta's ownership/borrow + deterministic `drop` to make
resource release on the fault path **provable, allocation-free, and GC-pause-free**, the
property the research flags as a genuine, marketable advantage (research §F). [scope:
positioning claim; the language mechanism it rests on is a Cajeta primitive]
- 2.f.i — **RAII safety guards:** a held resource (a drive enable, a brake-release, a current
  budget, a motion permit) is an owned guard whose `drop` *is* the de-energize action; when
  the owning scope unwinds on a fault, the safe action happens deterministically, in reverse
  acquisition order, with no GC and no finalizer queue.
- 2.f.ii — **Borrow-checked e-stop aliasing:** the stop-demand signal is shared by borrow,
  never by an unsynchronized alias; the compiler statically rejects a data race on the e-stop
  path (the "statically-checked aliasing for the e-stop chain" the research calls out).
- 2.f.iii — **Allocation-free fault path:** all buffers (event rings, reaction tables, command
  slots) are pre-allocated at construction; the fault path is verified to perform **zero**
  heap operations and take **no fallible call** (§1.b.v) — analogous to `transform`'s
  hot-path allocation discipline.
- 2.f.iv — **No-pause guarantee:** because release is `drop`-driven rather than GC-driven,
  worst-case release latency is the deterministic §2.c.iii bound, not a collector pause — the
  property a hard-real-time safety loop requires.
- Acceptance: a test (or static check) asserts the fault path allocates nothing and that
  dropping the supervisor releases every guard in order; a borrow-violation test confirms the
  e-stop alias is rejected at compile time.

### 2.g Conformance hooks, diagnostics & audit
Definition: the structure that makes a `safety` deployment *auditable* against the functional-
safety standards, without claiming a rating (§1.c.i). [scope: ISO 10218-1/-2, IEC 61508,
FSoE/CIP Safety — all paywalled, planning-grade]
- 2.g.i — **Fault log / black box:** a bounded, lock-free ring of (timestamp, fault class,
  reaction, statusword) records — the post-incident audit trail — drainable into `log`
  (MCAP) without touching the fault path.
- 2.g.ii — **Proof-obligation surface:** each reaction-table entry can carry a stated latency
  budget and a test reference, so the suite doubles as the evidence that §2.c bounds hold.
- 2.g.iii — **Diagnostic coverage hooks:** periodic self-tests (channel-discrepancy probe,
  watchdog round-trip, stuck-input detection) reported as a health status — the "diagnostic
  coverage" that IEC 61508 SIL argumentation expects. [scope]
- 2.g.iv — **Safety-fieldbus codec [scope, deferred]:** FSoE (Safety-over-EtherCAT, ETG) and
  CIP Safety are the on-the-wire safety protocols; should a codec be in scope, it follows the
  driver-layer pattern (protocol-codec tests against **recorded byte streams**), but the
  specs are paywalled — treat as a re-research-before-plan gap (research §F, §J). ~ depends on
  §2.e / `io`.
- Acceptance: log ring is hardware-free-testable and never blocks the fault path; self-test
  hooks are pure functions of injected fault conditions.

---

## 3. Acceptance criteria (beyond per-feature tests)
- 3.a — **Fail-safe by construction:** a property test asserts that every uninitialized /
  starved / signal-lost monitor resolves to demand-to-stop (§1.b.i), and that no code path
  auto-clears a latch (§1.b.ii).
- 3.b — **Bounded, allocation-free fault path:** the path from fault-latched to safe-state
  command emitted performs zero allocation and no fallible call (§1.b.v, §2.f.iii), with a
  worst-case latency bound pinned by test (§2.c.iii).
- 3.c — **Hardware-free:** the whole suite runs in CI with no devices — monitors driven by
  synthetic event+monotonic-clock streams, the drive side by a mock `SafeStatePort`; the
  CiA 402 mapping validated as a codec test against the recorded bit-pattern table
  (driver-layer discipline). Runs on x86-64 and at least one RISC-V target.
- 3.d — **No upward dependencies:** `safety` imports only `transform` and the Cajeta runtime;
  it defines the `Constraint`, `SafeStateCommand`, and `SafeStatePort`/`HwWatchdog` contracts
  that `control`, `motor`, and `io` consume — the dependency edges point **into** `safety`,
  never out of it.
- 3.e — **Convention conformance:** PFL constraints carry `transform` units and ordering
  unchanged (§1.b inherited); a test asserts force/torque and twist components are not
  silently reordered at the `control` boundary.
- 3.f — **Determinism:** every monitor and the reaction FSA are pure functions of (inputs,
  clock); identical input/clock streams produce identical reaction sequences (replayable).

## 4. Deliverables
- 4.a — `dev.cajeta.robotica.safety` implementing §2, with the §3 test suite (synthetic
  event/clock harness + recorded CiA 402 bit-pattern fixtures + a fault-path no-alloc check).
- 4.b — The published **contract types** (`SafeStateCommand`, `SafeStatePort`, `Constraint`,
  `HwWatchdog`) that downstream `control`/`motor`/`io` depend on — the sanctioned inward edges.
- 4.c — A plan at `agents/safety-plan.md` (authored when this spec is approved), TDD-ordered:
  contract types + fail-safe primitives → e-stop chain → watchdogs → fault-reaction FSA →
  PFL constraints → deterministic-drop fault-path proof → conformance/audit hooks.
- 4.d — A short **safety-positioning note** capturing the §2.f differentiator (deterministic-
  drop, allocation-free, GC-pause-free fault path) for the family's external pitch.

## 5. References (research-grounded)
- CiA 402 / IEC 61800-7 FSA — Controlword `0x6040` / Statusword `0x6041`, states (QUICK STOP
  ACTIVE, FAULT REACTION ACTIVE, FAULT, SWITCH ON DISABLED), and the Quick Stop / Disable
  Voltage / Shutdown / Fault-Reset bit patterns — the drive-side safe-state hook.
  **[verified]** (landscape §2)
- ros2_control read→update→write loop, default **100 Hz** — the deadline-monitor reference
  cadence. **[verified]** (landscape §3.1)
- CANopen EMCY / node-guarding, SYNC COB-ID `0x80` — emergency + liveness signaling context.
  **[verified]** (landscape §2)
- ISO 10218-1/-2 (industrial robot safety), **ISO/TS 15066** (collaborative robots — PFL
  biomechanical limits + SSM), IEC 61508 (SIL functional safety), IEC 61800-5-2 (STO), the
  safety fieldbuses **FSoE** (Safety-over-EtherCAT, ETG) and **CIP Safety**. **[scope —
  paywalled/ISO; not verified this pass; re-research before the plan]** (research §F, §J)
- Cajeta differentiator: ownership/borrow + deterministic drop ⇒ provable, GC-pause-free
  resource release and statically-checked e-stop aliasing on the fault path. **[scope:
  positioning, research §F call-out]**
