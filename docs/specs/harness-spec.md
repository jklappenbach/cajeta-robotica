# Robotica Harness — Specification

> Status: draft for review (2026-06-24). The **harness** is robotica's developer platform: a
> set of tools and technologies *wed together by a development lifecycle* for building, testing,
> deploying, and debugging embodied systems (controllers + ML) on real hardware. It is **not** a
> `dev.cajeta.robotica.*` library and is **not** deployed onto the device — it is the dev-time
> platform that *produces and operates* what runs on the device. Outline-numbered for
> addressability. Naming TBD (a confection/Spanish theme would fit cajeta/núcleo/caramelo).

## 1. Definition

### 1.1 Purpose
Turn the robotica **library family** (`transform`/`io`/`motor`/`fusion`/`estimation`/`control`/
`sim`/`log`/`safety`/`ai`) into a **turnkey platform**: a guided lifecycle that takes a developer
from "I have a robot idea" to "tested code + trained models running and debuggable on real
hardware," with current best practices baked in. The differentiator is **one language and one
toolchain from the matmul kernel to the FOC setpoint** — the harness is what makes that *turnkey*
rather than a pile of correct-but-DIY parts.

### 1.2 Scope
- A **project lifecycle** (model → simulate → author/test → HIL → deploy → on-HW debug → operate
  → capture) with tooling at each step.
- Two interleaved tracks: **Systems** (controllers/algorithms) and **Learning** (ML models),
  converging at deploy.
- A **cross-device debug + telemetry interface** that rides existing ecosystems.
- **Starter samples** for the common platforms/devices.
- The **compiler/runtime requirements** for compact on-device deployment (§7).

### 1.3 Non-goals
- **Not deployed on the device.** The harness runs on the developer host / CI; only the compiled
  executable (+ an optional debug agent) goes to the device.
- **Not a new debugger or a bespoke toolchain.** It makes cajeta **first-class in the existing
  ecosystems** (gdb, OpenOCD, probe-rs, Arduino IDE 2, RTT, Foxglove) by emitting standard
  artifacts — it does not reinvent them.
- **Not a library.** It orchestrates the robotica libraries; it does not add runtime APIs a robot
  links against (the libs stay clean and DCE-friendly).
- **No 8-bit AVR.** Classic ATmega Arduino (2KB RAM, 8-bit) is out of scope (§5.3).

### 1.4 Principle: tools wed by process
The harness's value is the **process** that connects mature tools, not the tools themselves. It
standardizes *one* deploy/debug/telemetry experience across device classes; only the transport
differs underneath (§6).

## 2. The development lifecycle (the spine)

**Use cases**
- **2.1 Model/scope.** As a developer, when I define the robot (frame tree, actuator/sensor
  inventory, control objectives), then the harness scaffolds a project using `transform` +
  URDF/SRDF + a manifest.
- **2.2 Simulate (HW-free).** As a developer, when I run the project before any hardware, then the
  whole sense→estimate→plan→control→actuate loop runs against `sim` (MJCF/URDF) and `io` **device
  simulators** (fake CAN/serial/I2C/GPIO + recorded byte streams).
- **2.3 Author + unit-test.** As a developer, when I write controller/estimation/planning code,
  then I test it hardware-free with `cajeta-unit` against recorded fixtures + sim (the family's
  recorded-byte-stream test philosophy).
- **2.4 HIL.** As a developer, when I bring up real hardware, then the harness lets me swap
  sim→real **one channel at a time** (real `io` drivers) while watching `log` telemetry, so I
  isolate driver/timing/calibration issues incrementally.
- **2.5 Deploy.** As a developer, when I target a device, then the harness cross-builds, installs
  the executable (+ optional debug agent), and runs it (§5, §7).
- **2.6 On-HW debug.** As a developer, when I debug on the device, then I connect through the
  device-appropriate transport to a standard debugger and stream telemetry (§6).
- **2.7 Operate + capture.** As a developer, when the robot runs, then `log` (MCAP) captures
  telemetry and inference-I/O, `safety` guards e-stop/watchdog, and captured data feeds back into
  the Learning track — the **data flywheel**.

## 3. The Learning track (ML lifecycle)

Parallel to Systems; integrates at deploy. This is the **robotics-scoped home for the deferred
`caramelo` framework** — turnkey ML over núcleo, bounded to robotics model classes.

**Use cases**
- **3.1 Identify.** As a developer, when I state a task (perception: detection/segmentation/depth;
  policy: imitation/RL/VLA; learned sensor-fusion), then the harness suggests an appropriate model
  (a curated robotics **model zoo** + task→model guidance).
- **3.2 Data.** As a developer, when I assemble training data, then I use dataset/episode-buffer
  tooling — seeded by the §2.7 capture flywheel.
- **3.3 Train.** As a developer, when I train, then I use núcleo (`nn`/`optim`/`autograd`) with
  best-practice defaults (splits/augmentation/checkpointing); training may run offline (incl. the
  Python LeRobot pipeline today).
- **3.4 Eval.** As a developer, when I evaluate, then I get metrics + recorded-stream replay
  (reproducible, hardware-free).
- **3.5 Export.** As a developer, when I export a model, then weights load via the **neutral
  núcleo weight-loader** (state_dict/`.pt`/safetensors → núcleo `Parameter`s) — not via the torch
  façade.
- **3.6 Deploy/infer.** As a developer, when I deploy a policy, then `ai` runs it on
  `cajeta.math`→`cajeta.xpu`; later, on-device adaptation (the cradle/SPELA thread) reuses
  `nucleo.autograd`.

## 4. Tooling map (tools / tactics / strategies per step)

**Use cases**
- **4.1** As a developer, then each lifecycle step maps to concrete tooling: scaffold (`transform`
  + manifest), sim (`sim` + `io` simulators), test (`cajeta-unit` + fixtures), HIL (channel-swap
  runner + `log`), deploy (cajeta build + §7 embedded profile + flash via `arduino-cli`/`probe-rs`),
  debug+telemetry (§6), operate (`log`/`safety` + Foxglove).
- **4.2** As a developer, when a step has a best-practice default, then the harness applies it
  (e.g. recorded-fixture tests, channel-at-a-time HIL, split-debug deploy, non-halting
  observability for real-time loops).

## 5. Targets & tiers

**Use cases**
- **5.1 Tier 1 — Raspberry Pi (Linux SBC; Pi/Orange Pi/RISC-V/Jetson).** As a developer, when I
  target a Pi, then the **full lifecycle works now**: cajeta compiles native Linux-ARM, debug is
  standard `gdb`/`gdbserver` over SSH/IP, telemetry is `log`→Foxglove over IP. The harness adds
  cross-build + deploy + remote-gdb + telemetry wiring.
- **5.2 Tier 2 — modern 32-bit Arduino (RP2040 first; then SAMD/ESP32/UNO-R4).** As a developer,
  when I target a modern MCU board, then deployment is gated on a **cajeta embedded target**
  (§7); once present, debug/telemetry come "for free" by emitting standard ELF+DWARF + Serial/RTT
  into the existing OpenOCD/probe-rs/Arduino-IDE ecosystem. **Beachhead: RP2040** (Pi Pico /
  Arduino Nano RP2040) — it bridges both worlds (Raspberry Pi's MCU *and* a first-class Arduino
  board), Cortex-M0+, 264KB RAM, SWD via picoprobe, excellent open tooling.
- **5.3 Out of scope — 8-bit AVR** (ATmega Uno/Nano/Mega): 2KB RAM / 8-bit / weak LLVM-AVR — too
  constrained for a memory-safe runtime. Not promised.

## 6. Cross-device debug & telemetry interface

Strategy: **standardize one interface** — a debug agent + the **MCAP/Foxglove** telemetry
protocol (already in `log`) — with the **transport abstracted per device class**. "First-class"
means cajeta emits the *same standard artifacts* the C/Rust/Python toolchains consume, so the
mature debug ecosystem adopts cajeta binaries rather than us building a parallel debugger.

**Use cases**
- **6.1 Linux SBC.** As a developer, then debug is `gdbserver`/`gdb` (or an in-proc agent) over
  IP/SSH, and telemetry is Foxglove over WebSocket — works today on native cajeta ELF.
- **6.2 MCU with a debug port** (Cortex-M/ESP32/RP2040). As a developer, then debug is
  **gdb-over-probe via OpenOCD/probe-rs over SWD/JTAG** (the gold standard — needs the debug pins
  / signal lines exposed), with **SWO/ITM trace**, **SEGGER RTT**, **semihosting**, or a **UART
  debug-monitor** as alternatives; telemetry is Serial/RTT (→ Foxglove host-side). Requires cajeta
  to emit standard **ELF + DWARF** (§7).
- **6.3 Motor controller / FOC drive.** As a developer, then I prefer **non-halting
  observability** (you cannot pause a spinning motor): stream FOC telemetry over CAN/CANopen
  (CiA-402) or RTT, with the `safety` watchdog mandatory — breakpoint-halting is unsafe in a
  real-time control loop.
- **6.4 Split debug.** As a developer, when I deploy, then the **stripped** image flashes to the
  device and the **ELF+DWARF** stays host-side for the debugger — debug info never costs device
  flash/RAM.
- **6.5 Consistent DX.** As a developer, then the deploy/debug/telemetry *commands* are the same
  Pi→MCU; only the transport (IP / SWD / UART / CAN) differs.

## 7. Compact-code requirements (compiler/runtime for on-device deploy)

cajeta already holds the two biggest embedded size levers: **aggressive tree-shake DCE**
(default-on for `--emit=exe`) and **no GC** (the ownership/drop model needs no collector runtime).
The rest is an **embedded profile** bundling standard LLVM levers + cajeta-specific work.

**Use cases**
- **7.1 Embedded profile.** As a developer, when I build for a device, then a single
  **`--profile=embedded`** (or target-triple-driven) flips the full size configuration together
  (the way `--profile=test` bundles its flags): `-Oz` + `-ffunction-sections -fdata-sections` +
  linker `--gc-sections` + **LTO** (full LTO for size; ThinLTO for build speed) + `--strip-all`
  (split-debug, §6.4) + target codegen (`-mcpu`/`-mthumb`/`-mfloat-abi`).
- **7.2 Reflection-off / closed-world.** As a developer, when I build embedded, then reflection is
  **off** (`--reflect=closed`) so RTTI globals + `Class<T>` metadata are DCE-stripped (lean-linker
  / bounded-reflection model).
- **7.3 No-unwind / abort-on-panic.** As a developer, when I build embedded, then exception
  **unwinding tables + landing pads are dropped** in favor of abort-on-panic (cf. Rust
  `panic=abort`) — a major MCU size win. *(New work: cajeta has exceptions today.)*
- **7.4 Minimal freestanding runtime.** As a developer, when I build embedded, then the runtime
  drops fibers/carrier/OS dependencies and uses static/**arena** allocation (building on the
  frame-arena), linking newlib-nano or no libc. *(New work.)*
- **7.5 Conditional alignment.** As a developer, when I build embedded, then the **64-byte
  Arrow/SIMD alignment retrofit falls back to natural alignment** — 64-byte padding wastes scarce
  MCU RAM. *(New work in the column/tensor retrofit.)*
- **7.6 Monomorphization control.** As a developer, then unused template instantiations are
  DCE'd and identical instantiations deduped (monomorphization is a size source).
- **Acceptance:** a representative robot executable builds under `--profile=embedded` and fits the
  RP2040 flash/RAM budget; the same source debugs over SWD via probe-rs/OpenOCD using the
  host-side DWARF.

## 8. Starter samples (the getting-started kit)

**Use cases**
- **8.1** As a newcomer, when I start, then a **sensor** sample reads an IMU and fuses to attitude
  (`io.sensor` + `fusion`).
- **8.2** As a newcomer, then an **actuator-with-robust-control** sample drives a servo/FOC motor
  with PID/impedance control (`io.motor` + `control`).
- **8.3** As a newcomer, then **Bluetooth** and **IP networking** samples show connectivity.
- **8.4** As a newcomer, then each sample runs in sim first (Tier 1) and has a deploy path; the
  samples double as end-to-end validation of the libraries.

## 9. Dependencies (on the cajeta compiler/stdlib)
- **Tier 1 (Pi):** native Linux-ARM (exists); `log` MCAP/Foxglove (exists).
- **Tier 2 (MCU):** a **cajeta embedded target** — embedded codegen (Thumb/RISC-V/Xtensa via
  LLVM), the §7 minimal runtime + embedded profile, standard ELF+DWARF + Serial/RTT. This is a
  **cajeta-compiler frontier**, tracked as its own work, but designed under the §6 interface so
  Tier-2 inherits the Tier-1 DX.
- `ai` → núcleo core + neutral weight-loader (not the torch façade); `cajeta.xpu` spatial index
  for perception/estimation spatial queries.

## 10. Acceptance criteria (spec-level)
- The lifecycle (§2) is runnable end-to-end on Tier 1 (Pi): scaffold → sim → test → HIL → deploy →
  gdb/Foxglove debug → capture.
- The debug/telemetry interface (§6) is consistent across device classes (transport abstracted).
- `--profile=embedded` (§7) produces an image that fits the RP2040 beachhead and debugs over SWD
  via the existing ecosystem.
- The starter samples (§8) run in sim and deploy to at least one Tier-1 device.

## 11. Open questions
- **Name** for the harness (confection/Spanish theme?).
- Harness as its **own repo** vs. a top-level part of cajeta-robotica.
- Tier-2 sequencing: how much of the embedded target is prerequisite vs. incremental (RP2040
  first; which features gate a first flash).
- ML track (§3): how much is harness-native vs. thin orchestration over núcleo, and when the
  caramelo name attaches.
- HIL channel-swap mechanism (config-driven sim↔real per channel).

## 12. Deliverables
- This spec, then a plan (`agents/.../harness-plan.md`) — Tier-1 lifecycle + samples first; the
  cajeta embedded-target work scoped as a dependency track.
- A reference scaffold + the §8 starter samples.
- The §6 debug/telemetry interface + the §7 `--profile=embedded` (the latter spanning into the
  cajeta compiler repo).
