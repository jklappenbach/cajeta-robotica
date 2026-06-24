# `dev.cajeta.robotica.motor` — Specification

The actuator/drive **driver** layer of cajeta-robotica. `motor` is where the
sense→estimate→plan→control→actuate spine meets copper: it turns control-layer setpoints
into the exact bytes a smart drive expects, and turns the drive's feedback frames back into
typed state. **The drive closes the FOC (current/velocity/position) loop; `motor` writes
setpoints and reads feedback.** It depends on `transform` (shared time/units/value types) and
on `io` (the physical transports — serial/UART, CAN/CAN-FD, I2C); it **never** depends upward
on `control`, `planning`, or anything in the estimation/AI tiers. See the family spec
[`../robotica-spec.md`](../robotica-spec.md) §2.e and the actuation-plane research
[`../../research/robotics-software-landscape.md`](../../research/robotics-software-landscape.md)
§§1–3.

---

## 1. Definition

`motor` provides **byte-exact protocol codecs and device state machines** for the actuator
classes a robot drives: smart serial-bus servos (DYNAMIXEL Protocol 2.0), BLDC/FOC drives
(moteus, ODrive, VESC), industrial servo drives via the CiA 402 / IEC 61800-7 device-control
state machine (over CANopen / CoE), and the low-end PWM-servo and stepper drivers. It also
defines the small **driver abstraction** (an actuator/sensor component model with a typed
command/state interface registry and a fixed-rate read→update→write executor) that lets the
`control` layer command heterogeneous drives uniformly. It is protocol logic, not transport:
every codec is unit-testable **without hardware** by encoding to / decoding from recorded byte
streams.

### 1.a Capability summary
- 1.a.i — A **common actuator-driver model**: System / Actuator / Sensor component types, a
  typed command/state interface registry keyed by joint/sensor/gpio, and a read→update→write
  loop (the ros2_control `hardware_interface` contract, research §3.1).
- 1.a.ii — **DYNAMIXEL Protocol 2.0** instruction/status packet codec — header, length, CRC-16,
  byte stuffing, the full instruction set (research §1.1).
- 1.a.iii — **Sync / Bulk batching** — command or read a whole chain of bus servos in one bus
  transaction (Sync Write/Read, Bulk Write/Read, Fast Sync Read).
- 1.a.iv — **FOC drive register protocols**: moteus CAN-FD multiplexed-subframe codec, ODrive
  CANSimple message set, VESC UART packet codec (research §1.2).
- 1.a.v — **CiA 402 / IEC 61800-7 device-control FSA**: Controlword `0x6040` / Statusword
  `0x6041` driven state machine, Modes-of-operation `0x6060`/`0x6061`, CSP/CSV/CST (research §2).
- 1.a.vi — **CiA 402 transport bindings** over CANopen (PDO/SDO/SYNC) and CoE.
- 1.a.vii — **PWM hobby servos (PCA9685)** and **steppers (TMC/step-dir)** (research §1.3,
  *[scope]*).
- 1.a.viii — **Feedback decode, fault surfacing, and safe-state** entry on the actuation path.

### 1.b Conventions
> `motor` **inherits the family normative conventions from `transform` §1.b by reference** and
> does not redefine them. Angles are radians, lengths meters, time SI seconds
> (`transform` §1.b.iv); timestamps/durations are `transform`'s `Time`/`Duration`
> (§2.e.i); twist/wrench ordering, when a 6-DOF command is expressed, is translation-first
> per `transform` §1.b.ii. The notes below are **transport-local** encoding facts, not new
> family conventions.
- 1.b.i — **Joint-space command altitude.** A `motor` command is a per-actuator (1-DOF) scalar
  setpoint — position (rad), velocity (rad/s), torque/current (N·m / A), or a small gain set —
  *not* an SE(3) pose. Cartesian/whole-body intent is resolved above, in `control`/`kinematics`.
- 1.b.ii — **SI at the API boundary, drive units on the wire.** Every codec is responsible for
  the bidirectional conversion between SI command/state values and the drive's native register
  encoding (counts, fixed-point LSBs, drive-specific scaling); SI never leaks the wire format.
- 1.b.iii — **Per-protocol byte order is a property of the protocol, fixed by its spec, not by
  the family.** DYNAMIXEL fields and control-table values are **little-endian**; CANopen object
  payloads are little-endian; moteus subframe values use documented per-type LSB scaling. Each
  codec pins its own and tests it against recorded bytes.
- 1.b.iv — **Sign/direction is a per-drive calibration**, surfaced as an explicit, documented
  parameter on the driver — never silently baked into a codec.

### 1.c Non-goals
- 1.c.i — **Does not close the FOC loop.** Clarke/Park, current PI, SVPWM stay on the drive MCU;
  reimplementing FOC natively is a separate, larger effort (research §1.2 scope note).
- 1.c.ii — **Does not own the transport.** Opening a socketcan/CAN-FD socket, an RS-485 UART, or
  an I2C bus — and any EtherCAT MainDevice / Distributed-Clock master — belongs to `io`
  (family §2.b). `motor` consumes a byte/frame channel; it does not implement the link.
- 1.c.iii — **No trajectory generation, no kinematics/IK, no fusion/estimation.** Those are
  `planning`/`control`, `kinematics`, and `fusion`/`estimation` respectively.
- 1.c.iv — **No motion-policy or safety policy.** `motor` can *enter* a drive's safe state on
  command or on a missed-deadline signal, but the watchdog/e-stop policy itself is `safety`.

---

## 2. Features

### 2.a Common actuator-driver model
Definition: the small framework contract every concrete driver implements so `control` can
command heterogeneous hardware uniformly — modeled on ros2_control's `hardware_interface`
(research §3.1, **[verified]**).
- 2.a.i (use case) — Declare a hardware component as exactly one of **System** (complex
  multi-DOF), **Actuator** (simple 1-DOF), or **Sensor** (read-only).
- 2.a.ii — Register typed **command interfaces** (position/velocity/effort/…) and **state
  interfaces** keyed by `joint` (command + state), `sensor` (state only), or `gpio` (either),
  per the element/interface hierarchy.
- 2.a.iii — Run a fixed-rate **read → update → write** executor: read state from hardware,
  let the controller update, write commands — with a configurable loop rate (default 100 Hz).
- 2.a.iv — Report a per-driver lifecycle (unconfigured → inactive → active → error) and surface
  a typed health/fault status into the state registry.
- 2.a.v — Resolve a logical joint name to a concrete `(driver, bus id, register/address)` so the
  same control code binds to a DYNAMIXEL id, a moteus node, or a CiA 402 slave unchanged.
- Acceptance: the registry round-trips command/state names and types; a recorded-trace harness
  drives a mock component through a full read→update→write cycle with no hardware.

### 2.b DYNAMIXEL Protocol 2.0 packet codec
Definition: the byte-exact instruction/status packet format over TTL/RS-485 — the cleanest
first deliverable because the whole control surface is a packet codec (research §1.1,
**[verified]**).
- 2.b.i — **Encode an instruction packet**: prefix `0xFF 0xFF 0xFD 0x00`, 1-byte ID
  (0–252; `0xFE` = broadcast), 2-byte little-endian Length = `param_count + 3`, instruction,
  parameters, 2-byte CRC.
- 2.b.ii — **CRC-16** over header-through-parameters (excluding the CRC field), matching the
  ROBOTIS polynomial; verify on decode.
- 2.b.iii — **Byte stuffing**: insert an extra `0xFD` after any `0xFF 0xFF 0xFD` run in the
  payload on encode, and strip it on decode, so payload can never be mistaken for a header.
- 2.b.iv — **Instruction set**: Ping `0x01`, Read `0x02`, Write `0x03`, Reg Write `0x04`,
  Action `0x05`, Factory Reset `0x06`, Reboot `0x08`, Clear `0x10`, Control-Table Backup `0x20`,
  Status `0x55`, Sync Read `0x82`, Sync Write `0x83`, Fast Sync Read `0x8A`, Bulk Read `0x92`,
  Bulk Write `0x93`, Fast Bulk Read `0x9A`.
- 2.b.v — **Decode a status packet**: validate prefix/length/CRC, surface the error byte, and
  return parameters; reject malformed/truncated frames deterministically.
- 2.b.vi — Read/write a **control-table** field by (address, length) with SI↔raw conversion
  (§1.b.ii).
- Acceptance: encode/decode round-trips a corpus of recorded packets byte-for-byte; CRC and
  byte-stuffing vectors pass; a corrupted byte is detected and rejected.

### 2.c DYNAMIXEL Sync / Bulk batched transactions
Definition: the throughput primitive — command or read an entire arm in **one** bus
transaction (research §1.1, **[verified]**).
- 2.c.i — **Sync Write** a single control-table address/length to *N* devices in one packet
  (same address + same length across devices) — e.g. goal-position for a whole chain.
- 2.c.ii — **Sync Read** / **Fast Sync Read** the same address/length from *N* devices, parsing
  one concatenated status response into per-id values.
- 2.c.iii — **Bulk Write** / **Bulk Read** to *N* devices at **different** addresses and lengths
  in one packet (the heterogeneous-chain case).
- 2.c.iv — Build a batch from the driver model's joint registry (§2.a.v) so one control tick
  emits one bus transaction.
- Acceptance: a recorded multi-servo Sync/Bulk exchange decodes to the correct per-id map;
  encoded batch matches the recorded master bytes.

### 2.d FOC drive register protocols (moteus / ODrive / VESC)
Definition: register-map + frame-encoding codecs for the BLDC/FOC drives that close the loop
internally; the host writes setpoints and reads feedback (research §1.2, **[verified]**).
- 2.d.i — **moteus (CAN-FD)**: encode/decode the multiplexed **subframe** format — write
  opcodes `0x00/0x04/0x08/0x0c` for int8/int16/int32/float, a varuint register count, a start
  register, then values with documented per-type LSB scaling (e.g. current LSB 1 / 0.1 / 0.001 A).
- 2.d.ii — **moteus setpoint map**: Mode `0x000`, Position `0x020`, Velocity `0x021`,
  Kp-scale `0x023`, Max-torque `0x025`, Velocity-limit `0x028` — honoring **NaN** sentinels
  (`0x020` "use current position", `0x025` "use phase current limit").
- 2.d.iii — **ODrive (CANSimple)**: pack/unpack the per-axis `node_id` + command id into the CAN
  arbitration id; encode Set_Input_Pos/Vel/Torque, Set_Controller_Mode, Set_Limits,
  Set_Axis_State, Clear_Errors/Estop; decode Get_Encoder_Estimates, Get_Iq, Get_Temperature,
  Get_Bus_Voltage_Current, Get_Error, and Heartbeat.
- 2.d.iv — **ODrive bitrate/addressing**: model classic CAN 2.0b and CAN-FD (separate nominal +
  data-phase bitrates, BRS), and the RxSdo/TxSdo register read/write channel.
- 2.d.v — **VESC (UART)**: frame as start byte (`2` short / `3` long), 1–2 length bytes, payload,
  CRC16-CCITT over the payload, stop byte; encode setters (e.g. set-current) and the
  request/async-callback `get_values` telemetry struct (voltage, temperature, current, RPM,
  duty, faults).
- Acceptance: each codec round-trips recorded drive traces; moteus NaN sentinels, ODrive
  arbitration-id packing, and VESC CRC16-CCITT all verified against captured bytes.

### 2.e CiA 402 / IEC 61800-7 device-control FSA & modes
Definition: the network-agnostic, fully enumerated drive state machine — one implementation
drives a large class of industrial servo/inverter/stepper drives (research §2, **[verified]**;
highest-leverage surface per research §6.1).
- 2.e.i — Drive the **device-control FSA** through its states: NOT READY TO SWITCH ON,
  SWITCH ON DISABLED, READY TO SWITCH ON, SWITCHED ON, OPERATION ENABLED, QUICK STOP ACTIVE,
  FAULT REACTION ACTIVE, FAULT — with the motor energized only in OPERATION ENABLED.
- 2.e.ii — Emit **Controlword** (`0x6040`) command bit patterns: Shutdown `xxxxxxxx0xxxx110`,
  Switch On `xxxxxxxx0xxxx111`, Disable Voltage `xxxxxxxx0xxxxx0x`, Quick Stop
  `xxxxxxxx0xxxx01x`, Enable Operation `xxxxxxxx0xxx1111`, Fault Reset `xxxxxxxx1xxxxxxx`
  (rising edge; only bits 0,1,2,3,7 change state).
- 2.e.iii — Decode **Statusword** (`0x6041`) to the current FSA state and compute the next
  legal Controlword to reach a requested state (transition planner).
- 2.e.iv — Select an operating mode via **`0x6060`** and confirm via **`0x6061`**: 1 Profile
  Position, 3 Profile Velocity, 6 Homing, 8 CSP, 9 CSV, 10 CST.
- 2.e.v — Stream **cyclic-synchronous** setpoints (CSP target position / CST target torque) at a
  fixed cycle, reading the matching feedback object each cycle.
- 2.e.vi — Surface drive faults and run the **Fault Reset** edge to recover from FAULT.
- Acceptance: a transition-table test asserts every (state, command)→state edge and rejects
  illegal transitions; Controlword/Statusword bit-mask vectors pass against the normative table;
  all hardware-free.

### 2.f CiA 402 transport bindings (CANopen / CoE)
Definition: how the §2.e object dictionary is moved on the wire — the binding layer between the
FSA and `io`'s links (research §2 "Transport bindings", **[verified]** for the object/SYNC
model; **[scope]** for EtherCAT DC timing, which lives in `io`).
- 2.f.i — **CANopen (CiA DS301)**: map FSA objects onto **PDOs** (cyclic process data) and
  **SDOs** (acyclic config), pace cyclic exchange off the **SYNC** object (COB-ID `0x80`), and
  surface **EMCY** emergency messages and node-guarding state.
- 2.f.ii — **CoE (CANopen over EtherCAT)**: present the same object-dictionary access over an
  EtherCAT mailbox/process-data channel supplied by `io`'s MainDevice.
- 2.f.iii — Build a **PDO mapping** (object → process-image offset) for CSP/CST so one cycle
  writes Controlword + target and reads Statusword + actual.
- 2.f.iv — *[scope]* EtherCAT **Distributed-Clock** synchronization (reference clock = first
  DC-capable slave's 64-bit ns System Time `0x0910`; offset + propagation-delay + drift
  compensation) is required for true cyclic-synchronous motion but is implemented in `io`'s
  EtherCAT master; `motor` consumes the synchronized cycle, it does not compute DC offsets.
- Acceptance: PDO/SDO encode/decode round-trips recorded CANopen frames; SYNC-paced cyclic
  exchange reproduces a recorded master trace; the DC dependency is documented, not duplicated.

### 2.g PWM hobby servos & steppers *[scope]*
Definition: the low-end actuators — small, well-bounded register/timing targets the research
flags as *not independently verified* (research §1.3). Treated as scope notes pending
re-research before this feature's plan.
- 2.g.i *[scope]* — **PCA9685** 16-channel I2C PWM: program the prescaler for ~50 Hz and write
  per-channel ON/OFF tick registers for a 1–2 ms hobby-servo pulse (register map to be verified
  against the NXP datasheet before plan).
- 2.g.ii *[scope]* — **Step/direction** stepper output: generate step pulses at a commanded
  rate with a direction line (timing-source ownership to be settled with `io`).
- 2.g.iii *[scope]* — **TMC UART** (e.g. TMC2209) register configuration: microstep, current,
  stealthChop (register set to be verified against the TMC datasheet before plan).
- Acceptance (deferred): codec/timing tests added once §1.3 surfaces are re-verified with
  primary datasheets; until then this feature ships interface stubs only.

### 2.h Feedback, fault & safe-state handling
Definition: the read-side and the actuation-path safety hooks common to all drivers.
- 2.h.i — Decode periodic feedback (position/velocity/current/temperature/bus-voltage/error)
  into the §2.a state registry with `transform` timestamps.
- 2.h.ii — Normalize each protocol's fault/error reporting (DYNAMIXEL error byte, ODrive
  Get_Error, VESC fault code, CiA 402 FAULT state + EMCY) into one typed fault enum + raw detail.
- 2.h.iii — Provide a uniform **enter-safe-state** action (Quick Stop / disable voltage / zero
  torque) any driver can execute immediately, including on a missed-deadline signal from
  `safety`.
- 2.h.iv — Guarantee **deterministic resource release** on the fault/teardown path using Cajeta
  ownership/drop (no GC pause between fault detection and the drive being commanded safe).
- Acceptance: a recorded fault trace decodes to the correct typed fault; the safe-state action
  emits the correct bytes for each protocol; teardown releases the transport handle deterministically.

---

## 3. Acceptance criteria (beyond per-feature tests)
- 3.a — **Protocol-codec conformance against recorded byte streams.** Every codec (DYNAMIXEL,
  moteus, ODrive, VESC, CiA 402/CANopen) has encode/decode tests that round-trip captured
  real-device byte streams; CRC/byte-stuffing/bit-pattern vectors match the normative spec.
- 3.b — **Hardware-free CI.** The whole suite runs with no devices, on x86-64 and at least one
  RISC-V target; drivers are exercised through recorded-trace mocks (family §3.b).
- 3.c — **Board-agnostic, no native deps.** `motor` uses only the byte/frame channels `io`
  provides (socketcan/serial/I2C); it pulls in no non-portable native library (family §3.c).
- 3.d — **No upward dependencies.** `motor` imports only `transform` and `io` from the family —
  never `control`, `planning`, `estimation`, `fusion`, `kinematics`, `ai`, `sim`, or `safety`.
- 3.e — **SI boundary / drive-unit interior.** A conformance test asserts SI↔raw conversion is
  bidirectional and lossless within tolerance, and that no wire-format scaling leaks past the
  driver API (§1.b.ii).
- 3.f — **FSA legality.** The CiA 402 driver only ever emits legal (state, command) transitions;
  an exhaustive transition-table test rejects illegal ones.
- 3.g — **Safe-state liveness.** From any active state, enter-safe-state (§2.h.iii) produces the
  correct protocol bytes and releases resources deterministically (§2.h.iv).

## 4. Deliverables
- 4.a — `dev.cajeta.robotica.motor` implementing §2 (with §2.g shipping interface stubs until
  its scope notes are re-verified), plus the §3 test suite.
- 4.b — A corpus of **recorded byte-stream fixtures** (DYNAMIXEL Sync/Bulk exchange, moteus/
  ODrive/VESC traces, a CiA 402 CANopen cyclic exchange) that the codec tests assert against.
- 4.c — A short driver-author guide: how to add a new drive by implementing the §2.a contract
  and a codec, with the SI-boundary and recorded-trace-test requirements.
- 4.d — A plan at `agents/motor-plan.md` (authored when this spec is approved), TDD-ordered:
  driver model (§2.a) → DYNAMIXEL codec (§2.b) → Sync/Bulk batching (§2.c) → FOC register codecs
  (§2.d) → CiA 402 FSA (§2.e) → CANopen/CoE bindings (§2.f) → feedback/fault/safe-state (§2.h) →
  PWM/stepper (§2.g, after re-research).

## 5. References (research-grounded)
- DYNAMIXEL Protocol 2.0 packet format, CRC-16, byte stuffing, instruction set, Sync/Bulk —
  research §1.1; ROBOTIS e-Manual https://emanual.robotis.com/docs/en/dxl/protocol2/ . **[verified]**
- moteus CAN-FD subframe codec + setpoint register map (NaN sentinels) — research §1.2;
  https://github.com/mjbots/moteus/blob/main/docs/reference.md . **[verified]**
- ODrive CANSimple message set + arbitration-id/node_id packing, CAN-FD bitrates — research §1.2;
  https://docs.odriverobotics.com/v/latest/manual/can-protocol.html . **[verified]**
- VESC UART packet frame + CRC16-CCITT + `get_values` telemetry — research §1.2;
  http://vedder.se/2015/10/communicating-with-the-vesc-using-uart/ . **[verified]**
- CiA 402 / IEC 61800-7 FSA, Controlword `0x6040` / Statusword `0x6041` / Modes
  `0x6060`/`0x6061`, CSP/CSV/CST — research §2;
  https://www.can-cia.org/can-knowledge/cia-402-series-canopen-device-profile-for-drives-and-motion-control
  and AMK DS402 interface PDF. **[verified]**
- CANopen DS301 PDO/SDO/SYNC (COB-ID `0x80`)/EMCY transport binding; CoE over EtherCAT —
  research §2 "Transport bindings". **[verified]** (object/SYNC model) / **[scope]** (EtherCAT
  Distributed-Clock timing — owned by `io`, research §1.4).
- ros2_control `hardware_interface` 3-type component model + read→update→write loop (default
  100 Hz) — research §3.1;
  https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html .
  **[verified]**
- PCA9685 PWM, TMC2209 UART, step/direction steppers — research §1.3. **[scope]** (not
  independently verified; re-research before the §2.g plan).
