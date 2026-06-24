# `dev.cajeta.robotica.io.sensor` — Specification

The **sensor-driver** package of cajeta-robotica: the *transport + parse* layer that turns
bytes off a wire (I2C/SPI register banks, serial frames, UDP packets, GPIO edges) into
typed, timestamped sensor samples in the family's units and frames. It is a **driver
layer**, not a math layer — it emits **raw** measurements or **device-fused** outputs and
stops there; the estimation math (AHRS, EKF, the F/T-bridge fusion proper) lives in
`fusion`. See the family spec [`../../robotica-spec.md`](../../robotica-spec.md) §2.c and the
research [`../../research/robotics-stack-part2-perception-to-ai.md`](../../research/robotics-stack-part2-perception-to-ai.md)
§A.2–A.5.

---

## 1. Definition

`io.sensor` provides **device drivers** for the sensing front-end of a robot: IMUs (raw and
on-chip-fused), encoders (incremental quadrature and absolute), force/torque sensors and load
cells, LiDAR (2-D and 3-D), and sonar/ToF rangefinders. Each driver is a codec + a small state
machine over a transport supplied by `io` (§1.d): it configures the device, reads its data
registers/frames, applies the device's own calibration/scaling, stamps the result with a
`transform.Time`, and hands up a value type expressed in family units. The split that
organizes the whole package — **does the host do the fusion, or did the chip?** — is the key
design axis (research §A.2): a raw-IMU driver streams accel/gyro/mag for `fusion` to integrate,
while a BNO08x driver is a *transport+parse* job that surfaces the quaternion the chip already
computed.

### 1.a Capability summary
- 1.a.i — Stream **raw** calibrated accel/gyro/mag from MPU-6050/9250 / ICM-20948-class IMUs,
  or a **device-fused** orientation quaternion from a BNO055/BNO08x (SH-2).
- 1.a.ii — Decode **incremental quadrature** (A/B/Z) counts and read **absolute** angle from
  magnetic encoders (AS5048/AS5600) and clocked-serial absolute encoders (SSI, BiSS-C).
- 1.a.iii — Map a 6-axis **force/torque** strain vector to a `Wrench` via its 6×6 calibration
  matrix; read a **load cell** through an HX711 24-bit ADC.
- 1.a.iv — Parse a **LiDAR** stream (RPLidar serial; Velodyne/Ouster UDP) into ranges / a point
  buffer; read a **sonar/ToF** rangefinder (HC-SR04 echo timing; VL53L0X over I2C).
- 1.a.v — A uniform **sample model** — value + `transform.Time` stamp + frame id + status/health
  — and a uniform device lifecycle (open → configure → start → read → stop).

### 1.b Conventions (NORMATIVE by inheritance)
> This package **inherits transform's normative conventions by reference** (transform-spec
> §1.b) and does not restate or override them. In particular: quaternions are **Hamilton,
> scalar-last `[x,y,z,w]`, `w ≥ 0`** (transform §1.b.i); twist/wrench are **translation-first
> `[ρ;θ]` / `[force;torque]`** (transform §1.b.ii); **radians, meters, SI seconds**
> (transform §1.b.iv); right-handed frames (transform §1.b.v).
- 1.b.i — Every device-fused orientation a driver emits (e.g. a BNO08x quaternion) is
  **converted into the transform convention at the driver boundary** — Hamilton, scalar-last,
  canonical sign — so no downstream consumer ever sees a vendor's scalar-first/JPL frame
  (research §A.1 quaternion-convention hazard).
- 1.b.ii — Each sample carries the **frame id** of the sensor's measurement frame (the IMU
  case, the encoder shaft, the F/T sensor flange); placing that frame in the robot is the
  `transform` frame-tree's job, not this package's.
- 1.b.iii — Timestamps are `transform.Time`; a driver records the **best timestamp it can
  source** (hardware/DMA timestamp where the transport in §1.d exposes one, host-receipt time
  otherwise) and **labels which** so estimators can model latency.

### 1.c Non-goals
- 1.c.i — **No fusion math.** AHRS (Madgwick/Mahony), the EKF/UKF/error-state tier, and
  multi-sensor state estimation live in `fusion`/`estimation` (research §A.1). This package
  emits the *inputs* to those filters (and passes through a chip's own fused output verbatim).
- 1.c.ii — **No transport implementation.** Opening the UART/CAN/I2C/SPI/GPIO/UDP socket,
  bit-timing, and timestamps are `io`'s job (§1.d); a driver consumes a transport handle.
- 1.c.iii — **No cameras / depth / point-cloud pipelines** — that is `io.vision` (the
  native/heavy-dependency edge of the family). LiDAR/sonar here are bounded serial/UDP/GPIO
  range sensors, not imaging.
- 1.c.iv — **No URDF/extrinsic calibration discovery.** Where a sensor sits on the robot is
  configuration consumed by `transform`/`kinematics`, not derived here.

### 1.d Dependencies
- 1.d.i — **`transform`** — `Time`/`Duration`, units, vec3, the quaternion/rotation type,
  `Wrench`, frame ids. (Downward only; never imports a sibling driver, `fusion`, or upward.)
- 1.d.ii — **`io`** — the transport handles: serial/UART, I2C/SPI register read/write,
  GPIO/edge-timing, CAN, and raw UDP sockets (family spec §2.b). Drivers are **board-agnostic**:
  they touch hardware only through these `io` handles, so the package carries **no native
  dependency** and runs on any Linux SBC target (family pillar 3.c).

---

## 2. Features

> Research basis: §A.2–A.5 are tagged **[scope note — not verified this pass]** (research §A.2,
> §J). Concrete register addresses, frame layouts, bit orders, and packet structures below are
> therefore marked **[scope: verify vs datasheet]** and MUST be confirmed against the primary
> datasheet/protocol document before the corresponding plan item is coded. The *driver shape*
> (raw-vs-fused split, codec + small state machine, calibration as a matvec) is the load-bearing
> design content; the exact wire numbers are to-be-verified.

### 2.a IMU drivers (raw and on-chip-fused)
Definition: two driver families distinguished by **where fusion happens** (research §A.2). A
**raw** driver exposes accel/gyro(/mag) register banks; a **fused** driver parses a quaternion
the chip already computed.
- 2.a.i (use case) — Configure and stream **raw** accel + gyro from an MPU-6050/9250-class IMU
  over I2C/SPI: set full-scale ranges, sample-rate divider, and DLPF; read the data registers;
  apply scale to produce SI accel (m/s²) and gyro (rad/s). **[scope: verify register map vs
  datasheet]**
- 2.a.ii — Stream **raw 9-DoF** (accel + gyro + magnetometer) from an ICM-20948-class part,
  including the auxiliary-I2C magnetometer (AK09916) path, with per-axis scale and the chip's
  axis frame mapped into the sensor frame (§1.b.ii). **[scope: verify vs datasheet]**
- 2.a.iii — Apply a stored **calibration** to raw IMU output before emit: gyro bias, accel
  bias/scale, and a magnetometer hard-/soft-iron correction (a 3-vector offset + 3×3 matrix —
  a `transform` matvec). Calibration parameters are inputs to the driver, not estimated here.
- 2.a.iv — Read a **device-fused orientation quaternion** from a BNO055/BNO08x: parse the
  vendor protocol (BNO08x = **SH-2 / SHTP** over I2C/SPI/UART-RVC), extract the rotation-vector
  report, and **convert to the transform quaternion convention** at the boundary (§1.b.i).
  Surface the report's accuracy/status estimate as sample health (§2.f). **[scope: verify SH-2
  report ids vs datasheet]**
- 2.a.v — Expose, per driver, **which signals are present** (6-DoF vs 9-DoF vs fused) and the
  **measurement frame**, so a consumer can route raw IMU data to `fusion` or consume a fused
  quaternion directly without driver-specific branching.

### 2.b Encoder drivers (incremental and absolute)
Definition: shaft-angle/position sensors across three transport classes (research §A.3); the
reimplementation surface is "small and bounded — edge decode, SPI register reads, BiSS-C frame."
- 2.b.i — Decode **incremental quadrature (A/B)**: a 2-bit Gray-code edge counter that
  increments/decrements on each A/B transition (×1/×2/×4 decode), maintaining a signed count;
  convert counts→angle via counts-per-revolution. Where edge timing comes from `io` GPIO, the
  driver is the state machine over those edges.
- 2.b.ii — Handle the **index (Z) channel**: latch/zero the count on the index pulse to recover
  an absolute reference once per revolution, and report whether the index has been seen.
- 2.b.iii — Read an **absolute magnetic encoder** (AS5048 over SPI, AS5600 over I2C): register
  read of the raw angle, apply zero-offset and direction, emit absolute angle (rad). Surface the
  device's **magnitude/AGC** diagnostic field as health (magnet-out-of-range detection).
  **[scope: verify register map vs datasheet]**
- 2.b.iv — Read a clocked-serial **absolute** encoder: **SSI** (Gray or binary, configurable
  word length) and **BiSS-C** (the open SSI successor — clocked frame with **position + CRC +
  warning/error bits**), validating the CRC and decoding to angle/position. **[scope: verify
  BiSS-C frame layout / CRC polynomial vs spec]**
- 2.b.v — Compute and emit **velocity** from successive position samples + their `transform.Time`
  stamps (finite difference) as an optional derived field, clearly labeled as driver-derived
  (not a separately measured quantity).
- 2.b.vi — Detect and report **fault conditions**: quadrature illegal-transition (both channels
  changing), SSI/BiSS-C CRC failure, and magnetic-encoder magnet fault — as sample status, not
  as silent data corruption.

### 2.c Force/torque & load-cell drivers
Definition: strain-bridge sensors. A 6-axis F/T sensor's six raw strain channels map to a
`Wrench` through the sensor's **6×6 calibration matrix** — a matvec on `transform`/Cajeta linalg
(research §A.4). A load cell is a single-axis bridge over an HX711 24-bit ADC.
- 2.c.i — Read the **six raw strain/gauge channels** from a 6-axis F/T sensor (ATI/Robotous-class)
  over its transport, with per-channel zeroing (bias/tare capture).
- 2.c.ii — Apply the sensor's **6×6 calibration matrix** to the (biased) strain vector to produce
  a `Wrench` `[force(3); torque(3)]` in the sensor flange frame and family units (N, N·m),
  ordering the wrench per the inherited convention (§1.b, transform §1.b.ii). The calibration
  matrix is loaded from the device/config; this package does **not** *compute* the calibration.
- 2.c.iii — Read a **load cell via HX711**: bit-banged 2-wire protocol (DOUT/PD_SCK), 24-bit
  two's-complement sample, selectable gain/channel (clock-pulse count), with tare and a
  scale factor to engineering units (N or kg). **[scope: verify HX711 clocking/gain vs datasheet]**
- 2.c.iv — Report **saturation / over-range** per channel and an overall sensor-health status
  (e.g. bridge disconnected, sample timeout) as part of the sample.

### 2.d LiDAR drivers (2-D serial and 3-D UDP)
Definition: ranging scanners across two transport classes (research §A.5): a 2-D spinning serial
LiDAR (RPLidar) and 3-D UDP packet streams (Velodyne/Ouster). Output is **raw ranges / a point
buffer**; segmentation/mapping is for `estimation`/downstream, not here.
- 2.d.i — Drive an **RPLidar over serial**: issue scan/health/reset commands, parse the
  measurement response (angle, distance, quality per point) into a **2-D scan** (array of
  range-at-angle samples) with a per-scan `transform.Time`. **[scope: verify command set / packet
  format vs protocol doc]**
- 2.d.ii — Parse a **Velodyne/Ouster 3-D UDP** packet stream: reassemble data-block packets into
  a per-revolution **point buffer** (range/azimuth/elevation → x,y,z in the sensor frame),
  using the sensor's beam-angle table, and associate packet timestamps. **[scope: verify packet
  layout / beam-intrinsics source vs datasheet]**
- 2.d.iii — Emit a **point-buffer value type** (flat `[x,y,z(,intensity,ring,time)]` layout over
  a Cajeta tensor) that `io.vision`/downstream can consume — *the buffer format only*; no
  filtering, registration, or feature extraction (those are downstream).
- 2.d.iv — Surface **dropped/out-of-order UDP packets** and serial-frame desync as scan health,
  and expose the scanner's reported **device health/error** status where the protocol provides it.

### 2.e Sonar / ToF rangefinders
Definition: single-point distance sensors over GPIO timing or I2C (research §A.5).
- 2.e.i — Drive an **HC-SR04** ultrasonic rangefinder: emit the trigger pulse, time the echo
  high-duration via `io` GPIO edge timing, convert echo time → range using the speed of sound
  (with an optional temperature input), and emit range (m) + a validity flag (echo timeout →
  no-return). **[scope: verify trigger/echo timing vs datasheet]**
- 2.e.ii — Read a **VL53L0X** I2C time-of-flight sensor: configure ranging mode, read the range
  result register and its **range status**, and emit range (m) plus a quality/confidence field;
  map the device's range-status codes to the package's sample-health vocabulary (§2.f). **[scope:
  verify register/API sequence vs datasheet]**
- 2.e.iii — Expose each rangefinder's **field-of-view / min-max range / measurement frame** as
  static device metadata so a consumer can interpret a single range correctly.

### 2.f Common sensor model (cross-cutting)
Definition: the uniform shape every driver above shares, so consumers and tests are
device-independent.
- 2.f.i — A **`Sample<T>`** value type: payload `T` (vec3, quaternion, scalar angle, `Wrench`,
  scan, point buffer) + `transform.Time` + measurement frame id + **status/health** +
  timestamp-source label (§1.b.iii).
- 2.f.ii — A uniform **device lifecycle**: `open(transport, config) → configure → start → read
  → stop/close`, with deterministic resource release on drop (Cajeta ownership — the family's
  safety pillar), so a failed/unplugged device frees its transport without leak.
- 2.f.iii — A small **health/status vocabulary** (ok / stale / saturated / crc-fail /
  out-of-range / no-return / device-fault / timeout) that every driver maps its device-specific
  error codes into, so safety/fusion code reacts uniformly.
- 2.f.iv — A **`SensorSource<T>` reader interface** (poll and/or callback) common across drivers,
  so `fusion` and tooling bind to the sample stream without naming a concrete device.
- 2.f.v — **Configuration is data**: ranges, rates, bias/calibration tables, CPR, beam tables,
  and zero-offsets are passed in (loaded from config), never hardcoded — so the same driver
  serves many board wirings.

---

## 3. Acceptance criteria (beyond per-feature tests)
- 3.a — **Protocol-codec tests against recorded byte streams.** Per the family pillar for driver
  layers (family spec §3.b), every driver's parse/format path is tested **without hardware** by
  replaying recorded device byte streams / register transcripts / UDP captures and asserting the
  decoded `Sample` (and round-tripping commands the driver emits). Fixtures are checked in.
- 3.b — **Raw-vs-fused split is explicit and tested.** A raw IMU driver emits accel/gyro(/mag)
  and never a fused orientation; a BNO08x driver emits a quaternion already converted to the
  transform convention (§1.b.i). A convention test asserts the boundary conversion (no
  scalar-first/JPL leakage).
- 3.c — **No fusion or estimation logic** ships in this package (§1.c.i): the only math present
  is device-defined scaling, the F/T 6×6 matvec, quadrature/SSI decode, CRC checks, and
  range/echo arithmetic. Anything iterative/probabilistic belongs in `fusion`.
- 3.d — **Board-agnostic, no native deps** (§1.d.ii, family pillar 3.c): drivers touch hardware
  only via `io` handles; the test suite runs in CI on x86-64 and at least one RISC-V target with
  no devices attached.
- 3.e — **No upward / sibling-math dependencies:** imports are limited to `transform` and `io`
  (§1.d); the package never imports `fusion`, `estimation`, `io.vision`, or anything upward.
- 3.f — **Fault surfacing, not silent corruption:** CRC failures, illegal quadrature transitions,
  saturation, echo timeouts, magnet-out-of-range, and dropped packets are reported as sample
  health (§2.f.iii), exercised by malformed-stream fixtures.
- 3.g — **Timestamp/frame discipline:** every emitted sample carries a `transform.Time`, a
  timestamp-source label, and a measurement frame id (§1.b.ii–iii).

## 4. Deliverables
- 4.a — `dev.cajeta.robotica.io.sensor` implementing §2, with the §3 protocol-codec test suite
  and its recorded-stream fixtures.
- 4.b — A **datasheet-verification checklist** resolving every **[scope: verify …]** tag in §2
  to a cited primary datasheet/protocol document, completed before the corresponding driver is
  coded (closes the research §A.2/§J gaps for this package).
- 4.c — A plan at `agents/sensor-plan.md` (authored when this spec is approved), TDD-ordered:
  common sensor model (§2.f) → encoders (§2.b, simplest codec) → raw + fused IMU (§2.a) →
  force/torque + load cell (§2.c) → sonar/ToF (§2.e) → LiDAR (§2.d).

## 5. References (research-grounded)
- IMU raw-vs-fused split (MPU-6050/9250, ICM-20948 raw over I2C/SPI register banks; BNO055/BNO085
  on-chip fusion, BNO085 SH-2/Hillcrest protocol) — research §A.2. **[scope — not verified this
  pass; verify register/report maps vs datasheet]**
- Encoders (quadrature A/B/Z Gray-code edge decode; absolute magnetic AS5048/AS5600 over SPI/I2C;
  industrial SSI / BiSS-C clocked-serial absolute) — research §A.3. **[scope — verify BiSS-C
  frame/CRC vs spec]**
- Force/torque (6-axis → wrench via 6×6 calibration matrix, a linalg matvec; load cell via HX711
  24-bit ADC, bit-banged 2-wire) — research §A.4. **[scope — verify HX711 clocking vs datasheet]**
- LiDAR/sonar/ToF (RPLidar serial; Velodyne/Ouster UDP packet streams; PCD/`PointCloud2` buffer
  formats; HC-SR04 echo-timing GPIO; VL53L0X I2C) — research §A.5. **[scope — verify packet/timing
  formats vs datasheets]**
- Quaternion-convention hazard (pin ONE convention family-wide; convert at the driver boundary) —
  research §A.1 / transform-spec §1.b. **[verified]**
- Fusion math (Madgwick/Mahony, EKF/UKF/error-state KF) is **out of scope** here and lives in
  `fusion` — research §A.1. **[verified that it belongs in `fusion`, not this package]**
- Low-level transports are board-agnostic over Linux device nodes (socketcan/serial/raw
  sockets/device nodes), ISA-agnostic incl. RISC-V — research §A.5 / family spec §1.a.iii. **[scope]**
