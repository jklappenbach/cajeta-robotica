# `dev.cajeta.robotica.io` — Specification

The **bus & transport driver layer** of cajeta-robotica: portable, board-agnostic access to
the physical wires that sensors and motors ride on — serial/UART (incl. RS-485), CAN/CAN-FD,
EtherCAT, I2C, SPI, GPIO/PWM, and raw USB. It depends only on `transform` (for time/units
types) and Cajeta's `numpy`/`tensor` byte buffers; it depends on nothing else in the family
and is depended on by `io.sensor`, `io.vision`, and `motor`. See the family spec
[`../robotica-spec.md`](../robotica-spec.md) §2.b and the research
[`../../research/robotics-software-landscape.md`](../../research/robotics-software-landscape.md)
§1.1/§1.2/§1.4/§2 and
[`../../research/robotics-stack-part2-perception-to-ai.md`](../../research/robotics-stack-part2-perception-to-ai.md)
§A.2–A.5.

---

## 1. Definition

`io` provides **typed, owned handles over Linux device nodes and sockets** plus the **framing /
bit-timing codecs** each transport needs, so the higher layers exchange *timestamped byte
buffers* with hardware without writing fd/ioctl/socket plumbing. It is deliberately thin: it
moves and frames bytes and stamps them with time, but it does **not interpret** them — the
meaning of a register, a CAN payload, or a servo packet lives in `io.sensor`, `motor`, and
`io.vision`. Every transport's codec is split from its I/O so the codec is **unit-testable
against recorded byte streams** with no hardware (the driver-layer test pillar, family §3.b).

### 1.a Capability summary
- 1.a.i — A mockable **device handle / socket seam** (open/read/write/ioctl/poll over a Linux
  fd) that every transport is built on, so codecs replay against recorded byte streams.
- 1.a.ii — **Serial/UART** with full line configuration and **half-duplex RS-485** direction
  control (the smart-serial-bus-servo substrate, research §1.1).
- 1.a.iii — **CAN / CAN-FD** over SocketCAN: classic and FD frames, bit-rate switching (BRS),
  hardware/software timestamps, acceptance filters, bus-state/error reporting (research §1.2).
- 1.a.iv — **EtherCAT** transport: a raw packet socket carrying EtherCAT datagrams, working
  counter, and the Distributed-Clock synchronization primitives (research §1.4) — **[scope]**.
- 1.a.v — **I2C** and **SPI** register access — the read/write-register transaction pattern
  sensor ICs need (research §A.2–A.5) — **[scope]**.
- 1.a.vi — **GPIO / PWM** line I/O, timestamped edge events, and echo-timing capture (research
  §1.3, §A.5) — **[scope]**.
- 1.a.vii — Raw **USB** bulk/interrupt/control transfers over usbfs — **[scope]**.

### 1.b Conventions (INHERITED — do not redefine here)
> `io` **inherits the family's normative conventions verbatim from `transform` §1.b** and adds
> no geometric conventions of its own. Quaternion (Hamilton, scalar-last), twist/wrench order
> (translation-first), active-transform composition, and SI units (radians/meters/seconds) are
> **defined in `transform` and referenced, never redefined here**. The few additions below are
> *transport* conventions only.
- 1.b.i — **Timestamps** are `transform.Time` (nanosecond integer base; monotonic for latency
  /scheduling, wall for logging). Hardware (SO_TIMESTAMPING/SIOCGSTAMP) timestamps are
  preferred when the kernel/NIC supplies them; otherwise a software receive timestamp is taken
  as close to the syscall as possible and **flagged as software-sourced**.
- 1.b.ii — **Byte order on the wire is per-transport and explicit**: SocketCAN frames and
  payloads are little-endian host-struct layouts; multi-byte register fields are passed as
  raw byte buffers and **`io` does not byte-swap** — endianness of a field is the decoding
  layer's responsibility (`io.sensor`/`motor`).
- 1.b.iii — **Buffers are caller-owned borrows on the hot path** (Cajeta ownership): a send
  borrows the caller's buffer for the duration of the call; a receive fills a caller-provided
  buffer and returns the filled length. No hidden allocation per frame.
- 1.b.iv — A **handle owns its fd**; deterministic drop closes it (and, for RS-485/GPIO,
  returns the line to a safe/inert state). This is the safety-plane guarantee at the wire
  (family §1.a.iv, §2.n.iii).

### 1.c Non-goals
- 1.c.i — **No payload interpretation.** Register maps, servo packet formats (DYNAMIXEL,
  moteus/ODrive/VESC), and CiA 402 state machines belong to `motor`; sensor register decode
  belongs to `io.sensor`. `io` frames and timestamps bytes only.
- 1.c.ii — **No camera/video capture.** V4L2/UVC/GenICam image pipelines are `io.vision`
  (heavy/native deps are confined there, family §3.c); `io`'s USB surface (§2.h) is raw
  transfer only.
- 1.c.iii — **No native/heavy third-party deps.** Everything binds Linux device nodes/sockets
  directly (SocketCAN, `/dev/i2c-*`, `/dev/spidev*`, GPIO character device, `AF_PACKET`,
  usbfs) and is therefore ISA-agnostic (family §1.a.iii, research §5). No libusb, no SOEM/IgH
  linkage, no vendor SDKs.
- 1.c.iv — **No real-time guarantees of its own.** `io` exposes the primitives (DC sync,
  hardware timestamps, non-blocking poll); hard determinism is an OS/RTOS concern (PREEMPT_RT),
  out of scope here (research §4, §1.4).

---

## 2. Features

### 2.a Device handle / socket seam (the portability & test seam)
Definition: a thin owned wrapper over a Linux fd exposing `open`/`read`/`write`/`ioctl`/
`poll`/`close`, behind an interface a **recorded-stream fake** can implement — the single point
that makes every transport above it hardware-free testable.
- 2.a.i (use case) — Open a device node or socket by path/spec, obtaining an owned handle whose
  drop closes the fd deterministically (§1.b.iv).
- 2.a.ii — Blocking and non-blocking (`O_NONBLOCK`) reads/writes of a caller-owned byte buffer,
  returning bytes transferred (§1.b.iii).
- 2.a.iii — Readiness/`poll` with a timeout (deadline from `transform.Duration`) over one or
  many handles; report ready/timeout/error distinctly.
- 2.a.iv — Issue a typed `ioctl` (request code + in/out struct buffer) — the substrate for I2C,
  SPI, GPIO, and CAN-config operations.
- 2.a.v — Substitute a **replay fake** that serves a recorded byte stream and records writes,
  so any transport's codec runs in CI against captured traffic.
- Acceptance: a codec built on the seam round-trips a recorded capture identically; a dropped
  handle is observably closed; no leak under error paths.

### 2.b Serial / UART (incl. RS-485 half-duplex)
Definition: a configured `termios`-style serial port and the half-duplex direction control the
smart-serial-bus-servo substrate requires (research §1.1; the DYNAMIXEL/Feetech TTL+RS-485
family, and TMC2209-style single-wire UART, §1.3).
- 2.b.i — Open a port with explicit line config: baud rate, data bits, parity, stop bits, and
  flow control (none/RTS-CTS); set raw mode (no canonical processing).
- 2.b.ii — Read/write byte buffers with inter-byte and total read timeouts (`VMIN`/`VTIME` or
  poll-based), so a fixed-length response frame can be awaited deterministically.
- 2.b.iii — **Half-duplex RS-485 transaction**: assert transmit-enable (driver-enable line) for
  the write, flush (drain) the TX, then release to receive — exposed as one transact-and-read
  call so the turnaround is race-free (research §1.1, family §2.b.ii). Support both kernel
  `RS485` ioctl gating and a GPIO-driven DE/RE line.
- 2.b.iv — Query/flush input & output queues; report framing/parity/overrun errors.
- 2.b.v — Apply a software receive timestamp (§1.b.i) to a completed read.
- Acceptance: a recorded half-duplex request/response pair replays byte-exactly through the
  seam fake (no hardware); turnaround ordering (TX-enable → drain → RX) is enforced.

### 2.c CAN / CAN-FD (SocketCAN)
Definition: a bound SocketCAN interface exchanging classic CAN 2.0 and CAN-FD frames, the host
side of the BLDC/FOC drive register protocols (research §1.2: moteus CAN-FD 1 Mbit nominal /
5 Mbit data with bit-rate switching; ODrive CANSimple classic + experimental FD).
- 2.c.i — Bind a `PF_CAN`/`SOCK_RAW` socket to a named interface (e.g. `can0`); optionally
  enable CAN-FD frames (`CAN_RAW_FD_FRAMES`).
- 2.c.ii — Send a frame: 11-bit or 29-bit (extended) ID, up to 8 (classic) or 64 (FD) data
  bytes, with the **BRS** (bit-rate switch) and ESI flags for FD (research §1.2).
- 2.c.iii — Receive frames with **hardware timestamps when available** (SO_TIMESTAMPING),
  software timestamp otherwise (§1.b.i), filling a caller buffer.
- 2.c.iv — Install **acceptance filters** (ID + mask lists) and an error-frame mask so only
  relevant arbitration IDs / bus errors are delivered.
- 2.c.v — Report **bus state** (error-active / error-passive / bus-off) and recover from
  bus-off; expose TX/RX error counters where the driver provides them.
- 2.c.vi — **[scope]** Configure bitrate / data-bitrate / sample point via netlink (`IFLA`),
  or document it as an out-of-band `ip link` precondition — exact per-controller CAN-FD timing
  must be re-researched per board (research §1.2 caveat, §5).
- Acceptance: frame encode/decode (ID, flags, DLC↔length, FD vs classic) round-trips through a
  recorded `candump`-style capture with no hardware; BRS/ESI/extended-ID bits map correctly.

### 2.d EtherCAT transport **[scope]**
Definition: the raw transport beneath an EtherCAT MainDevice — a packet socket carrying
EtherCAT frames (datagrams/commands + working counter) and the Distributed-Clock sync
primitives — modeled on SOEM/IgH but reimplemented native-free (research §1.4). The *master
state machine and CoE/CiA 402 mapping live in `motor`*; `io` owns only the wire.
- 2.d.i — Open an `AF_PACKET`/`SOCK_RAW` socket bound to a NIC and send/receive raw Ethernet
  frames (EtherCAT EtherType **0x88A4**) without interrupts on the hot path (research §1.4;
  the EtherType value itself is **[scope]** — verify against the standard before coding).
- 2.d.ii — Frame/parse an EtherCAT datagram: command type, addressing (auto-increment /
  configured-station / logical), offset, length, working-counter field; concatenate multiple
  datagrams into one frame.
- 2.d.iii — **[scope]** Distributed-Clock support: read each slave's 64-bit nanosecond ESC
  System Time (register `0x0910`), and apply the three compensation steps — **offset**,
  **propagation-delay**, and **drift** — against the reference clock (the first DC-capable
  slave) to reach bus-wide nanosecond synchrony (research §1.4, verified algorithm; concrete
  register/timing handling to be re-researched).
- 2.d.iv — **[scope]** Cyclic exchange primitive: send a process-data frame and collect its
  working counter each cycle, the substrate a CSP/CST control loop (in `motor`) drives.
- Acceptance: datagram framing (command/address/WKC) round-trips against a recorded EtherCAT
  capture; DC compensation math is finite-difference/round-trip checkable on synthetic clock
  samples with no hardware. **Most of §2.d is [scope] — re-research before its plan (family §5).**

### 2.e I2C register access **[scope]**
Definition: the read-register / write-register transaction pattern sensor ICs expose over
`/dev/i2c-*` (research §A.2–A.5: MPU-6050/9250, ICM-20948 register banks; VL53L0X; AS5600;
PCA9685 PWM expander §1.3). Bytes only — the register *meaning* is `io.sensor`/`motor`.
- 2.e.i — Open an I2C bus and address a device (7-bit; **[scope]** 10-bit), via the SMBus/I2C
  ioctl interface.
- 2.e.ii — **Write-then-read** a register: send a register-pointer write, then a repeated-start
  read of N bytes (the canonical register fetch), as one atomic `I2C_RDWR` transaction.
- 2.e.iii — Write a register: pointer byte(s) + N data bytes in one transaction; support 8- and
  16-bit register addresses (banked devices).
- 2.e.iv — Burst read/write a contiguous register block (e.g. a full IMU sample) in one
  transaction.
- 2.e.v — Report NAK / arbitration-loss / bus errors distinctly.
- Acceptance: a recorded register-access transcript (pointer-write + repeated-start read)
  replays byte-exactly through the seam fake. **[scope] — re-research device specifics.**

### 2.f SPI register access **[scope]**
Definition: full-duplex SPI transfers over `/dev/spidev*` with the R/W-bit register convention
absolute/magnetic encoders and high-rate IMUs use (research §A.2–A.5: AS5048 SPI, ICM-20948
SPI; SSI/BiSS-C clocked-serial absolute encoders are a related, **[scope]** target).
- 2.f.i — Open a spidev node and configure **mode (CPOL/CPHA → modes 0–3)**, bits-per-word,
  max clock (Hz), and bit order.
- 2.f.ii — Perform a full-duplex transfer (simultaneous TX/RX of equal-length buffers); chain
  multiple transfers with per-segment CS-change/delay control (`SPI_IOC_MESSAGE`).
- 2.f.iii — Register read/write convention helper: set/clear the **R/W bit in the MSB** of the
  command byte, then transfer address + payload (the common encoder/IMU pattern).
- 2.f.iv — Burst multi-byte register read (auto-increment) in one CS assertion.
- 2.f.v — **[scope]** Clocked-serial absolute frames (SSI / BiSS-C) where a board exposes them
  over SPI-like clocking — frame structure to be re-researched (research §A.3).
- Acceptance: a recorded full-duplex transfer (MOSI/MISO pair) replays byte-exactly; mode and
  R/W-bit helper produce the expected command bytes. **[scope].**

### 2.g GPIO / PWM / timing capture **[scope]**
Definition: digital line I/O, timestamped edge events, and PWM generation over the Linux GPIO
character device and the PWM sysfs/chardev interfaces (research §1.3 PCA9685-class PWM,
hobby-servo ~50 Hz / 1–2 ms pulse; §A.5 HC-SR04 echo-timing, HX711 bit-banged 2-wire).
- 2.g.i — Request a GPIO line as input or output with bias (pull-up/down/none) and active-level;
  read/write its level; release returns the line inert (§1.b.iv).
- 2.g.ii — Subscribe to **edge events** (rising/falling/both) delivered with a kernel timestamp
  (§1.b.i) — the substrate for quadrature index pulses and pulse-width capture.
- 2.g.iii — **Echo-timing capture**: measure the high-pulse width between two edges (the HC-SR04
  range pattern, research §A.5), reported as a `transform.Duration`.
- 2.g.iv — PWM channel control: set period, duty cycle, polarity, and enable/disable — the
  hobby-servo / ESC pulse substrate (research §1.3); express servo pulses as 1–2 ms within a
  ~20 ms period.
- 2.g.v — **[scope]** Bit-banged 2-wire clocking (HX711-style 24-bit ADC) as a timing-driven
  GPIO pattern — re-research before committing.
- Acceptance: a recorded edge-event sequence replays with correct ordering and pulse-width
  arithmetic; line drop is observed inert. **[scope].**

### 2.h USB (raw transfer) **[scope]**
Definition: raw bulk / interrupt / control transfers over the usbfs device nodes
(`/dev/bus/usb/*`), native-free — for serial-over-USB bridges and packetized devices that are
*not* cameras (UVC capture is `io.vision`, §1.c.ii).
- 2.h.i — Open a USB device by bus/address (or VID/PID match) via usbfs; claim an interface.
- 2.h.ii — Submit a **control transfer** (setup packet + data stage) for device configuration.
- 2.h.iii — Submit **bulk / interrupt** transfers (IN/OUT) on an endpoint, with timeout, filling
  or draining a caller buffer.
- 2.h.iv — Release the interface and reset on drop; report stall/timeout/disconnect distinctly.
- Acceptance: a recorded control + bulk exchange replays through the seam fake. **[scope] —
  whether usbfs alone suffices vs. a small native shim is an open question (§4 / §6).**

---

## 3. Acceptance criteria (beyond per-feature tests)
- 3.a — **Codec ≠ I/O.** Every transport's framing/encoding is a pure function over byte
  buffers, separated from the fd/socket, and tested **against recorded byte streams** with no
  hardware (family §3.b, the driver-layer pillar).
- 3.b — **Board-agnostic / native-free.** No package member links a vendor SDK, libusb, or an
  EtherCAT master library; all transports bind Linux device nodes/sockets directly and build on
  x86-64 and at least one RISC-V target (family §3.c, research §5).
- 3.c — **Convention conformance.** All timestamps are `transform.Time`; software-sourced
  timestamps are flagged (§1.b.i); `io` performs no field byte-swapping (§1.b.ii). A test
  asserts these at the API boundary.
- 3.d — **No upward dependencies.** `io` imports only `transform` and Cajeta `numpy`/`tensor`;
  nothing else in `dev.cajeta.robotica.*` (family §1.b.ii).
- 3.e — **Deterministic resource & safety release.** Every handle owns its fd; drop closes it
  and returns RS-485/GPIO/PWM lines to a safe/inert state, on both normal and fault paths
  (§1.b.iv, family §2.n.iii) — verified with leak/owner tests.
- 3.f — **Allocation discipline.** Send/receive borrow caller buffers; no per-frame heap
  allocation on the hot path (§1.b.iii).
- 3.g — **Scope honesty.** §2.d (EtherCAT), §2.e/§2.f (I2C/SPI device specifics), §2.g (GPIO/
  PWM timing), §2.h (USB), and CAN-FD per-board bit timing (§2.c.vi) are **[scope]** and carry
  re-research notes; no false precision is committed to a plan before its primary-source pass
  (family §5).

## 4. Deliverables
- 4.a — `dev.cajeta.robotica.io` implementing §2, with the §3 test suite (codec-vs-recorded-
  stream tests for every transport, plus owner/leak and convention-conformance tests).
- 4.b — A small library of **recorded byte-stream fixtures** (RS-485 servo transaction,
  `candump`/CAN-FD capture, I2C register transcript, SPI MOSI/MISO pair, GPIO edge sequence,
  EtherCAT datagram capture) used as the golden inputs for 4.a.
- 4.c — A dependency-ordered plan at `agents/io-plan.md` (authored when this spec is approved),
  TDD-ordered: device-handle seam (§2.a) → serial/RS-485 (§2.b) → CAN/CAN-FD (§2.c) → I2C/SPI
  (§2.e/§2.f) → GPIO/PWM (§2.g) → EtherCAT (§2.d) → USB (§2.h), with the [scope] transports
  gated on their re-research notes.

## 5. References (research-grounded)
- DYNAMIXEL Protocol 2.0 over TTL/RS-485 — the half-duplex serial-bus-servo substrate (§2.b).
  Research §1.1, ROBOTIS e-Manual. **[verified]** (1.0/Feetech relatives **[scope]**).
- moteus CAN-FD (1 Mbit nominal / 5 Mbit data, BRS) and ODrive CANSimple (classic + experimental
  FD) — the CAN/CAN-FD host surface (§2.c). Research §1.2. **[verified]** (per-board bit timing
  **[scope]**).
- EtherCAT transport: SOEM/IgH model, `SOCK_RAW`/`AF_PACKET` NIC access, IEC 61158-12, and the
  Distributed-Clock algorithm — reference clock = first DC-capable slave, 64-bit ns ESC System
  Time (`0x0910`), offset/propagation-delay/drift compensation (§2.d). Research §1.4.
  **[verified algorithm; transport-codec & EtherType details [scope]]**.
- CiA 402 / IEC 61800-7 CoE and CANopen (PDO/SDO/SYNC) transport bindings — consumed *above* `io`
  by `motor`; informs §2.c/§2.d framing. Research §2. **[verified]**.
- I2C/SPI sensor register banks (MPU-6050/9250, ICM-20948, AS5048/AS5600, VL53L0X), GPIO echo
  timing (HC-SR04), HX711 bit-bang, PCA9685 / hobby-PWM, TMC2209 UART (§2.e–§2.g). Research
  §A.2–A.5 and §1.3. **[scope — re-research per datasheet]**.
- Board-agnosticism of socketcan / raw sockets / device nodes across ISAs incl. RISC-V (§1.c.iii,
  §3.b). Research §5. **[verified architectural claim; per-board matrix [scope]]**.
