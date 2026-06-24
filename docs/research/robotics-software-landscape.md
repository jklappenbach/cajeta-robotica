# Robotics Software Landscape — Reimplementation Surface for a Cajeta-Native Stack

Scope: protocols, standards, architectures, and algorithms behind the libraries that
drive actuators/servos and run robots on single-board computers (Orange Pi, Raspberry
Pi, RISC-V). The emphasis is the *reimplementation surface* — the on-the-wire formats,
register maps, state machines, and framework abstractions a Cajeta-native port must
reproduce — not the libraries' names. Only the claims below survived 3-vote adversarial
verification; sections marked *(unverified / scope note)* are connective tissue, not
fact-checked findings, and are flagged as such.

---

## 1. Actuator / drive classes and their wire protocols

### 1.1 Smart serial-bus servos — DYNAMIXEL Protocol 2.0 (primary, high confidence)

DYNAMIXEL is the best-documented serial-bus servo protocol and the cleanest first target
for a Cajeta-native driver because the entire control surface is a byte-exact packet
format over TTL/RS-485.

**Instruction packet layout (Protocol 2.0):**

```
[0xFF][0xFF][0xFD][0x00][ID][LEN_L][LEN_H][INSTR][PARAM...][CRC_L][CRC_H]
 \________ 4-byte header/reserved prefix ______/
```

- Header = `0xFF 0xFF 0xFD`, Reserved = `0x00` (together a 4-byte prefix).
- Packet ID = 1 byte (0–252 device id, `0xFE` = broadcast).
- Length = 2 bytes, little-endian, value = **(number of parameters + 3)** — the `+3`
  covers the Instruction byte plus the two CRC bytes.
- Instruction = 1 byte; Parameters = variable; CRC = 16-bit.

**Byte stuffing:** whenever the pattern `0xFF 0xFF 0xFD` appears inside the payload, an
extra `0xFD` is inserted (→ `0xFF 0xFF 0xFD 0xFD`) so it cannot be mistaken for a header.
The 16-bit CRC covers every byte from the header through the parameters (excluding the
CRC field itself).

**Instruction byte values:** Ping=`0x01`, Read=`0x02`, Write=`0x03`, Reg Write=`0x04`,
Action=`0x05`, Factory Reset=`0x06`, Reboot=`0x08`, Sync Read=`0x82`, Sync Write=`0x83`,
Bulk Read=`0x92`, Bulk Write=`0x93`. (Full set also includes Clear=`0x10`, Control Table
Backup=`0x20`, Status=`0x55`, Fast Sync Read=`0x8A`, Fast Bulk Read=`0x9A`.)

**Sync vs Bulk addressing — the key throughput primitive:** *Sync* Read/Write target
**multiple devices at the same control-table address and the same data length** in one
packet; *Bulk* Read/Write target **multiple devices at different addresses / lengths**.
A Cajeta driver that wants to command a whole arm in one bus transaction must implement
Sync Write (and Sync/Fast-Sync Read for feedback).

Source: ROBOTIS e-Manual, https://emanual.robotis.com/docs/en/dxl/protocol2/ (primary).

*Scope note (unverified):* Feetech/SCS/STS and Waveshare bus servos use a similar
header+ID+length+instruction+checksum frame but were not independently verified here;
treat them as protocol-family relatives of DYNAMIXEL Protocol 1.0 for planning. The
DYNAMIXEL Protocol 1.0 layout itself was not separately verified.

### 1.2 BLDC / FOC drivers (CAN-bus register protocols, primary, high confidence)

These drivers close the field-oriented-control (FOC) current/velocity/position loops on
the drive MCU; the host's job is to write setpoints and read feedback over CAN. That makes
the **register map plus the CAN frame encoding** the entire reimplementation surface.

**moteus / mjbots (CAN-FD register protocol):**
- Wire layer is **CAN-FD** (1 Mbit nominal / 5 Mbit data bitrate). Each CAN-FD frame
  carries one or more *subframes*; register writes use opcodes `0x00/0x04/0x08/0x0c`
  for int8/int16/int32/float, encoding a varuint register count, a start register, then
  values. Register values are therefore sent in a multiplexed subframe format with
  documented per-type scaling (e.g. current LSB = 1 A / 0.1 A / 0.001 A for
  int8/int16/int32), **not raw bytes**.
- FOC setpoint register map: `0x000` = Mode, `0x020` = Position command, `0x021` =
  Velocity command, `0x023` = Kp scale, `0x025` = Maximum torque, `0x028` = Velocity
  limit. (Domain detail: position `0x020` and max-torque `0x025` use NaN to mean
  "use current position / use phase current limit".)
- Source: https://github.com/mjbots/moteus/blob/main/docs/reference.md +
  https://mjbots.github.io/moteus/ (primary).

**ODrive (CANSimple protocol):**
- Supports classic CAN 2.0b **and experimental CAN-FD** with separately configurable
  nominal (`can.config.baud_rate`) and data-phase (`can.config.data_baud_rate`) bitrates
  plus bit-rate switching (`tx_brs`). Autobaud detection (firmware ≥0.6.11) scans
  10k/40k/125k/250k/500k/1 Mbps when `baud_rate=0`. FD support is on the newer MCU ODrives
  (Pro/S1/Micro), not legacy v3.x, and does not yet exploit the larger FD payload.
- Each axis is addressed by a configurable `ODrive.Axis.CanConfig.node_id` (packed into
  the CAN arbitration ID alongside the command id); the protocol has a "Discovery &
  Addressing" mechanism.
- Register-style message set includes: Set_Input_Pos, Set_Input_Vel, Set_Input_Torque,
  Get_Encoder_Estimates, Set_Controller_Mode, Set_Limits, Get_Iq, Get_Temperature,
  Get_Bus_Voltage_Current, Heartbeat, Estop, Get_Error, Clear_Errors (plus Get_Version,
  RxSdo/TxSdo register read-write channel, Set_Axis_State, Set_Traj_*, Set_Pos_Gain,
  Set_Vel_Gains, etc.).
- Source: https://docs.odriverobotics.com/v/latest/manual/can-protocol.html (primary).

**VESC (UART/CAN protocol):**
- UART packet frame: start byte (`2` = short packet, `3` = long packet), one or two
  length bytes, payload, two-byte CRC (CRC16-CCITT over the payload), stop byte.
- API model: commands via setter functions (`bldc_interface_set_current(10.0)`);
  telemetry is asynchronous — register an rx callback with
  `bldc_interface_set_rx_value_func`, then request data with
  `bldc_interface_get_values` (the callback fires with an `mc_values` struct: voltage,
  temperature, current, RPM, duty, faults, when the response arrives).
- Source: http://vedder.se/2015/10/communicating-with-the-vesc-using-uart/ (primary,
  author = VESC creator Benjamin Vedder).

*Scope note (unverified):* SimpleFOC (Arduino library + Commander serial ASCII interface)
and the FOC math itself (Clarke/Park transforms, current PI loops, SVPWM) were not
independently fact-checked here; they are the algorithmic layer the above drivers
implement internally. A Cajeta driver that only talks to ODrive/moteus/VESC does **not**
need to reimplement FOC — those boards close the loop. Reimplementing FOC natively is a
separate, larger effort (the "smart driver firmware" altitude).

### 1.3 Hobby PWM + steppers *(scope note — not independently verified)*

PCA9685 (16-channel I2C PWM, standard ~50 Hz / 1–2 ms servo pulse), StepStick/TMC2209
step-dir drivers, GRBL and Klipper firmware, and Linux GPIO/PWM access were part of the
charter but produced no surviving verified claims. Flagged as a gap to research before
committing a Cajeta plan. Of note: the PCA9685 I2C register interface and the TMC2209
UART register set are both small, well-bounded reimplementation targets if pursued.

### 1.4 Industrial CAN / EtherCAT — the determinism tier (primary, high confidence)

This is where the richest, most stable *standards* surface lives, and where a Cajeta-native
implementation has the most long-term leverage (open masters are C and aging).

**EtherCAT masters:**
- **SOEM (Simple Open EtherCAT Master)** is a C *library* (not an application, ~97% C)
  for developing EtherCAT MainDevices (masters), "specifically designed for real-time
  communication in embedded systems." It is the natural direct port reference for a
  Cajeta-native master. Real-world determinism still depends on the host OS/RTOS
  (PREEMPT_RT / Xenomai) and tuning — SOEM targets the requirement but does not alone
  guarantee hard determinism. Source: https://github.com/OpenEtherCATsociety/SOEM (primary).
- **IgH / EtherLab EtherCAT Master** is architected as a **Linux kernel module** (Linux
  2.6+), implemented per **IEC 61158-12** (the EtherCAT standard). Applications run either
  as kernel modules using the Application Interface directly, or as userspace programs via
  the EtherCAT library / RTDM. For NIC access it ships **native EtherCAT drivers that
  operate the hardware without interrupts**, plus a **generic driver** that reuses the
  lower layers of the standard Linux network stack for any kernel-supported chip; devices
  the master declines fall through to the normal kernel network stack (the generic driver
  binds a `SOCK_RAW` packet socket only to accepted interfaces). Source:
  https://docs.etherlab.org/ethercat/1.5/pdf/ethercat_doc.pdf (primary).

**EtherCAT Distributed Clocks (DC) — the synchronization algorithm:** all DC-capable
slave clocks synchronize to a **reference clock = the local clock of the first DC-capable
slave**. Each slave clock is a **64-bit nanosecond-resolution counter** (ESC System Time,
register `0x0910`) that starts at zero on power-up. After **offset** and
**propagation-delay / drift compensation**, the master can reach **nanosecond synchrony**
across the bus. A Cajeta master must implement these three compensation steps to drive
cyclic-synchronous motion. Source: same IgH doc (primary).

---

## 2. The motion-profile standard — CiA 402 / IEC 61800-7 (primary, high confidence)

CiA 402 (a.k.a. DSP 402 V2.0) is the single most important *spec* for a Cajeta motion
stack: it is a network-agnostic, fully enumerated state machine. Implementing it once buys
you Maxon EPOS, most EtherCAT servo drives, and CANopen drives.

**What it standardizes:** the functional behavior of controllers for **servo drives,
frequency inverters, and stepper motors** — exactly the device classes a robot stack must
drive. It is internationally standardized as **IEC 61800-7**: the functional/FSA part
**CiA 402-2 → IEC 61800-7-201**, and the PDO-mapping part **CiA 402-3 → IEC 61800-7-301**.
Underlying base standards: **CiA DS301 V4.01** (CANopen application layer) and
**CiA DS402 V2.0** (drives & motion-control device profile). The cyclic-synchronous modes
derive from the **ETG "Implementation Guideline for the CiA 402 Drive Profile."**

**The device-control finite state automaton (FSA) — the precise reimplementation spec:**
- Host writes **Controlword, object `0x6040`** to drive transitions; reads
  **Statusword, object `0x6041`** for the current state. The state determines which
  commands are accepted and whether high power is applied.
- States: NOT READY TO SWITCH ON, SWITCH ON DISABLED, READY TO SWITCH ON, SWITCHED ON,
  OPERATION ENABLED, QUICK STOP ACTIVE, FAULT REACTION ACTIVE, FAULT. The motor moves only
  in OPERATION ENABLED.
- Commands selected by specific Controlword bit patterns (only bits 0,1,2,3,7 change
  state): Shutdown=`xxxxxxxx0xxxx110`, Switch On=`xxxxxxxx0xxxx111`,
  Disable Voltage=`xxxxxxxx0xxxxx0x`, Quick Stop=`xxxxxxxx0xxxx01x`,
  Enable Operation=`xxxxxxxx0xxx1111`, Fault Reset=`xxxxxxxx1xxxxxxx` (rising edge).

**Operating modes:** selected via **`0x6060` "Modes of operation"**, acknowledged via
**`0x6061` "Modes of operation display"**. Codes: 1=Profile Position, 3=Profile Velocity,
6=Homing, 8=Cyclic Synchronous Position (CSP), 9=Cyclic Synchronous Velocity (CSV),
10=Cyclic Synchronous Torque (CST). A robot joint controller typically uses CSP/CST at the
EtherCAT cycle rate.

**Transport bindings:**
- Over EtherCAT via **CoE (CANopen over EtherCAT)**, with real-time determinism from the
  EtherCAT **distributed-clock** principle.
- Over **CANopen (CiA DS301 V4.01)** using PDO/SDO/SYNC (SYNC object COB-ID `0x80`), node
  guarding, and emergency (EMCY) messages, with real-time determinism from the SYNC object.

Sources: https://www.can-cia.org/can-knowledge/cia-402-series-canopen-device-profile-for-drives-and-motion-control
and AMK DS402 interface PDF
https://www.amk-motion.com/amk-dokucd/dokucd/en/content/resources/pdf-dateien/pdk_204628_ds402_standard_en_.pdf
(both primary/normative).

---

## 3. Middleware / control frameworks

### 3.1 ros2_control hardware_interface — the framework reimplementation target (primary, high confidence)

ros2_control's `hardware_interface` is the cleanest abstraction to mirror in Cajeta because
it cleanly separates "talk to hardware" from "compute control."

- **Exactly three hardware component types**: **System** (complex multi-DOF), **Actuator**
  (simple 1-DOF), **Sensor** (read-only). Selected via the `type` attribute on the
  `<ros2_control>` tag in URDF (e.g. `type="actuator"`).
- **Interface hierarchy by element**: `<joint>` tags carry both **command and state**
  interfaces; `<sensor>` tags carry **state interfaces only**; `<gpio>` tags can carry
  **both**.
- **Controller Manager loop**: a sequential **read → update → write** real-time loop
  (reads states from hardware, updates controllers, writes commands), with loop frequency
  set by the read-only `update_rate` parameter, **default 100 Hz**.

A Cajeta reimplementation target is thus: a component model with these three types, a
typed command/state interface registry keyed by joint/sensor/gpio, and a fixed-rate
read-update-write executor. Sources:
https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html
and https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html
(primary).

### 3.2 Rest of the middleware charter *(scope note — not independently verified)*

MoveIt 2, Nav2, micro-ROS, the DDS implementations (CycloneDDS, Fast-DDS) and the
IEEE/OMG DDS standard, rclcpp/rclpy/rclrs, and the lightweight alternatives (LinuxCNC/
Machinekit, zenoh/MQTT transports, and the math libraries Eigen/Pinocchio/KDL/Drake for
kinematics & dynamics) were in the charter but yielded no surviving verified claims here.
They are real and relevant but should be researched before being written into a Cajeta
plan. **Getting-started recommendation (editorial, low confidence):** for the Cajeta
robotics entry point, target the **ros2_control hardware_interface contract** (§3.1) as
the framework API and a **single fixed-rate read-update-write loop** rather than the full
ROS 2 / DDS stack — it is the smallest abstraction that still interoperates with the
ecosystem.

---

## 4. Real-time + compute architecture (partly verified)

Two patterns, both supported by the evidence above:

1. **SBC high-level + MCU/smart-driver closes the fast loop.** The SBC writes setpoints
   and reads feedback over CAN/CAN-FD/UART; the drive (ODrive/moteus/VESC, §1.2) runs FOC
   at kHz internally. This is the lowest-risk pattern for a Cajeta stack: the host only
   implements packet/register codecs (verified surfaces) and a ~100 Hz–1 kHz command loop,
   never hard-real-time current control.

2. **Hard real-time on the SBC.** EtherCAT master (SOEM userspace or IgH kernel module,
   §1.4) on a PREEMPT_RT kernel with CPU isolation, driving CiA 402 drives (§2) over
   distributed clocks at the cycle rate. Verified facts: SOEM/IgH target embedded real-time
   but determinism depends on OS/RTOS tuning; DC gives nanosecond bus sync.

*Scope note (unverified):* specific PREEMPT_RT latency numbers, CPU-isolation/`isolcpus`
tuning, and IRQ-threading realities on Pi/Orange Pi/RISC-V boards were not fact-checked.

---

## 5. Per-board guide *(scope note — NOT independently verified)*

The per-board charter (Raspberry Pi 5/4, Orange Pi 5 / RK3588, RISC-V boards: Milk-V
Duo/Mars, StarFive VisionFive 2) — recommended OS images, PREEMPT_RT status, CAN support
(MCP2515/SPI vs native CAN-FD), GPIO/PWM/I2C/SPI library access, ROS 2 build status, and
the RISC-V robotics-ecosystem gaps — produced **no surviving verified claims**. This is the
largest evidence gap in the report. Do not write board-specific guidance into a Cajeta plan
until this is researched with primary sources (board vendor docs, kernel `defconfig`s,
ros.org build farm status, the `python-can`/`socketcan` driver matrix).

Architecturally, the verified protocol surfaces above are board-agnostic: a Cajeta CAN/
CAN-FD codec, EtherCAT master, and serial-bus servo driver depend only on `socketcan` /
raw packet sockets / a UART, all of which exist on Linux regardless of ISA — so the RISC-V
gap is likely in the *higher* libraries (ROS 2, DDS, Drake/Pinocchio prebuilts), not the
low-level wire protocols a Cajeta stack would own.

---

## 6. Where a Cajeta-native implementation has the most leverage

Ranked by verified-surface clarity and ecosystem gap:

1. **CiA 402 / IEC 61800-7 state machine + modes (§2)** — fully enumerated normative spec
   (objects `0x6040/0x6041/0x6060/0x6061`, 8 states, 7 commands, bit patterns, mode
   codes). Highest leverage: one implementation drives a huge class of industrial drives.
2. **DYNAMIXEL Protocol 2.0 (§1.1)** — byte-exact, self-contained, Sync/Bulk batching;
   ideal first deliverable and unit-testable without hardware (codec + CRC + byte stuffing).
3. **EtherCAT master + Distributed Clocks (§1.4)** — open references (SOEM/IgH) are aging C;
   a memory-safe Cajeta master is a genuine gap-filler. Largest effort.
4. **moteus / ODrive / VESC CAN+UART codecs (§1.2)** — small register-map/subframe codecs;
   high value because the hard FOC work stays on the drive.
5. **ros2_control hardware_interface contract (§3.1)** — mirror the 3-type component model
   and read-update-write loop to interoperate with the ROS ecosystem without adopting DDS.

---

## Sources (all primary unless noted)

- DYNAMIXEL Protocol 2.0 — https://emanual.robotis.com/docs/en/dxl/protocol2/
- moteus reference / CAN protocol — https://github.com/mjbots/moteus/blob/main/docs/reference.md ; https://mjbots.github.io/moteus/
- ODrive CAN protocol — https://docs.odriverobotics.com/v/latest/manual/can-protocol.html
- VESC UART protocol — http://vedder.se/2015/10/communicating-with-the-vesc-using-uart/
- SOEM — https://github.com/OpenEtherCATsociety/SOEM
- IgH/EtherLab EtherCAT Master — https://docs.etherlab.org/ethercat/1.5/pdf/ethercat_doc.pdf
- CiA 402 series — https://www.can-cia.org/can-knowledge/cia-402-series-canopen-device-profile-for-drives-and-motion-control
- AMK DS402 (DSP 402 V2.0) interface — https://www.amk-motion.com/amk-dokucd/dokucd/en/content/resources/pdf-dateien/pdk_204628_ds402_standard_en_.pdf
- ros2_control hardware_interface — https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html
- ros2_control Controller Manager — https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html
