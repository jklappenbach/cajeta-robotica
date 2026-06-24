# cajeta-robotica — Family Specification (`dev.cajeta.robotica.*`)

Top-level specification for the cajeta-robotica library family. This document defines **what
the family is** and decomposes it into packages; each package has its own spec under
`docs/specs/<package-path>/` that enumerates its use cases in full. Grounded in the landscape
research under [`docs/research/`](../research/) (Part 1 = actuation plane; Part 2 =
perception → AI).

---

## 1. Definition

`cajeta-robotica` is a **Cajeta-native robotics stack**: the protocols, algorithms, and
formats needed to *engineer and operate actuators/servos and run a robot* — reimplemented in
Cajeta rather than bound to C/C++/ROS. It is a **family of focused external libraries over one
small shared foundation**, published under the reverse-DNS namespace `dev.cajeta.robotica.*`.
It is **not** part of the Cajeta language stdlib.

### 1.a Goals
- 1.a.i — One coherent type system from the **GPU matmul kernel that runs a policy down to the
  FOC setpoint on the wire** — no Python↔C++↔firmware seam.
- 1.a.ii — Reuse Cajeta's existing **numpy/tensor/linalg/FFT/GPU** substrate; the math-heavy
  layers (transforms, fusion, dynamics, estimation, control, inference) compound on it.
- 1.a.iii — Run on Linux single-board computers (Raspberry Pi, Orange Pi, RISC-V) — the
  low-level wire protocols depend only on socketcan / serial / raw sockets / device nodes and
  are ISA-agnostic.
- 1.a.iv — Make **safety** a first-class, statically-checkable concern using Cajeta's
  ownership/borrow + deterministic-drop model.

### 1.b Architecture & packaging
- 1.b.i — **Foundation:** `dev.cajeta.robotica.transform` (SE(3)/SO(3) Lie math, quaternions,
  the time-stamped frame tree, core geometry/units types). Everything depends on it; it depends
  on nothing in the family. Pin **one** twist-order + quaternion convention here, forever.
- 1.b.ii — **Focused libs** depend on the foundation and (sparingly) each other; **never
  upward**. Heavy-dependency members (vision, sim, AI inference) sit at the edges so importing
  `transform` never drags in a camera or GPU-inference stack.
- 1.b.iii — **DCE rationale:** Cajeta's link-time dead-code elimination means a robot importing
  only `transform` + a fusion filter ships *only that code*. Breadth is free at deployment, so
  the family unifies **types** (one SE(3), one quaternion) while partitioning **dependencies
  and versioning** across libs — monolith ergonomics, focused-library footprint.
- 1.b.iv — **Spec ↔ package mirror:** `docs/specs/io/sensor/` ↔ `dev.cajeta.robotica.io.sensor`,
  etc.

### 1.c Layer map (data flow: sense → estimate → plan → control → actuate, with infra + safety cross-cutting)
```
                 ┌────────────────────────── ai (policy inference) ───────────────────────────┐
  io.sensor ─┐   │                                                                             │
  io.vision ─┼─▶ fusion ─▶ estimation ─▶ planning ─▶ control ─▶ motor ─▶ (drive closes FOC loop)
  io (buses) ┘        ▲         ▲            ▲          ▲           ▲
                      └ kinematics / dynamics ┘        │           │
        transform (SE(3)/SO(3) frame tree, core types) ── underlies ALL of the above
        log (MCAP/Foxglove observability) · sim (MJCF) · safety (e-stop/watchdog/fault) ── cross-cutting
```

---

## 2. Packages (feature decomposition)

Each subsection names a package, its responsibility, representative use cases (full
enumeration in the package's own spec), and the reference implementation(s) it models per the
research. Leverage tags: **[foundation]** start-here · **[verified]** research-confirmed
reimplementation surface · **[scope]** lower-confidence, re-research before its plan.

### 2.a `transform` — Lie-group foundation **[foundation] [verified]**
Responsibility: SE(3)/SO(3) and quaternion math, exp/log/adjoint/Jacobians, the time-stamped
SE(3) **frame tree** (parent→child transforms with temporal interpolation), and shared
geometry/units/time types.
- 2.a.i — Compose, invert, and interpolate rigid-body poses.
- 2.a.ii — Convert between rotation representations (quaternion / matrix / axis-angle / twist)
  under one fixed convention.
- 2.a.iii — "Where was frame A relative to B at time *t*?" (tf2-style buffered lookup).
- Models: micro-Lie-theory (arXiv:1812.01537), Sophus/manif, ROS tf2. *Build first.*

### 2.b `io` — buses & transport **[scope]**
Responsibility: portable access to the physical transports — serial/UART, CAN/CAN-FD
(socketcan), EtherCAT, I2C, SPI, GPIO/PWM, USB — that sensors and motors ride on.
- 2.b.i — Open a CAN-FD socket, send/receive frames with hardware/software timestamps.
- 2.b.ii — Half-duplex RS-485 transactions (the serial-bus-servo substrate).
- 2.b.iii — I2C/SPI register read/write for sensor ICs.

### 2.c `io.sensor` — sensor drivers **[scope]**
Responsibility: drivers for IMUs (raw + on-chip-fused), encoders (quadrature, absolute
magnetic, SSI/BiSS-C), force/torque sensors, LiDAR (2D/3D), sonar/ToF rangefinders.
- 2.c.i — Stream calibrated accel/gyro/mag (or a fused quaternion from a BNO08x).
- 2.c.ii — Decode quadrature counts / read an absolute-encoder angle.
- 2.c.iii — Map a 6-axis F/T bridge to a wrench via its calibration matrix.
- 2.c.iv — Parse an RPLidar/Velodyne stream into a point cloud.

### 2.d `io.vision` — cameras & depth **[scope]**
Responsibility: image/point-cloud input — V4L2/UVC/GenICam cameras, RealSense/OAK-D depth,
point-cloud buffer formats.
- 2.d.i — Capture frames from a V4L2 device with a chosen pixel format.
- 2.d.ii — Stream aligned depth+color and produce an organized point cloud.

### 2.e `motor` — actuator/drive drivers **[verified]**
Responsibility: command drives and read feedback — DYNAMIXEL Protocol 2.0, ODrive/moteus/VESC
register protocols, PWM hobby servos, steppers, and CiA 402 (CANopen/EtherCAT) drives.
- 2.e.i — Sync-write a position/torque command to a chain of bus servos in one transaction.
- 2.e.ii — Stream setpoints to a FOC drive over CAN-FD; read encoder/current feedback.
- 2.e.iii — Drive a CiA 402 servo through its state machine (controlword/statusword, CSP/CST).
- Models: DYNAMIXEL 2.0, moteus/ODrive/VESC, CiA 402 / IEC 61800-7 (research Part 1).

### 2.f `fusion` — attitude & state fusion **[verified core]**
Responsibility: turn raw inertial/aiding measurements into orientation/state — Madgwick,
Mahony, and the EKF/UKF/error-state-KF tier.
- 2.f.i — Fuse gyro+accel(+mag) into a drift-corrected orientation quaternion.
- 2.f.ii — Run a 15-state error-state KF fusing IMU + aiding (odom/GPS/vision).
- Models: Madgwick/Mahony (x-io), robot_localization. *Second build target.*

### 2.g `estimation` — SLAM & localization **[verified core]**
Responsibility: factor-graph estimation and localization building blocks — pose-graph
optimization, odometry, scan matching, particle-filter localization.
- 2.g.i — Build and optimize a pose graph (MAP = sparse Cholesky/QR on Cajeta linalg).
- 2.g.ii — Maintain an `odom→map` estimate from odometry + loop closures.
- Models: GTSAM; g2o/Ceres (scope).

### 2.h `kinematics` — FK/IK **[scope]**
Responsibility: forward/inverse kinematics, geometric/analytic Jacobians, URDF/SRDF parsing.
- 2.h.i — Compute end-effector pose from joint angles (and the inverse).
- 2.h.ii — Build a kinematic chain from a parsed URDF.

### 2.i `dynamics` — rigid-body dynamics **[verified]**
Responsibility: spatial-algebra RBD — RNEA (inverse), ABA (forward), CRBA (joint-space
inertia), plus analytical derivatives for optimization/MPC.
- 2.i.i — Inverse dynamics: joint torques for a desired motion.
- 2.i.ii — Analytical derivatives, finite-difference-verifiable without hardware.
- Models: Pinocchio (RNEA/ABA/CRBA + analytical derivatives).

### 2.j `planning` — motion planning & trajectories **[verified]**
Responsibility: sampling-based planning (RRT/RRT*/PRM), online jerk-limited trajectory
generation, and collision detection (GJK/EPA).
- 2.j.i — Plan a collision-free configuration-space path.
- 2.j.ii — Generate a time-optimal, jerk-limited trajectory to a (possibly moving) target.
- 2.j.iii — Query distance/penetration between convex shapes.
- Models: OMPL RRT*, Ruckig OTG, FCL/coal GJK-EPA.

### 2.k `control` — control above the drive **[verified core]**
Responsibility: PID, impedance/admittance, operational-space control, MPC, whole-body control,
and the QP solver they share.
- 2.k.i — Render a virtual impedance (mass-spring-damper) at the end effector.
- 2.k.ii — Solve a per-tick WBC/MPC QP mapping task objectives + constraints to joint torques.
- Models: ProxQP / eiquadprog / qpmad (QP); impedance/WBC (scope).

### 2.l `sim` — simulation interface **[scope]**
Responsibility: parse robot models and drive/observe a simulator (MuJoCo MJCF first;
URDF/SDF/USD as relatives).
- 2.l.i — Load an MJCF/URDF model and step a simulation with applied controls.
- 2.l.ii — Read simulated sensors back through the same `io.sensor` interfaces.

### 2.m `log` — telemetry & observability **[verified]**
Responsibility: record and stream robot state — the MCAP container and the Foxglove WebSocket
protocol.
- 2.m.i — Write a typed, self-describing MCAP log of channels/messages.
- 2.m.ii — Stream live state to Foxglove Studio over the WebSocket protocol.
- Models: MCAP spec, Foxglove ws-protocol v1.

### 2.n `safety` — the safety plane **[scope, differentiator]**
Responsibility: e-stop chains, hardware/software watchdogs, deterministic fault reaction,
and functional-safety conformance hooks.
- 2.n.i — A watchdog that forces drives to a safe state on a missed deadline.
- 2.n.ii — Power-&-force-limiting (ISO/TS 15066) constraints injected into `control`.
- 2.n.iii — Provable, GC-pause-free resource release on the fault path (Cajeta ownership).
- Standards (scope): ISO 10218-1/-2, ISO/TS 15066, IEC 61508, FSoE, CIP Safety.

### 2.o `ai` — embodied-AI inference **[forward-looking]**
Responsibility: on-device learned-policy inference and integration with the imitation/RL +
VLA ecosystem, riding Cajeta's GPU/tensor stack.
- 2.o.i — Run a learned policy (vision+proprioception → action) on-device.
- 2.o.ii — Load/deploy a policy from the LeRobot/VLA ecosystem; emit actions into `control`.
- Models: LeRobot, SmolVLA / VLA.

---

## 3. Cross-cutting requirements
- 3.a — **One convention, everywhere.** A single quaternion + twist-order convention defined in
  `transform` and used by every dependent package (the bug class that fragments ROS).
- 3.b — **Hardware-free testability.** Math layers (transform, fusion, dynamics, planning,
  control) ship unit tests that run with no hardware (identity/round-trip/finite-difference
  checks); driver layers ship protocol-codec tests against recorded byte streams.
- 3.c — **Board-agnostic core.** No package in the sense→estimate→plan→control→actuate spine may
  depend on a non-portable native lib; heavy/native deps are confined to `io.vision`, `sim`, `ai`.

## 4. Deliverables
- 4.a — This family spec + a per-package spec under each `docs/specs/<package-path>/`.
- 4.b — A dependency-ordered plan per package under `agents/` (authored when its spec is approved).
- 4.c — First implementation milestone: the `transform` foundation (§2.a), test-first.

## 5. Open scope (re-research before the relevant package's plan)
Per-board/RISC-V support matrix; sensor register/wire detail (`io.sensor`/`io.vision`);
EKF/UKF internals; impedance/MPC/WBC specifics; sim model-format detail; and the entire
functional-safety standards surface. Tracked in the research reports' gap sections.
