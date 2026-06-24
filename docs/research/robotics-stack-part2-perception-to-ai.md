# Robotics Stack, Part 2 — Perception → Estimation → Planning → Control → AI

Companion to [robotics-software-landscape.md](robotics-software-landscape.md) (the
*actuation plane*: DYNAMIXEL 2.0, moteus/ODrive/VESC, EtherCAT/CiA 402, ros2_control).
This part covers everything **above the drive**: sensing/perception, the estimation &
world-model layer, kinematics/dynamics/planning, control algorithms above the drive,
developer infrastructure, the safety plane, and the embodied-AI frontier — through the
**reimplementation lens**: the algorithms, formats, and standards a Cajeta-native stack
would reproduce, and where Cajeta's existing numpy/tensor/linalg/FFT/GPU substrate
compounds.

**Confidence convention** (same as Part 1): claims tagged **[verified]** survived this
pass's 3-vote adversarial check against the cited primary source (25 sources fetched, 25
claims verified, 0 killed). Sections tagged **[scope note]** are connective tissue filled
from general domain knowledge and the fetched-but-not-top-ranked sources — relevant and
load-bearing for planning, but **not** independently fact-checked here; verify before
writing into a Cajeta spec. The explicit gap list is in §J.

---

## A. Sensing / perception

### A.1 Sensor-fusion algorithms — the highest-leverage Cajeta targets [verified core]

These are pure math over IMU samples: self-contained, unit-testable **without hardware**
(feed recorded accel/gyro/mag, assert quaternion output), and they ride straight on
Cajeta's linalg. This is the best first perception deliverable.

- **[verified] Madgwick** — gradient-descent quaternion AHRS. Treats orientation
  estimation as minimizing the error between the measured gravity/magnetic field and the
  field predicted by the current quaternion; one gradient-descent step per sample, fused
  with gyro integration by a single tunable gain β. Quaternion state (4 floats), no matrix
  inverse — cheap enough for kHz on an MCU. Source: x-io.co.uk (Madgwick's own release).
- **[verified] Mahony** — complementary/DCM filter posed as a **PI controller on SO(3)**:
  the orientation error (from accel/mag vs. predicted) drives a proportional + integral
  correction of the gyro bias. Also quaternion/DCM state, no inverse. Source: x-io.co.uk.
- **[scope note] EKF / UKF / error-state (indirect) KF.** The full probabilistic tier the
  charter asked for did not produce a surviving top-ranked finding this pass. Structure to
  reproduce: a 15–16-state error-state KF (orientation, velocity, position, gyro bias,
  accel bias) with an IMU-driven *predict* and aiding *updates* (mag, GPS, vision); the
  **error-state** formulation keeps the filter linear around the current estimate and
  sidesteps quaternion-normalization singularities — the standard for VIO/INS. `robot_localization`
  (ROS) is the reference EKF/UKF node (sensor-agnostic fusion of odom/IMU/GPS into one
  `odom→base_link` estimate). Reimplementation surface: predict/update equations + a
  quaternion-aware state manifold. Heavy on Cajeta's linalg (Cholesky/matrix ops already
  built). *Not verified this pass.*

> **Quaternion-convention hazard:** pin ONE convention (Hamilton vs. JPL, scalar-first vs.
> scalar-last) across the whole family from day one. Cross-library convention mismatch is a
> top source of silent orientation bugs. (Related: the Sophus/manif twist-order hazard in §B.1.)

### A.2–A.5 Sensor hardware interfaces [scope note — not verified this pass]

The charter's hardware-driver detail did not survive verification; treat the following as
planning-grade, to be re-researched against datasheets before a Cajeta plan:

- **IMUs.** MPU-6050/9250 and ICM-20948 expose **raw** accel/gyro(/mag) over I2C/SPI
  register banks — the host runs the fusion (§A.1). **BNO055 / BNO085 do on-chip fusion**
  (output a fused quaternion directly; BNO085 uses the SH-2/“hillcrest” protocol) — so a
  Cajeta driver for those is a *transport+parse* job, not a fusion job. The split (raw vs.
  fused IMU) is the key driver-design decision.
- **Encoders.** Incremental **quadrature (A/B/Z)** = a 2-bit Gray-code edge counter + index;
  absolute magnetic (**AS5048/AS5600**, AMT) over SPI/I2C, and industrial **SSI / BiSS-C**
  (clocked serial absolute). Reimplementation surface is small and bounded (edge decode,
  SPI register reads, BiSS-C frame).
- **Force/torque.** 6-axis F/T (ATI, Robotous) deliver raw strain → wrench via a **6×6
  calibration matrix** (a matvec on Cajeta linalg); load cells via HX711 (24-bit ADC,
  bit-banged 2-wire).
- **Cameras / depth / LiDAR / sonar.** **V4L2** (Linux capture ioctls) and **USB UVC** for
  webcams; **GenICam / GigE Vision** for industrial cameras (a standardized feature/transport
  model). Depth: librealsense (RealSense), DepthAI (OAK-D), stereo/ToF, point clouds as
  **PCD / `PointCloud2`**. LiDAR: RPLidar (serial), Velodyne/Ouster (**UDP packet** streams).
  Sonar/ToF rangefinders: HC-SR04 (echo-timing GPIO), VL53L0X (I2C). These are mostly
  *transport + buffer-format* work, board-agnostic over Linux device nodes.

---

## B. Estimation & world model

### B.1 Lie-group SE(3)/SO(3) — the shared foundation primitive [verified]

- **[verified]** SE(3)/SO(3) Lie-group math is the **single primitive shared across
  transforms, SLAM, and dynamics** — implement it once, everything above depends on it. The
  micro-Lie-theory reference (arXiv:1812.01537) supplies the closed-form **exp/log maps and
  Jacobians**; **Sophus** (`se3.hpp`) pins a concrete SE(3) memory layout (a unit quaternion
  + translation = 7 params for 6 DoF) and a **translation-first twist** ordering.
- **[verified] Convention hazard:** Sophus orders the twist **translation-first**, while
  **manif orders it rotation-first** — opposite. A Cajeta foundation lib must pick one and
  document it loudly; mixing them silently corrupts every Jacobian.

This is the **foundation lib** (§H): pure linalg, fully unit-testable without hardware
(`exp(log(X)) == X`, adjoint identities), and the natural extension of Cajeta's existing
matrix/quaternion work. Strongest first-deliverable candidate.

### B.2 Transform trees — tf2 model [scope note + primary source]

tf2 (ROS) maintains a **time-stamped tree of SE(3) frames**: a buffer of timestamped
parent→child transforms, with **temporal interpolation** (slerp on rotation, lerp on
translation) so a consumer can ask "where was frame A relative to B at time *t*." The
reimplementation surface: a frame-graph + a per-edge time-series ring buffer + interpolated
lookup + the SE(3) algebra from §B.1. A clean, bounded, high-value Cajeta target that sits
directly on the foundation lib. Source: ROS tf2 concept docs (secondary).

### B.3 Factor-graph estimation / SLAM — GTSAM model [verified]

- **[verified] GTSAM** is the estimation reference: BSD-licensed C++ **factor graphs** where
  MAP inference **maximizes the product of factors**, which reduces to a **sparse linear
  solve (Cholesky/QR)** — i.e. it lands squarely on Cajeta's existing sparse-linear-algebra
  surface. It uses SE(2)/SE(3) exp-maps (so it depends on §B.1). This is the backbone for
  pose-graph SLAM, visual-inertial odometry, and calibration.
- **[scope note]** g2o and Ceres are the alternative factor-graph/NLLS engines; ICP/scan-
  matching, particle-filter/AMCL localization, and odometry are the front-end building
  blocks feeding the graph. *Not verified this pass.*

---

## C. Kinematics, dynamics & planning

### C.1 Rigid-body dynamics — Pinocchio model [verified]

- **[verified] Pinocchio** is the dynamics reference. It implements the classical
  spatial-algebra trio — **RNEA** (inverse dynamics), **ABA** (forward dynamics), **CRBA**
  (joint-space inertia) — and its distinguishing feature is **closed-form ANALYTICAL
  derivatives** of those algorithms (RSS 2018, Carpentier/Mansard), which is exactly what
  gradient-based control/optimization (MPC, trajectory optimization) needs. Crucially it is
  **finite-difference-verifiable without hardware** (compare analytical vs. numerical
  Jacobians in a unit test). Heavy linalg/spatial-vector math — compounds on Cajeta's tensor
  stack. Sources: Pinocchio README + the analytical-derivatives paper.
- **[scope note]** KDL/orocos (simpler FK/IK + Jacobians) and Drake (multibody + optimization)
  are the alternatives; forward/inverse kinematics, the geometric/analytic Jacobian, and
  **URDF/SRDF parsing** (the model front-end) round out this layer.

### C.2 Motion planning — OMPL [verified]

- **[verified] OMPL `RRT*`** is the canonical **asymptotically-optimal sampling-based
  planner** (the configuration-space RRT/RRT*/PRM family). Reimplementation surface: a
  C-space abstraction (sample / nearest-neighbor / steer / collision-check callbacks) plus
  the tree-rewiring logic — algorithmic, hardware-free, unit-testable on synthetic spaces.
  Source: OMPL RRT* class docs + the optimal-planning literature.

### C.3 Online trajectory generation — Ruckig [verified]

- **[verified] Ruckig** is the clean **OTG (online trajectory generation)** target:
  **time-optimal, jerk-limited** profiles, and notably the first to handle a **non-zero
  target acceleration** online (arXiv:2105.04830). Well-specified, self-contained (input:
  current + target kinematic state + limits; output: the next position/velocity/accel
  setpoint), trivially unit-testable. *Note the open-source "Community" edition limits
  non-zero-target-acceleration / multi-DoF time-sync vs. the commercial "Pro" edition* — a
  Cajeta-native implementation of the full algorithm is therefore a genuine gap-filler.

### C.4 Collision detection — FCL / GJK-EPA [scope note + primary source]

The **GJK** (distance/intersection) + **EPA** (penetration depth) pair over convex shapes,
plus broadphase, is the collision core (FCL, and its successor **coal**). GJK/EPA are
compact, well-documented geometric algorithms (support functions + simplex evolution) —
a bounded Cajeta target. Source: coal GJK/EPA docs (secondary).

---

## D. Control algorithms above the drive

- **[verified] QP is the recurring high-frequency core.** Whole-body control, MPC, some
  planning, and some estimation all reduce to solving a **quadratic program every control
  tick**. Four solver families exist; the 2025 robotics benchmarks favor **ProxQP**
  (augmented-Lagrangian) — with the safety-relevant property that it **converges to the
  closest feasible QP** when the problem is infeasible — while dense small-WBC problems are
  solved **sub-millisecond** by **eiquadprog/qpmad**. A Cajeta-native QP solver (dense for
  WBC, sparse for MPC) is a high-leverage, linalg-heavy target. Sources: IEEE T-RO 2025
  benchmark + a 2025 solver review *(single studies, one possibly co-authored by the ProxQP
  group — treat the ranking as directional).*
- **[scope note]** Above the QP sit **PID** (+ tuning), **impedance/admittance control** (the
  essential contact/manipulation primitive: render a virtual mass-spring-damper at the end
  effector), **operational-/task-space control**, **MPC**, and **whole-body control** (a QP
  that maps task objectives + constraints to joint torques). These are the algorithmic
  payload a Cajeta control lib would host on top of the QP solver and the §C dynamics.
  *Structures known, not verified this pass.*

---

## E. Developer infrastructure

### E.1 Logging + visualization — MCAP & Foxglove [verified]

- **[verified] MCAP** is a clean, self-describing log-container spec: **8 magic bytes at
  start and end**, **ID-keyed Schema / Channel / Message** records (any serialization). A
  bounded, well-specified IO target — a Cajeta MCAP reader/writer makes the whole stack
  introspectable and interoperable with existing tooling. Source: mcap.dev/spec.
- **[verified] Foxglove WebSocket protocol v1** (subprotocol `foxglove.websocket.v1`):
  JSON control frames with an `op` field + binary frames with a **1-byte opcode**, carrying
  encodings `json` / `protobuf` / `ros1` / `cdr`. Implementing it streams live Cajeta-robot
  state into Foxglove Studio (the modern rviz/rqt replacement) for free. Source: foxglove
  ws-protocol spec.

These two are **interop wire formats, not math** — implement them and the Cajeta stack
plugs into the existing observability ecosystem without adopting ROS/DDS.

### E.2 Simulation — MuJoCo / MJCF [primary source]

**MuJoCo** is now **Apache-2.0** and its **MJCF** XML model format is fully documented
(bodies/joints/geoms/actuators/sensors tree). It is the de-facto sim for RL/embodied-AI
(fast, differentiable-adjacent, contact-rich). A Cajeta stack should at minimum **parse
MJCF and speak to a MuJoCo instance**; a native sim is a much larger effort. Gazebo/Ignition
(SDF), Isaac Sim (USD), and PyBullet are the alternatives. Source: MuJoCo modeling docs.

### E.3 Behavior orchestration — behavior trees [primary source]

**BehaviorTree.CPP** defines an **XML tree model** (control nodes: Sequence/Fallback/etc.;
leaf Action/Condition nodes) over a **blackboard** (a typed key-value context for passing
data between nodes). Compact, well-specified, and the standard for robot task autonomy — a
clean Cajeta target. Source: behaviortree.dev XML-format docs.

**[scope note]** Robot-description formats — **URDF/SRDF/Xacro** (ROS), **MJCF** (MuJoCo),
and increasingly **OpenUSD** (NVIDIA) — are the model front-ends a Cajeta stack must parse;
URDF first.

---

## F. Safety plane [scope note — not verified this pass]

The functional-safety charter produced no surviving verified finding (primary standards are
paywalled/ISO). Planning-grade summary to re-research:

- **ISO 10218-1/-2** (industrial robot safety), **ISO/TS 15066** (collaborative robots —
  the **power-&-force-limiting / PFL** biomechanical limits), **IEC 61508** (SIL functional
  safety), and the safety fieldbuses **FSoE (Safety-over-EtherCAT, ETG)** and **CIP Safety**.
- Practical primitives: hardware + software **watchdogs**, dual-channel **e-stop chains**,
  deterministic **fault reaction** (the CiA 402 FAULT states in Part 1 are the drive-side
  hook), and bounded-latency shutdown.

> **Cajeta differentiator:** this is the layer where Cajeta's **ownership/borrow model and
> deterministic drop** are a genuine, marketable advantage — provable resource release on a
> fault path, no GC pause in a safety loop, and statically-checked aliasing for the e-stop
> chain. Worth making an explicit design pillar, not an afterthought.

---

## G. Leverage ranking — best Cajeta-native targets

Ranked by (verified-surface clarity) × (compounds on Cajeta's numpy/linalg/GPU) ×
(unit-testable without hardware):

1. **SE(3)/SO(3) Lie foundation (§B.1)** [verified] — the shared primitive everything else
   needs; pure linalg; `exp/log/adjoint` identity tests. **Start here.**
2. **AHRS sensor fusion — Madgwick/Mahony (§A.1)** [verified] — tiny, hardware-free-testable,
   immediately useful; gateway to the EKF/UKF tier.
3. **Ruckig-style jerk-limited OTG (§C.3)** [verified] — self-contained, well-specified, and
   the open-source originals are feature-limited (real gap to fill).
4. **tf2-style transform tree (§B.2)** — frame graph + interpolation on the §B.1 foundation.
5. **QP solver (§D)** [verified] — the high-frequency core under WBC/MPC; heavy linalg.
6. **MCAP + Foxglove IO (§E.1)** [verified] — interop wire formats; make the stack observable.
7. **Pinocchio-style RBD + analytical derivatives (§C.1)** [verified] — largest math payload;
   compounds hard on the tensor stack; enables MPC/optimization. Bigger effort.
8. **GTSAM-style factor graphs (§B.3)** [verified] — sparse-solve estimation backbone.
9. **OMPL-style sampling planner (§C.2)** [verified]; **GJK/EPA collision (§C.4)** — algorithmic, bounded.

---

## H. Library recommendation & packaging shape

**Recommendation (verified-synthesis, the workflow independently reached the same shape you
proposed):** build a **family of focused external libraries over one small shared foundation
lib — none in the language stdlib.**

- **Foundation:** `cajeta.robotics.lie` (or `.transform`) — SE(3)/SO(3)/quaternions, the
  time-stamped frame tree, core geometry/units types. Models **manif + Sophus** (pick and
  document ONE twist-order convention, §B.1). Everything depends on this; nothing depends
  upward.
- **Focused libs on top**, each modeling a named reference: fusion (Madgwick/Mahony →
  `robot_localization` for the EKF tier), estimation (**GTSAM**), dynamics (**Pinocchio**),
  planning (**OMPL** + **Ruckig**), control (**ProxQP**-class QP + impedance/WBC), and the IO
  libs (**MCAP** + **Foxglove**). Heavy-dependency members (vision, MuJoCo sim, the inference
  layer) live at the **edges** so importing transforms never drags in a camera or CUDA stack.

**On DCE (your question):** [verified-synthesis, medium confidence] Cajeta's link-time
dead-code elimination **removes the binary-size argument** that forced C/C++/ROS to
fragment — a robot importing only `lie` + a fusion filter ships only that code even from a
broad library. **But DCE does not erase** the other three monolith costs: **compile time**,
**API cohesion/versioning churn**, and **transitive heavy/native deps** (vision, sim, CUDA).
Those three argue for the **family-over-foundation** shape, not one blob — which is exactly
your preference. So: keep the *types* unified (one SE(3), one quaternion convention, one
pose — the coherence ROS's `tf`/`tf2` schism never had) by routing them all through the
shared foundation, while keeping *dependencies* partitioned across separately-versioned libs.
DCE is what lets the foundation be generous without taxing small deployments (incl. RISC-V,
since the low-level math has no native deps).

---

## I. The embodied-AI frontier [primary sources; forward-looking]

The strategic reason Cajeta-in-robotics is more than a port: the field's center of gravity
is **on-device learned policies**.

- **LeRobot** (HuggingFace) is the emerging open framework for real-world imitation/RL and
  policy sharing — datasets, pretrained policies, and a deployment path. Source: github.com/huggingface/lerobot.
- **VLA (vision-language-action)** models map camera+instruction → actions; **SmolVLA** is a
  compact, consumer-hardware-targeted VLA (HuggingFace), and the space is moving fast
  (arXiv:2511.05642). Sources: HF SmolVLA blog + the VLA paper.

**Why Cajeta is uniquely coherent here:** you already own the GPU/tensor/numpy substrate.
A native actuator + transform + control stack (Parts 1–2) on top of that substrate means
**one language and one type system from the matmul kernel that runs the policy down to the
FOC setpoint on the wire** — no Python↔C++↔firmware boundary, no serialization tax across
the inference→control seam. That single-language, GPU-to-actuator story is the differentiated
pitch; the foundation + family above is the substrate that makes it real.

---

## Sources

Verified (primary unless noted):
- AHRS Madgwick/Mahony — https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
- Micro-Lie theory — https://arxiv.org/pdf/1812.01537 ; Sophus SE3 — https://github.com/strasdat/Sophus/blob/main/sophus/se3.hpp
- GTSAM — https://gtsam.org/tutorials/intro.html
- Pinocchio — https://github.com/stack-of-tasks/pinocchio ; analytical derivatives (RSS 2018) — https://laas.hal.science/hal-01866228/document
- OMPL RRT* — https://ompl.kavrakilab.org/classompl_1_1geometric_1_1RRTstar.html ; optimal planning — https://arxiv.org/abs/2105.04830
- Ruckig OTG — https://arxiv.org/abs/2105.04830
- QP solvers (T-RO 2025) — https://scaron.info/publications/tro-2025.html ; solver review — https://arxiv.org/html/2510.21773v1
- MCAP — https://mcap.dev/spec ; Foxglove ws-protocol — https://github.com/foxglove/ws-protocol/blob/main/docs/spec.md
- MuJoCo MJCF — https://mujoco.readthedocs.io/en/stable/modeling.html
- BehaviorTree.CPP XML — https://www.behaviortree.dev/docs/learn-the-basics/xml_format/
- LeRobot — https://github.com/huggingface/lerobot ; SmolVLA — https://huggingface.co/blog/smolvla ; VLA survey — https://arxiv.org/pdf/2511.05642

Supporting (secondary/blog): ahrs.readthedocs.io (Madgwick), mwrona.com (attitude EKF),
Adafruit BNO085 fusion, deepwiki AHRS/coal GJK-EPA, ROS tf2 concept docs.

## J. Explicit gaps (re-research before a Cajeta spec)

IMU register maps; encoder SSI/BiSS-C; F/T calibration; camera/depth/LiDAR wire formats;
**EKF/UKF/error-state-KF + robot_localization** internals; tf2 implementation details;
FCL/GJK-EPA specifics; MPC/impedance/WBC beyond the QP core; sim formats (MJCF/USD detail);
URDF/SRDF parsing; and the **entire safety plane** (ISO 10218, ISO/TS 15066, IEC 61508,
FSoE/CIP Safety). These are scope-noted above, not verified.
