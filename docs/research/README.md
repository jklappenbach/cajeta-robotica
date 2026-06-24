# research/robotics — index

Research toward "taking Cajeta into robotics": the protocols, standards, algorithms, and
framework abstractions a Cajeta-native actuator/robot stack would reimplement. Two reports,
read in order:

## 1. [robotics-software-landscape.md](robotics-software-landscape.md) — the actuation plane
Actuator/drive **wire protocols** and the low-level control surface:
- DYNAMIXEL Protocol 2.0 (serial-bus servo packet codec); moteus/ODrive/VESC CAN+UART
  register protocols; EtherCAT masters (SOEM/IgH) + Distributed Clocks.
- The **CiA 402 / IEC 61800-7** motion-profile state machine (controlword/statusword/modes).
- The **ros2_control hardware_interface** framework contract (3 component types, read-update-write loop).
- SBC-vs-MCU and hard-real-time architecture patterns.

## 2. [robotics-stack-part2-perception-to-ai.md](robotics-stack-part2-perception-to-ai.md) — everything above the drive
- **A** Sensing/perception: Madgwick/Mahony AHRS fusion (+ EKF/UKF scope), IMU/encoder/F-T/
  camera/depth/LiDAR interfaces.
- **B** Estimation & world model: **SE(3)/SO(3) Lie foundation** (the shared primitive),
  tf2 transform trees, **GTSAM** factor-graph SLAM.
- **C** Kinematics/dynamics/planning: **Pinocchio** RBD + analytical derivatives, **OMPL**
  RRT*, **Ruckig** jerk-limited OTG, GJK/EPA collision.
- **D** Control above the drive: **QP solvers** (ProxQP class), impedance/MPC/WBC.
- **E** Dev infra: **MCAP** + **Foxglove** IO wire formats, **MuJoCo/MJCF** sim, behavior trees.
- **F** Safety plane (ISO 10218 / ISO-TS 15066 / IEC 61508 / FSoE) — a Cajeta ownership-model differentiator.
- **G–I** Leverage ranking, the **library recommendation + DCE packaging shape** (a *family*
  of focused external libs over one small SE(3) **foundation** lib, none in stdlib), and the
  **embodied-AI frontier** (LeRobot / VLA — one language from matmul kernel to FOC setpoint).

## Confidence
Both reports tag findings **[verified]** (survived 3-vote adversarial check vs. a primary
source) vs. **[scope note]** (planning-grade, not fact-checked — verify before a Cajeta spec).
Part 1 gaps: per-board/RISC-V guide, hobby PWM/steppers, ROS 2/DDS middleware. Part 2 gaps:
sensor-hardware register/wire detail, EKF/UKF internals, the safety standards, listed in its §J.

## Suggested first deliverable
The **SE(3)/SO(3) Lie foundation lib** (Part 2, §B.1 / §G #1): pure linalg on Cajeta's
existing matrix/tensor stack, unit-testable without hardware, and the dependency root of the
whole family.
