# cajeta-robotica

A **Cajeta-native robotics stack** — the protocols, algorithms, and formats needed to engineer
and operate actuators/servos and run a robot, reimplemented in [Cajeta](https://github.com/jklappenbach/cajeta)
rather than bound to C/C++/ROS. It is a **family of focused libraries over one small shared
foundation**, published under the package namespace `dev.cajeta.robotica.*`.

> **Status: design.** The full spec set is written (`docs/specs/`); implementation has not
> started. First build target is the `transform` foundation. This is **not** part of the Cajeta
> language stdlib — it is an external library family.

## Why

- **One language from the matmul kernel that runs a policy down to the FOC setpoint on the
  wire** — no Python↔C++↔firmware seam. The math-heavy layers (transforms, fusion, dynamics,
  estimation, control, inference) ride on the **`dev.cajeta.nucleo`** ML/scientific stack
  (autograd · nn/optim · column · sparse/linalg · torch/scipy façades) over the **`cajeta.xpu`**
  multi-target compute substrate (one kernel MIR → CPU / NVPTX / AMDGPU / SPIR-V — so a policy
  runs on whatever silicon the edge robot carries). See `docs/research/nucleo-integration-analysis.md`.
- **Family over a shared foundation.** One `transform` foundation defines SE(3)/SO(3) and the
  frame tree — and *one* quaternion + twist convention used everywhere (the coherence ROS's
  `tf`/`tf2` split never had). Focused libs depend on it, never upward.
- **DCE makes breadth free.** Cajeta's link-time dead-code elimination means a robot importing
  only `transform` + a fusion filter ships *only that code*. So the family unifies **types**
  while partitioning **dependencies and versioning** across libs — monolith ergonomics,
  focused-library footprint.
- **Safety as a differentiator.** Cajeta's ownership/borrow + deterministic drop give provable,
  GC-pause-free resource release on e-stop/watchdog/fault paths.

## The package family

Data flow: **sense → estimate → plan → control → actuate**, with infra + safety cross-cutting,
all over the `transform` foundation.

| Package | Responsibility | Notes |
|---|---|---|
| **`transform`** | SE(3)/SO(3) Lie math, quaternions, the time-stamped frame tree, core geometry/units/time types. **The foundation everything depends on.** | foundation · build first |
| `io` | Buses & transport: serial/UART, CAN/CAN-FD (socketcan), EtherCAT, I2C, SPI, GPIO/PWM, USB. | driver |
| `io.sensor` | Sensor drivers: IMUs, encoders, force/torque, LiDAR, sonar/ToF. | driver |
| `io.vision` | Cameras (V4L2/UVC/GenICam), depth (RealSense/OAK-D), point clouds. | driver · heavy-dep edge |
| `motor` | Actuator/drive drivers: DYNAMIXEL 2.0, ODrive/moteus/VESC, CiA 402, PWM/steppers. | driver |
| `fusion` | Attitude/state fusion: Madgwick, Mahony, EKF/UKF/error-state KF. | math |
| `estimation` | Factor-graph SLAM & localization, pose-graph optimization, odometry, ICP. | math |
| `kinematics` | FK/IK, Jacobians, URDF/SRDF parsing. | math |
| `dynamics` | Rigid-body dynamics (RNEA/ABA/CRBA) + analytical derivatives. | math |
| `planning` | Sampling planners (RRT*), jerk-limited trajectories (Ruckig-style), GJK/EPA collision. | math |
| `control` | PID, impedance/admittance, MPC, whole-body control, the shared QP solver. | math |
| `sim` | Simulation interface: MJCF/URDF model + sim/real sensor parity. | infra · heavy-dep edge |
| `log` | Telemetry: MCAP container + Foxglove WebSocket protocol. | infra |
| `safety` | E-stop, watchdogs, deterministic fault reaction, functional-safety hooks. | cross-cutting |
| `ai` | On-device learned-policy inference (LeRobot/VLA) via `nucleo.nn`/`torch` façade + `nucleo.autograd` over `cajeta.xpu`. | forward-looking edge |

```
                 ┌────────────────────────── ai (policy inference) ───────────────────────────┐
  io.sensor ─┐   │                                                                             │
  io.vision ─┼─▶ fusion ─▶ estimation ─▶ planning ─▶ control ─▶ motor ─▶ (drive closes FOC loop)
  io (buses) ┘        ▲         ▲            ▲          ▲           ▲
                      └ kinematics / dynamics ┘        │           │
        transform (SE(3)/SO(3) frame tree, core types) ── underlies ALL of the above
        log (MCAP/Foxglove) · sim (MJCF) · safety (e-stop/watchdog/fault) ── cross-cutting
```

## Layout

```
docs/specs/            specs, mirroring the dev.cajeta.robotica.* package tree
  robotica-spec.md       the family spec (taxonomy, pillars, packaging rationale)
  transform/             dev.cajeta.robotica.transform  (the foundation)
  io/ io/sensor/ io/vision/   dev.cajeta.robotica.io[.sensor|.vision]
  motor/ fusion/ … ai/   one spec dir per package
docs/research/         the landscape research the specs are grounded in (two reports)
agents/                plans + work stacks (spec → plan → develop workflow)
```

Every spec follows one format: **Definition** (with normative conventions inherited from
`transform`), **Features** with enumerated use cases, **Acceptance criteria**, **Deliverables**,
and research-grounded **References** tagged `[verified]` (primary-source confirmed) vs `[scope]`
(planning-grade, re-research before that package's plan).

## Where to start

1. Read the [family spec](docs/specs/robotica-spec.md).
2. Read the [`transform` foundation spec](docs/specs/transform/transform-spec.md) — the first
   thing to build, and the source of the family-wide conventions.
3. Background: the [landscape research](docs/research/) (Part 1 = actuation plane; Part 2 =
   perception → AI).

## License

[MIT](LICENSE) © 2026 Julian Klappenbach.
