# `dev.cajeta.robotica.io.vision` — Specification

The **camera & depth edge** of cajeta-robotica: image and point-cloud *input* from V4L2/UVC
and GenICam cameras, RealSense/OAK-D depth sensors, and the PCD / `PointCloud2` buffer
formats the rest of the stack consumes. This is a **heavy-dependency edge package** — native
deps (librealsense, DepthAI) are permitted here, unlike the board-agnostic spine. It depends
on `transform` (for camera extrinsics, the optical frame, and timestamps) and nothing else in
the family; it depends **never upward**. See the family spec
[`../../robotica-spec.md`](../../robotica-spec.md) §2.d and §3.c, and the research
[`../../../research/robotics-stack-part2-perception-to-ai.md`](../../../research/robotics-stack-part2-perception-to-ai.md)
§A.2–A.5.

---

## 1. Definition

`io.vision` is **transport + buffer-format** code: open a camera device, negotiate a pixel
format, pull framebuffers, decode/convert them, deproject depth into an organized point cloud,
and read/write the interop point-cloud containers (PCD, `PointCloud2`). It is *not* a
perception or learning layer — it produces the time-stamped image and point-cloud buffers that
`fusion`, `estimation`, and `ai` consume. Every camera/depth frame carries a `transform`
timestamp and is associated with a frame-tree frame (its optical frame), so a downstream
consumer can place every pixel and point in space and time.

> **[scope]** The entire hardware-driver surface this package reimplements is
> research §A.2–A.5, which is tagged **[scope note — not verified this pass]**: "mostly
> *transport + buffer-format* work, board-agnostic over Linux device nodes." The protocol and
> format numbers below are planning-grade and **must be re-verified against the V4L2 kernel
> API, the GenICam/GigE Vision standards, the librealsense/DepthAI APIs, and the PCL/ROS
> buffer-format specs before this package's plan is written.**

### 1.a Capability summary
- 1.a.i — Capture frames from a **V4L2 / USB UVC** device with a chosen pixel format and a
  zero-copy streaming buffer loop.
- 1.a.ii — Drive an **industrial GenICam camera** (GigE Vision / USB3 Vision) through its
  feature tree.
- 1.a.iii — Decode/convert **pixel formats** (FourCC families: YUYV/NV12/MJPG/RGB/Bayer) into a
  canonical image buffer.
- 1.a.iv — Carry a **pinhole camera model** (intrinsics + distortion) and project/deproject
  between pixels and 3-D rays/points, with extrinsics as a `transform` frame-tree edge.
- 1.a.v — Stream **aligned depth+color** from a RealSense / OAK-D sensor and deproject depth
  into an **organized point cloud**.
- 1.a.vi — Read and write the **PCD** and **`PointCloud2`** point-cloud buffer formats.
- 1.a.vii — **Time-stamp** every frame against `transform`'s clock and associate it with its
  optical frame.

### 1.b Conventions
- 1.b.i — **Inherits `transform` §1.b by reference (NORMATIVE).** Quaternion convention
  (Hamilton, scalar-last `[x,y,z,w]`), twist/wrench ordering (translation-first), active
  left-to-right `parent←child` transform composition, SI units (meters/radians/seconds), and
  right-handed frames are **fixed in `transform` and not redefined here.** Camera **extrinsics
  are SE(3) edges** in the `transform` frame tree (§2.d of transform); a frame's timestamp is a
  `transform` `Time`.
- 1.b.ii — **Camera optical frame (local convention, [scope]).** The per-camera *optical*
  frame is right-handed with **+Z forward along the optical axis, +X right, +Y down** (the
  computer-vision/REP-103 optical convention), distinct from a body frame (+X forward). The
  exact pinning — and the static optical↔body rotation supplied as a frame-tree edge — is a
  **[scope]** decision to fix in this package's plan; it must be consistent with `transform`'s
  right-handed, active-transform rules and never contradict them.
- 1.b.iii — **Canonical image buffer.** Decoded images are exposed in a single documented
  layout (row-major, explicit pixel format + stride + width/height + timestamp + frame id);
  raw device-native encodings are preserved only until conversion. Depth is exposed in **meters**
  (float) or as raw uint16 with an explicit depth scale.
- 1.b.iv — **Native deps are quarantined and feature-gated.** librealsense, DepthAI, and any
  GenTL producer are optional, behind import/feature gates, so a consumer that needs only PCD/
  `PointCloud2` codecs (or only V4L2) links none of the heavy/native stacks (DCE + gating).

### 1.c Non-goals
- 1.c.i — **No perception or learning.** Feature detection, stereo *matching algorithms*,
  object detection, visual odometry, and SLAM are `estimation`/`fusion`/`ai`, not here.
  `io.vision` ends at the time-stamped image/point-cloud buffer.
- 1.c.ii — **No LiDAR / sonar / ToF rangefinders.** RPLidar, Velodyne/Ouster, HC-SR04, and
  VL53L0X are `io.sensor` (family §2.c.iv). This package is cameras + depth-camera point clouds.
- 1.c.iii — **No general image processing.** Beyond format decode/convert and depth
  deprojection, filtering/resampling/registration of point clouds is out of scope.
- 1.c.iv — **No camera-intrinsic *calibration solver*.** This package *applies* a known
  intrinsic/extrinsic calibration; estimating it (checkerboard solve) is out of scope.

### 1.d Substrate — núcleo representations the buffers ARE
> `io.vision`'s output buffers are not bespoke types — they are núcleo substrate buffers, so a
> downstream consumer (`fusion`/`estimation`/`ai`) needs no marshalling. Mapping (cajeta repo
> `docs/specification/nucleo/`; gaps in `cajeta-robotica/docs/research/nucleo-integration-analysis.md`):
- 1.d.i — **Canonical image buffer (§1.b.iii) IS a `cajeta.math.Tensor`** (row-major, typed);
  decode/convert/debayer (§2.c) and filtering are `scipy.signal`/`ndimage` ops over it
  (`scipy-facade-spec.md` §4/§10.1), fused via `nucleo-expr`.
- 1.d.ii — **A point cloud IS a `nucleo.column` buffer / `Table<Point>`** — the *column ==
  tensor-buffer == Arrow-buffer* invariant (`nucleo-column-spec.md` §1.1/§7) means one decode
  is simultaneously dataframe-filterable, tensor-deprojectable, and zero-copy Arrow-exportable
  (to pyarrow/Open3D) with no inter-subsystem boundary.
- 1.d.iii — **PCD / `PointCloud2` codecs (§2.f) target núcleo-column buffers** (the family
  "codec-over-recorded-bytes" pattern); deprojection/project math is `cajeta.math` over the
  pinhole `Matrix<…>` (`syntax-sugar-spec.md` `@`). Native PCD/`PointCloud2` readers are an
  `io.vision`-owned gap (not in núcleo's codec set).

---

## 2. Features

### 2.a V4L2 / UVC camera capture
Definition: open a Linux video device, negotiate format, and run a streaming buffer loop using
the V4L2 ioctl API (UVC webcams enumerate as V4L2 devices). **[scope]** (research §A.5).
- 2.a.i (use case) — Enumerate devices and **query capabilities** (`VIDIOC_QUERYCAP`), supported
  formats/frame sizes/intervals (`VIDIOC_ENUM_FMT`/`ENUM_FRAMESIZES`/`ENUM_FRAMEINTERVALS`).
- 2.a.ii — **Set a pixel format and resolution** (`VIDIOC_S_FMT`) and validate the device's
  returned (possibly adjusted) format.
- 2.a.iii — **Stream with mmap buffers**: request buffers (`VIDIOC_REQBUFS`), `mmap`, queue/
  dequeue (`VIDIOC_QBUF`/`DQBUF`), `STREAMON`/`STREAMOFF` — a zero-copy capture loop yielding
  framebuffers + the kernel buffer timestamp.
- 2.a.iv — Get/set device **controls** (`VIDIOC_G/S_CTRL`: exposure, gain, white balance) where
  the device exposes them.
- 2.a.v — Surface device/format errors deterministically (unsupported format, EAGAIN/EBUSY,
  disconnect) without leaking the file descriptor or mapped buffers (ownership/drop).
- Acceptance: capture loop and format negotiation are exercised against **recorded V4L2
  buffer streams + a fake device-ioctl fixture** (no camera); buffer lifecycle leaks nothing.

### 2.b GenICam industrial cameras (GigE Vision / USB3 Vision)
Definition: drive a standards-compliant industrial camera through its GenICam **feature tree**
over a GenTL transport. **[scope]** (research §A.5: "GenICam / GigE Vision … a standardized
feature/transport model").
- 2.b.i — Discover cameras and load the camera's **GenApi feature description (XML)**; expose
  the feature node tree (integer/float/enum/command nodes).
- 2.b.ii — Get/set features by **SFNC** (Standard Features Naming Convention) names
  (`Width`, `Height`, `PixelFormat`, `ExposureTime`, `AcquisitionFrameRate`,
  `Gain`, `TriggerMode`).
- 2.b.iii — Start/stop acquisition and pull image buffers through a **GenTL** producer
  (`AcquisitionStart`/`Stop`, stream buffer grab).
- 2.b.iv — **[scope]** GigE Vision transport: control on **GVCP** (UDP control protocol, default
  port **3956**) and stream on **GVSP** (UDP stream protocol); USB3 Vision is the USB-bus analog.
  The decode of GVSP image-leader/payload/trailer packets is the codec-test surface.
- Acceptance: feature-tree load + SFNC get/set tested against a **recorded GenApi XML fixture**;
  GVSP packet reassembly tested against a **recorded GVSP packet capture** (no camera).

### 2.c Pixel formats & color conversion
Definition: decode the device-native FourCC encoding into the canonical image buffer (§1.b.iii).
**[scope]** (research §A.5).
- 2.c.i — Identify a buffer by its **FourCC** code and stride/plane layout.
- 2.c.ii — Convert **packed/planar YUV** (`YUYV`, `UYVY`, `NV12`, `I420`) → RGB/Gray.
- 2.c.iii — Decode **`MJPG`** (Motion-JPEG) frames to RGB.
- 2.c.iv — **Debayer** raw Bayer mosaics (`RGGB`/`GRBG`/`GBRG`/`BGGR`) to RGB.
- 2.c.v — Pass through already-canonical `RGB`/`BGR`/`GRAY`/`MONO16` without copy where possible.
- Acceptance: each conversion has a **round-trip / reference-image unit test** on synthetic
  buffers (deterministic, no hardware); conversions are pure functions over byte buffers.

### 2.d Camera model & calibration application
Definition: a pinhole camera model (intrinsics + distortion) plus the extrinsic that ties the
camera's optical frame into the `transform` frame tree. **[scope]** model; **[verified]**
SE(3)/frame dependency on `transform` (research §B.1).
- 2.d.i — Hold **intrinsics** `K = [fx, fy, cx, cy]` and a **distortion model** (plumb-bob /
  radtan `k1,k2,p1,p2,k3`; equidistant/fisheye `k1..k4`) for an image.
- 2.d.ii — **Project** a 3-D point in the optical frame → distorted pixel.
- 2.d.iii — **Deproject** a pixel + depth → 3-D point/ray in the optical frame (undistort).
- 2.d.iv — Register the camera's **extrinsic** (optical-frame pose) as a static SE(3) edge in
  the `transform` frame tree so points/poses can be looked up in any frame at the frame's time.
- 2.d.v — Carry the calibration as a serializable record (load from a calibration file; the
  *solving* of it is out of scope, §1.c.iv).
- Acceptance: **project∘deproject round-trips** to tolerance on synthetic intrinsics (no
  hardware); distortion/undistortion inverse-consistency tested.

### 2.e Depth cameras (RealSense / OAK-D)
Definition: stream aligned depth+color from a stereo/ToF depth sensor and deproject depth into
an organized point cloud, behind a feature-gated native dep (§1.b.iv). **[scope]** (research
§A.5: librealsense / DepthAI, "stereo/ToF, point clouds").
- 2.e.i — Open a depth sensor and configure streams (depth + color resolution/rate); read its
  **depth scale** and per-stream **intrinsics + the depth↔color extrinsic**.
- 2.e.ii — Stream **time-synchronized depth + color** frames (one acquisition → both buffers
  with a shared/associated timestamp).
- 2.e.iii — **Align depth to color** (reproject depth into the color frame using the
  depth↔color extrinsic + both intrinsics).
- 2.e.iv — **Deproject depth → organized point cloud** (one point per pixel, §2.d.iii), in the
  camera optical frame, with optional per-point RGB.
- 2.e.v — **[scope]** OAK-D/DepthAI runs stereo (and optionally on-device neural inference) on
  the device; this package consumes its **depth + image** outputs as buffers — any on-device
  inference result is opaque pass-through, not interpreted here.
- 2.e.vi — Native deps (librealsense2, DepthAI) are optional/feature-gated; absent the device
  SDK, the deprojection math (§2.e.iv) still runs on recorded depth+intrinsics fixtures.
- Acceptance: deprojection tested against a **recorded depth frame + intrinsics fixture**
  producing a known point cloud (no sensor); device SDK paths are integration-gated, not in the
  hardware-free suite.

### 2.f Point-cloud buffer formats (PCD / `PointCloud2`)
Definition: read/write the two interop point-cloud containers as pure byte-buffer codecs.
**[scope]** (research §A.5: "point clouds as **PCD / `PointCloud2`**").
- 2.f.i — **PCD reader/writer**: parse/emit the header (`VERSION FIELDS SIZE TYPE COUNT WIDTH
  HEIGHT VIEWPOINT POINTS DATA`) and the `ascii` / `binary` / `binary_compressed` data sections.
- 2.f.ii — **`PointCloud2` (ROS `sensor_msgs/PointCloud2`) codec**: the `PointField` array
  (name/offset/datatype/count), `point_step`/`row_step`, `is_bigendian`, `is_dense`, and the
  `height`/`width` organized-vs-unorganized distinction.
- 2.f.iii — Map between an in-memory typed point cloud and either container; preserve the
  organized (`height>1`) structure of a depth-derived cloud.
- 2.f.iv — Stamp a serialized cloud with its `transform` frame id + timestamp (§2.g) so it is
  self-locating.
- Acceptance: **encode→decode→encode round-trip is byte-stable** for ascii and binary PCD and
  for `PointCloud2`, against **recorded `.pcd` / serialized-`PointCloud2` fixtures** (no hardware).

### 2.g Frame time-stamping & frame-tree association
Definition: every produced buffer is located in time and space via `transform`. **[verified]**
dependency on `transform` (research §B.1–B.2).
- 2.g.i — Stamp each image/depth/cloud buffer with a `transform` `Time` derived from the device
  buffer timestamp (kernel/SDK timestamp → the family clock).
- 2.g.ii — Tag each buffer with its **optical frame id**, so a consumer can look up
  `T_target_camera` at the buffer's time via the frame tree.
- 2.g.iii — **[scope]** Optionally expose a clock-offset estimate (device clock ↔ host clock)
  as metadata; the *estimation* of that offset is planning-grade and may defer to a later plan.
- Acceptance: a buffer round-trips through stamp → frame-tree lookup → pose in a target frame
  on synthetic transforms (no hardware).

---

## 3. Acceptance criteria (beyond per-feature tests)
- 3.a — **Protocol/codec tests against recorded byte streams.** Per the family driver-layer
  rule (§3.b of the family spec), V4L2 buffer loops, GVSP packets, depth frames, and PCD/
  `PointCloud2` payloads are tested against **recorded fixtures**, not live hardware.
- 3.b — **Format round-trips.** PCD and `PointCloud2` encode↔decode are byte-stable; project↔
  deproject and distort↔undistort are inverse-consistent to tolerance.
- 3.c — **Native deps quarantined & optional.** librealsense / DepthAI / GenTL are feature-gated
  so a consumer of only the codecs or only V4L2 links none of them (DCE + gating verified by a
  minimal-dependency build).
- 3.d — **No upward dependencies.** `io.vision` imports only `transform` (and the permitted
  native SDKs); it imports nothing else in `dev.cajeta.robotica.*` and nothing depends upward
  on it except consumers above it in the spine.
- 3.e — **Convention conformance.** Optical-frame axis convention (§1.b.ii), canonical image/
  depth buffer layout (§1.b.iii), and the inherited `transform` conventions hold at every API
  boundary; depth is meters (or raw + explicit scale).
- 3.f — **Resource discipline.** File descriptors, mmap'd buffers, and SDK handles are released
  deterministically on drop / error paths (ownership model), even on device disconnect.

## 4. Deliverables
- 4.a — `dev.cajeta.robotica.io.vision` implementing §2, with the §3 test suite.
- 4.b — A **recorded-fixture corpus**: V4L2 buffer dumps, a GenApi XML + GVSP packet capture, a
  RealSense/OAK-D depth+intrinsics recording, and reference `.pcd` / `PointCloud2` files — the
  hardware-free codec test inputs.
- 4.c — A plan at `agents/vision-plan.md` (authored when this spec is approved), dependency-
  ordered: PCD/`PointCloud2` codecs → camera model (project/deproject) → V4L2/UVC capture →
  pixel-format conversion → GenICam → depth (RealSense/OAK-D) → frame-tree time-stamping. The
  codec/model layers (pure, hardware-free) land before the native-dep device layers.
- 4.d — **[scope]** Re-research note: before 4.c, re-verify the V4L2 ioctl/streaming API, the
  GenICam/GigE Vision (GVCP/GVSP) numbers, the PCD and `sensor_msgs/PointCloud2` field layouts,
  and the librealsense/DepthAI surfaces against their primary specs (research §A.5 is unverified).

## 5. References (research-grounded)
- Camera/depth transport+buffer surface — V4L2/UVC capture, GenICam/GigE Vision, librealsense
  (RealSense), DepthAI (OAK-D), point clouds as **PCD / `PointCloud2`** — research
  §A.2–A.5. **[scope: not verified this pass — re-research before plan]**
- SE(3)/SO(3) Lie foundation + the time-stamped frame tree this package stamps into —
  `transform` (family §2.a), research §B.1 (micro-Lie theory arXiv:1812.01537; Sophus). **[verified]**
- Frame-tree buffered-interpolated lookup (tf2 model) for `T_target_camera(t)` — research
  §B.2. **[scope: secondary source]**
- Optical-frame axis convention (CV/REP-103 optical: +Z forward, +X right, +Y down) — domain
  convention, to be pinned in the plan. **[scope]**
