# cajeta-robotica × núcleo — integration overlap & gap analysis

> Status: analysis for review (2026-06-24). **No spec edits made.** This document is the
> input to a later, approved spec-revision pass. Cross-repo: núcleo specs live in
> `cajeta-two/docs/specification/nucleo/`; robotica specs in this repo's `docs/specs/`.

## 0. Headline

cajeta-robotica was **designed for this substrate before it had a name.** The family spec
(§1.a.ii) and README already declare the math layers ride on *"Cajeta's existing
numpy/tensor/linalg/FFT/GPU substrate"* and pitch *"one language and one type system from
the matmul kernel that runs the policy down to the FOC setpoint on the wire."* **núcleo is
exactly that substrate, now layered** (column → expr → autograd → nn/optim → frame/sparse →
torch/keras/scipy/pandas façades, over `cajeta.math.Tensor` + the **XPU** multi-target kernel
layer). Integration is therefore low-risk: re-point robotica's flat substrate references at
núcleo's named layers, and grow núcleo where robotics has needs the ML specs didn't anticipate.

**XPU is what makes the thesis real.** XPU (`cajeta-two/src/cajeta/xpu/`) is *one kernel MIR,
four backends* — CPU (native), NVIDIA (NVPTX), AMD (AMDGPU), Vulkan (SPIR-V). núcleo's tensor
/autograd ops lower through it, so a policy forward pass runs on whatever silicon the edge robot
carries (Jetson, AMD board, or CPU-only). The "no Python↔C++↔firmware seam" claim depends on
this — and it holds.

## 1. Per-package overlap — what núcleo gives robotica for free

| robotica package | núcleo layer(s) it should ride | what it gets |
|---|---|---|
| **`ai`** (policy inference) | `torch` façade (§9 `.pt`/`state_dict`, weights-only), `nucleo-nn-optim` (`Module`/`Parameter`/`parameters()`), `nucleo-autograd`, `cajeta.math.Tensor`→XPU, `@Autocast` (§8) | policy architecture as `Module`; PyTorch/LeRobot weight load; type-safe, GPU-fused, no-global-state forward; mixed precision; (latent) on-device fine-tuning |
| **`io.vision`** | `nucleo-column` (the **column == tensor-buffer invariant**), `scipy.signal`/`ndimage` (§4/§10.1), `scipy.spatial` KDTree (§5.1), `records`, Arrow C Data Interface (§4) | decoded image **is** a `Tensor`; point cloud **is** `Table<Point>` (Arrow-filterable *and* tensor-bufferable on one set of bytes); debayer/format-convert/filter as fused kernels; correspondence search; zero-copy interop to pyarrow/Open3D |
| **`fusion`** (EKF/UKF, Madgwick/Mahony) | `cajeta.math` linalg, `scipy.optimize` (§3), `@Grad` | filter math on linalg; differentiable filters via `@Grad`; Cholesky for covariance |
| **`estimation`** (factor-graph SLAM, ICP) | `nucleo.sparse` + `sparse.linalg.cg` (§14), `scipy.optimize.leastSquares` (§3.4), `scipy.spatial` KDTree | sparse MAP solve; pose-from-correspondence least-squares; KDTree for ICP correspondence |
| **`kinematics`/`dynamics`** | `cajeta.math` + syntax sugar (`@`, `Matrix<T,R,C>` compile-time shapes), `nucleo-autograd` | shape-checked spatial-vector algebra; AD to *verify* analytical RNEA/ABA derivatives |
| **`control`** (PID/MPC/WBC/QP) | `scipy.optimize` (§3.6 linprog), `nucleo.optim` core, `nucleo.linalg` (Cholesky/LDLT) | QP/optimization primitives; dense+sparse factorizations |
| **`log`** (telemetry) | `nucleo.frame` (Polars-shaped `Table<T>`) | optional: columnar in-memory analysis of MCAP-logged runs |

**The single biggest free win:** `io.vision`'s point clouds and images. núcleo's
*column == tensor-buffer == Arrow-buffer* invariant means a `PointCloud2`/PCD payload decodes
once into a columnar buffer that is *simultaneously* a dataframe (filter/scan), a tensor
(matmul/deproject/broadcast), and a zero-copy Arrow export — **no subsystem boundary crossed**.
That is the same unification the splat flagship demonstrates, applied to perception.

## 2. Gaps — what núcleo must GROW for robotics

These are needs robotica has that the current núcleo spec set does **not** cover. Each is a
candidate addition to a núcleo spec (or a new one) at plan time.

1. **Real-time / determinism contract — the #1 gap.** *No* núcleo spec addresses latency,
   jitter, per-frame allocation, or a GC-/alloc-free hot loop. Robotics control is 100 Hz–1 kHz
   hard-real-time and `ai` inference is soft-real-time. The raw material exists (Cajeta's
   ownership + deterministic drop; the recently-merged thread-local **frame arena** + escape
   analysis; the compiled `@Grad`/`@Jit` fused-kernel path), but **no spec commits to a
   bounded-latency, allocation-disciplined compute contract.** Robotica's whole pitch (e-stop
   determinism, "GPU-to-actuator with no seam") needs this written down. → propose a
   "real-time / hot-loop" section, likely in `nucleo-expr` (lowering) + a robotica-side SLA.
2. **Batch-of-one / dynamic shapes.** `torch-facade` [T4] (fixed const-generic dims vs.
   dynamic dims) is unresolved. Streaming inference is batch-of-one with runtime-variable
   resolution. → confirm dynamic batch/rank is in v1; robotics can't preload a batch-flexible
   policy otherwise.
3. **Tensor force/eval triggers.** `nucleo-expr` [X5] (implicit-on-consume vs. explicit
   `.eval()`) is open. In a per-frame loop, *where* fusion forces determines per-tick allocation
   and latency. → robotics is a forcing-policy stakeholder.
4. **Distributions / sampling.** Stochastic policies (Gaussian/categorical action heads,
   diffusion-policy noise) need a `torch.distributions` equivalent. **Absent** from all núcleo
   specs. → add a distributions surface (façade or core).
5. **GPU/on-device spatial index — RESOLVED (2026-06-24).** `cajeta.xpu` provides an
   RT-as-compute spatial index (RTNN, device-verified, graduated from the prism/caramelo
   project); `scipy.spatial.KdTree`'s kNN/radius queries ride it (scipy `[S6]` resolved), and
   robotica `estimation`/`io.vision` consume the *same* xpu primitive for ICP correspondence /
   point-cloud nearest-neighbour. The GPU-spatial gap is filled.
6. **`Vmap` for batched inference** exists as a transform combinator but `torch` façade v1
   doesn't wrap it. Minor; document that `Vmap(forward)` is the batched-inference path.
7. **Image-preprocess utilities** (resize/crop/letterbox/normalize). Partly `scipy.ndimage`
   /`signal`; confirm resize + normalize are first-class so LeRobot/VLA image preprocessing runs
   GPU-side with no host round-trip.

**Robotica owns (not núcleo gaps), but should be expressed *over* núcleo buffers:**
PCD / `PointCloud2` codecs (target `nucleo-column` buffers, zero-copy Arrow), policy **config**
loading (weights are núcleo's; config is robotica's), episode/trajectory dataset semantics
(custom `Dataset` over `torch.utils.data`).

## 3. Where robotica specs are mis-pointed (re-point at plan time)

| location | current | re-point to |
|---|---|---|
| family spec §1.a.ii; README L14–16 | "Cajeta's numpy/tensor/linalg/FFT/GPU substrate" | the layered **núcleo** stack (`nucleo.column/expr/autograd/nn/optim/linalg/sparse` + scipy/torch façades) over `cajeta.math.Tensor` + **XPU** |
| `ai` §2.a–2.d | "load policy / forward pass on Cajeta's GPU/tensor stack" | `torch` façade `state_dict`/`.pt` load; `Module`/`Parameter` from `nucleo-nn-optim`; forward via `cajeta.math`→XPU; `@Autocast` precision |
| `ai` §1.c.i (non-goal: no training) | "training stays in Python" | keep v1 inference-only, **but add a note**: `nucleo-autograd` makes *on-device fine-tuning / continual learning* a cheap future capability (ties to the SPELA/cradle thread) — a strategic fork, not v1 scope |
| `io.vision` §1.b.iii, §2.e–2.f | "canonical image buffer", point-cloud formats | decoded image **is** `Tensor<…>`; point cloud **is** `Table<Point>`/`Column`; PCD/`PointCloud2` codecs target núcleo-column buffers; debayer/convert via `scipy.signal`/`ndimage` |
| `fusion`/`estimation`/`control`/`dynamics` (research §A–D) | "rides on Cajeta linalg / sparse-linalg / tensor stack" | name the specific núcleo layers: `scipy.optimize`, `nucleo.sparse`+`sparse.linalg`, `nucleo.linalg`, `@Grad` |

## 4. Mismatches / tensions to resolve before editing

- **Inference-only vs. autograd-enabled.** `ai` is explicitly inference-only. But the user's
  research thread (SPELA / cradle on-device continual learning) and `nucleo-autograd` make
  on-device adaptation cheap and native — arguably robotica's *differentiator* over a Python
  inference port. **Fork:** keep `ai` v1 frozen-inference, or scope an on-device-adaptation
  capability now?
- **Throughput-lazy vs. latency-real-time.** núcleo's lazy-expr/fusion is throughput-shaped;
  the control loop is latency-shaped. The compiled `@Grad`/`@Jit` path *should* yield a static,
  allocation-free kernel for the hot loop — but that must be confirmed and coupled to
  `nucleo-expr` [X6] (where lowering happens).
- **Edge isolation.** `ai`/`io.vision` are heavy/GPU edges; importing `transform`+`fusion` must
  not pull the inference stack. núcleo's per-layer packaging + Cajeta DCE supports this, but the
  re-point must keep façades and heavy layers behind the same import/feature gates robotica
  already specifies.

## 5. Recommendation

Integration is **high-value, low-risk** — robotica's architecture anticipated núcleo. Two
genuine núcleo spec *additions* are warranted by robotics and worth feeding back into the núcleo
spec set: (1) a **real-time / hot-loop compute contract**, (2) a **distributions/sampling**
surface — plus resolving the open TBDs robotics is a stakeholder in ([T4] dynamic shapes,
[X5]/[X6] forcing+lowering, [S6] GPU spatial index). The robotica-side edit pass is mostly
re-pointing substrate language and re-expressing `ai`/`io.vision` over named núcleo layers.
