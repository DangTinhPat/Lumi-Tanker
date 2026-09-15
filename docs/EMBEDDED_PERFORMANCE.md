# Embedded Core Performance

No target timing, stack, RAM-budget or hardware-readiness result is claimed here.
Host builds and allocation checks are separate evidence from ROS integration,
closed-loop simulation and physical hardware. No installation of a hardware
toolchain, flashing or motor operation is part of this work.

## One Portable Core

`src/tap_control/core/CMakeLists.txt` builds the existing seven implementation
files as `tap_control_core`, without ROS, ament, Gazebo or Python dependencies.
It requires C++17; the inspected core uses standard arrays and C++17 facilities
such as `std::clamp`, with no identified C++20 requirement. The ROS package can
use `add_subdirectory(core)` and link this same target. It owns its adapters,
test registration and install/export rules. There is no duplicate runtime core.

The standalone entry point registers tests only when configured as the top-level
source directory. Example host build, from the workspace root:

```sh
cmake -S src/tap_control/core -B /tmp/tap-core-host -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DTAP_MPC_MAX_HORIZON=30 -DTAP_BUILD_BENCHMARK=ON
cmake --build /tmp/tap-core-host -j2
ctest --test-dir /tmp/tap-core-host --output-on-failure
/tmp/tap-core-host/core_allocation_tests
```

`BUILD_TESTING=OFF` omits standalone tests. `TAP_BUILD_BENCHMARK=ON` builds the
existing `apps/control_benchmark.cpp` with its test-only reference MPC source;
its host duration summaries cover only the operations it names, not a proven
worst-case controller latency or an ESP32 deadline.
The benchmark is not a timing acceptance test.

## Capacity And Allocation Contract

`TAP_MPC_MAX_HORIZON` is a CMake cache integer in `[4,80]`, default `80`.
Capacity `30` is the intended future standalone configuration shown above;
the default `80` preserves ROS capacity unless explicitly overridden.
The target exports it as a PUBLIC compile definition so library and consumers
agree on header layout. The parent-owned header supplies the same default for
non-CMake consumers and validates its bounds. Runtime horizon requests are
clamped to the compiled capacity; changing capacity requires rebuilding every
consumer. A smaller capacity can change controller behavior, not just memory.
Do not mix differently configured libraries and headers across an ABI boundary.

MPC owns fixed-capacity impulse-response, condensed gradient-map and Hessian
arrays, avoiding the original full prediction tensor. Storage still scales
quadratically with capacity; reducing only the runtime horizon does not shrink
the object. EmbeddedController contains an MPC object. Avoid placing these large
objects on a small RTOS task stack; choose static or explicitly reserved storage
after examining the actual target linker map and memory budget.

`core_allocation_tests` replaces scalar, array, nothrow and aligned C++ allocation
operators in the test executable. Counting gates exclude logging and caller-owned
object storage but include valid construction, reset, update and MPC CoM rebuilds.
It exercises nonzero solver work, embedded control transitions, a synthetic jump
phase sequence, and station anchor acquisition/release. It prints `sizeof` for
the principal objects on the compiler/ABI actually used. These are object sizes,
not task stack high-water marks or complete firmware RAM use.

Zero counted allocations means no calls to those C++ operators in the exercised
paths. It does not intercept direct `malloc`/`calloc`/`realloc`, exception runtime
storage, driver allocations, ROS middleware, or every possible input path.
Invalid JumpController and StationKeeper configurations currently throw standard
exceptions. Exception-disabled firmware needs an explicit configuration-validation
design; a successful host build does not establish `-fno-exceptions` support.

## ESP32-S3 Limits

The future ESP32-S3 target has hardware single-precision (float32) floating point;
double-precision arithmetic is software, not equivalent FPU throughput. MPC and
many core signals use `float`, while StationKeeper uses `double`, jump height
bookkeeping uses `double`, and geometry-related expressions need an expression-
level type audit. Float storage alone does not prove all intermediates or math
library calls use float32. Transcendental functions are not automatically cheap
because an FPU exists. Do not mechanically narrow precision without numerical
equivalence and boundary-condition tests.

The MPC is a four-state inverted-pendulum acceleration optimizer, not a full
contact-constrained rigid-body controller. Its fixed iteration count bounds work
but does not prove convergence or stability. CoM scheduling can rebuild matrices
inside an update; measure that path separately from unchanged-model operation.
RTOS scheduling, interrupt contention, flash/cache behavior, memory placement,
sensor acquisition and actuator transport all contribute to end-to-end latency.
PC timings must never be rescaled or reported as measured ESP32 timings.

## Required Target Evidence

- Record board revision, clock, toolchain, flags, firmware revision, horizon,
  solver iterations, task affinity/priority and internal RAM versus PSRAM placement.
- Measure target cycles and wall time for construction, reset, ordinary update,
  worst tested solver states, pose-driven rebuilds, jump transitions and faults.
  Include maxima and tail distributions under realistic interrupt and I/O load.
- Inspect linker-map static RAM and code size; measure each task's stack high-water
  mark, interrupt stack, temporary solver storage and remaining heap under load.
- Measure sensor timestamps/freshness, bus transfer and motor-command completion,
  loop jitter, overruns and watchdog response against a stated control deadline.
- Reconcile MPC model sampling period with actual observation/update timing;
  run numerical equivalence, bounds, stale-input, reset and stop-priority tests.
- Preserve LH, LK, LW, RH, RK, RW ordering and native +Y axes on both sides.
  Any hardware adapter must validate frame signs, units and timestamp handling.

All target measurements and physical capability claims remain pending. Synthetic
jump phase traversal is allocation coverage, not evidence that the robot jumps
or lands safely.

## Algorithm Changes

- Store the impulse response `A^k B` instead of the triangular prediction
  tensor. Generate matrix powers in a small local workspace, not a persistent
  horizon-sized table. The default capacity and active control horizon are
  unchanged (80 capacity, 20 steps in the simulation profile).
- For horizons up to 30, accumulate shared Hessian stage terms along diagonals
  in O(N^2), and precompute the six-input linear gradient map when the model
  changes. The per-tick gradient becomes O(N), not O(N^2).
- For horizons above 30, retain the original float summation order for the
  Hessian and gradient, using compact storage. The condensed gradient failed
  an unchanged terminal-pitch tolerance at N80 and a -10 m coordinate offset;
  this sensitivity is not hidden by widening test thresholds. Long-horizon
  rebuilds still have cubic arithmetic, but avoid zero tensor entries.
- Keep the original sequential projected solver, 120 configured iterations,
  constraints, model-update threshold, filters and safety behavior. Solver
  work remains O(iterations * N^2). No fast-math, lower control frequency,
  coarser geometry or reduced solver budget is used to claim a speedup.

## Host Measurements

Same-executable frozen-reference comparisons, GCC 15.3.0 x86-64, warm caches.
These are illustrative samples with OS scheduling noise, not deadlines.

| Object | Before, capacity 80 | After, capacity 80 | After, capacity 30 |
| --- | ---: | ---: | ---: |
| MPC | 136336 B | 29376 B | 5176 B |
| EmbeddedController | Not separately measured | 29784 B | 5584 B |

The unchanged-capacity MPC reduction is 78.5%. The 96.2% reduction to capacity
30 additionally constrains maximum supported horizon; it is not attributed
solely to the arithmetic optimization. Object storage excludes code, stack,
other tasks, drivers, logging and the future estimator.

Final ROS-build benchmark at N20, capacity 80, 120 iterations, mean microseconds:

| Path | Reference | Optimized |
| --- | ---: | ---: |
| Steady update | 41.707 | 40.492 |
| Model rebuild only | 9.866 | 4.610 |
| Rebuild plus update | 51.997 | 44.425 |

Steady-update improvement is modest, about 3%, not a multi-fold whole-loop
speedup. Rebuild medians were 9.829/4.308 us (about 2.3x); p99 was
10.150/4.319 us. Observed maxima (12.965/65.264 us) show why these host samples
must not be called worst-case execution bounds. At N80, original-order compact
rebuild mean was 118.477 us versus 349.772 us for the frozen reference.

A separate Release capacity-30 sample at N20 measured rebuild means of
10.158/3.281 us, and steady updates 40.258/39.681 us. Different build flags and
capacity make these a separate comparison, not interchangeable timing claims.

Host GCC `-fstack-usage`, capacity 30: MPC buildModel 208 B, solveHorizon 280 B,
update 80 B; EmbeddedController update 368 B. These are individual compiler
frame reports, not cumulative task stack or ESP measurements. The frozen
reference solveHorizon frame was 904 B in that build.

## Verification

- Final ROS-package CTest: 11/11 suites passed. Standalone capacity-30 CTest:
  8/8 passed, including the independently frozen MPC reference comparison.
- Default-capacity equivalence: 192912 scalar comparisons across N4/20/30/31/80,
  resets, warm starts, saturation, contact/integration gates, moving pitch
  references, invalid inputs and +/-10 m position offsets. Maximum observed
  acceleration difference 0.000180721 m/s^2; velocity difference
  0.0000531673 m/s. These are sampled tests, not an all-input proof.
- Allocation gates counted zero C++ allocations across the exercised valid
  construction, reset, model rebuild, control, jump and station-keeper paths.
- ASan/UBSan equivalence executable passed the earlier 156741 comparisons;
  the added N31 boundary passed the normal executable, not a sanitizer rerun. The root
  Python suite passed 92 tests. ROS contract counts from the prior quality audit
  are historical, not newly rerun here.
- Installed controller SHA-256:
  `01b2fca1effbf0cf3d6255849297a586447fbdc380c2948521ae917523a7c98c`.
  Optimized MPC source SHA-256:
  `fc64d20951bc44fc7ad931fe6d545010c982ad84e721b8da8b46aada0b09703b`.

Raw host benchmark outputs are retained in `log/core_performance_01b2fca1/`.
The capacity-30 sample preceded the long-horizon summation fallback; N20/30
formulas are unchanged, but it is retained as a separate earlier sample.

## Closed-Loop Regression

Using the same bounded motors, gains, 20-step horizon, 120 solver iterations
and strict 10 mm peak / 2 mm final displacement gates from
[the quality audit](QUALITY_AUDIT.md), the final controller passed:

| Log directory | Test | Duration | Peak XY | Final XY |
| --- | --- | ---: | ---: | ---: |
| `transformation_1789208857707291089` | Two 25% squat/left-roll/right-roll cycles | 63.996 s | 9.474 mm | 0.276 mm |
| `transformation_1789208978758213053` | Two symmetric deep squats, -1.6 rad target, 11 s transitions | 56.004 s | 2.544 mm | 0.023 mm |

The pose run's maximum absolute pitch was 0.015486 rad and peak joint tracking
error 0.025628 rad. Measured applied-effort e-stop passed; cleanup left no live
owned processes or zombies. No geometry, mass or torque limit was changed.
The deep run reached body-origin Z 0.126788 m, maximum absolute pitch
0.027395 rad and peak joint-target error 0.049893 rad. Its applied-effort e-stop
and owned-process cleanup also passed. These two final-build runs preserve the
previous acceptance envelope; they do not claim that numerical optimization
improves every motion metric or eliminates physical drift.
These are flat-ground Gazebo results, not new rough-terrain or jump acceptance.
The earlier 6.39 cm jump with failed landing remains unvalidated; this
performance change does not supply additional physical motor capability.
