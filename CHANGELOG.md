# Changelog

Overview of changes to TinyBVH on the `main` branch.

---

## 1.9.2 — 2026-11-14

_New in 1.9.2.: A C API, the AMD RDNA4 BVH8 layout, an experimental DOBB layout, a
new parallel `Optimize`, and an online header customizer._

### API change
- `Save` returns `bool` on all layouts that have this method: `false` if the file
  could not be opened or written.

### Added
- **C API.** `tiny_bvh_c.h` (plain C99, C11 preferred) with its implementation in
  `tiny_bvh_c.cpp`, which contains TinyBVH and builds as a static or shared
  library. Each BVH is an opaque handle for one layout; functions return
  `tbvh_result`, and fatal errors come back as error codes instead of ending the
  process (except on build worker threads). Rays, hits and instances are
  binary-compatible with the C++ types.
- **`CWBVH8_AMD_HW`**: AMD RDNA4 hardware BVH8 layout with converter, OpenCL
  traversal and example (`tiny_bvh_amd_hwrt_bvh8.cpp`). Primitive nodes pack up to
  eight triangle pairs with deduplicated vertices. By Alessandro Salo ([#308],
  [#313]).
- **`BVH_DOBB`** (experimental): 8-wide compressed nodes whose child boxes use one
  of 104 discrete rotations (Kern et al., 2025), tuned with `DOBB_ALPHA`; viewer
  in `tiny_bvh_dobb_test.cpp`. By mmmuscus ([#315]).
- `settings.bonsaiTopTree` selects how the Bonsai top tree is built:
  `BONSAI_TOP_BINNED` (default), `BONSAI_TOP_FULLSWEEP`, `BONSAI_TOP_HPLOC`, or
  `BONSAI_TOP_CHEAPEST` (all three, keeping the lowest SAH).
- `settings.keepSplitting`: subdivide regardless of what SAH prefers.
- `BVH::Depth()`.
- Direct `Save`/`Load` for `BVH4_GPU` and `BVH4_AMD_HW` (previously only via the
  plain BVH), and for the new `CWBVH8_AMD_HW`. The static `Load( fileName, ctx )`
  now takes a `BVHContext`.
- Slang port of the CWBVH traversal kernel, `kernels/traverse_cwbvh.slang`, for
  Vulkan (SPIR-V), Metal, D3D12 and the other Slang targets. By Daniel Johansson
  ([#311]).
- TinyOCL: device-only buffers (`DEVICEONLY`, which can be created without host
  data) and per-kernel user defines (`Kernel( file, entryPoint, defines )`).

### Changed
- `Optimize` is reimplemented as parallel reinsertion (Meister & Bittner, 2018) on
  the thread pool, and compacts the tree itself.
- `BVH4_CPU` and `BVH8_CPU` build with the default SAH costs (`C_INT` = 1,
  previously 2), so their trees differ from 1.9.1.
- Faster Bonsai construction; `Compact` and leaf fitting are multi-threaded; AVX2
  code for CWBVH conversion; relaxed atomics in the builders improve ARM
  performance.
- `CWBVH_COMPRESSED_TRIS` is off by default (it was slower), and the precomputed
  CWBVH triangle data is more precise (suggested by Niels Brunekreef).
- Failed allocations, and a `BVHContext` without a `malloc` hook, are now fatal
  errors instead of returning null.

### Fixed
- `BuildHQ` no longer leaves `settings.useSpatialSplits` set after the call.
- `Load` on the wide and GPU layouts no longer leaks what the object held before,
  and keeps the caller's opacity micro maps.
- Opacity micro maps set on a layout are no longer cleared by the internal
  conversion in `Build` and `Refit`; `SetOpacityMicroMaps( maps, 0 )` clears them.
- Hang in the wide-BVH collapse (seen with `BVH4_GPU`).
- Allocation bug in `BVH_Verbose::Compact`.
- `MergeLeafs` produced wrong results on H-PLOC trees, which broke collapsing them
  to wide layouts.
- NEON builder: out-of-bounds pointer arithmetic in the sliced binning (undefined
  behaviour; trees are unchanged). By Daniel Johansson ([#314]).

### Tools, examples and repository
- New `tools/` folder:
  - `bvhopt.cpp` (moved from the root and much extended): builds optimized BVHs
    from a mesh for the binary, wide CPU, GPU and AMD hardware layouts, with
    `fast`, `hq` and `extreme` profiles, `sah`, `epo` and `raycost` metrics, and
    ray-weighted collapsing to the wide layouts.
  - `tinybvh_custom.cpp`: generates a single header with only the layouts,
    features and ISAs you need, also available
    [online](https://jbikker.github.io/tinybvh/) (built and deployed by a GitHub
    Actions workflow).
- Vulkan benchmark: integrated GPUs, BVH build and conversion timing, AMD hardware
  layouts through OpenCL.
- OpenCL headers updated.
- `CITATION.cff` was updated.

---

## 1.9.1 — 2026-10-12

_1.9.1 in a nutshell: Bonsai and Bonsai HQ become the default builders. And: an 
H-PLOC builder, the AMD hardware BVH4 layout, much faster SBVH builds, and an offline 
optimizer._

### API 
- **New default builders.** `Build` now uses Bonsai (Ganestam et al., 2015) on the
  SIMD builder, and with `useSpatialSplits` (or `BuildHQ`) Bonsai HQ (Ganestam &
  Doggett, 2016), so trees differ from 1.9.0. Set `settings.binnedBVH = true` to
  get the binned SAH and SBVH builders back.
- Indexed builds take `const Index*`: still `uint32_t*` for the single-precision
  layouts, but `uint64_t*` for `BVH_Double` and `BVH4_Double`.

### Added
- Bonsai and Bonsai HQ builders.
- H-PLOC builder (`settings.useHPLOC`), producing trees identical to the NexusBVH
  CUDA code, for experiments / comparisons.
- **`BVH4_AMD_HW`**: AMD hardware BVH4 format (BOX32 and triangle nodes) for RDNA3
  and up, traversed from OpenCL with AMD's BVH intersection intrinsics; examples
  `tiny_bvh_amd_hwrt.cpp` and `tiny_bvh_minimal_amd_hwrt.cpp`. By Alessandro Salo
  ([#302], [#305]).
- `BVH::Save( fileName, includeGeometry = true )` embeds the triangles, so a
  loaded BVH owns its geometry (`FreeGeometry` releases it). Every layout has a
  static `Load( fileName )` that returns a finished object, converting where needed.
- Build settings `enableThreading` (`false` keeps a build on the calling thread,
  for building many BLASes at once, one per thread), `discardIntermediates` and
  `minimizeMemUse`, plus SBVH tuning through `minSpatialOverlap` and
  `spatialSkipSmallestAxis`.
- TinyOCL: Linux support and AMD architecture detection (`ISGCN`, `ISCDNA`,
  `ISRDNA1` to `ISRDNA4`).

### Changed
- SBVH construction is much faster: incremental clipping, SSE fragment splitting,
  256-bit bin boxes on AVX2 (mirrored for NEON), a special case for nodes of up to
  eight primitives, skipping the smallest axis for spatial splits, and AVX2 node
  emission.
- When building for `BVH4_CPU`, nodes of four or fewer primitives are no longer
  split.
- Lower memory use during builds; faster handling of nodes with one or two
  primitives; the built-in job pool spins before it parks.

### Fixed
- Crash when compiling with `-ffast-math`.
- Rebuilding a BVH with a different triangle count.
- `BVH_GPU::ConvertFrom` ignored vertex indices.
- `MBVH::Refit`, `LeafCount` and `SAHCost` are iterative, avoiding stack overflows
  on deep trees ([#300]); `SplitLeafs` is now iterative too.
- Full-sweep builder: sort keys overflowed at the top of their range.
  Presplitting: a split position outside the triangle is now rejected ([#307]).
- Clang: missing includes ([#306]).

### Tools, examples and repository
- `bvhopt.cpp`: offline optimizer that spends as long as it takes to lower SAH and
  EPO cost, using only the public API.
- `build.sh` for building the benchmark on Linux.
- README fully revised.

---

## 1.9.0 — 2026-09-25

_Changes for 1.9.0: The library becomes six headers with templated layouts. 
Also a double-precision `BVH4`, NEON and much faster SSE traversal for `BVH4_CPU`, 
and robustness fixes, many of them by Wenzel Jakob. Includes development 
versions 1.8.1 and 1.8.2._

### API
- **The library is now six files** ([#290], proposed in [#289]): `tiny_bvh.h`
  (configuration and platform detection, and the only header application code
  includes), `tiny_bvh_base.h` (types, builders, traversal), and the SIMD kernels
  in `tiny_bvh_x86_float.h`, `tiny_bvh_x86_double.h`, `tiny_bvh_arm_float.h` and
  `tiny_bvh_arm_double.h`. Keep them together.
- Layouts are class templates in `tinybvh::impl`, on scalar and index type. The
  familiar names are aliases (`BVH = impl::BVH<float, uint32_t>`,
  `BVH_Double = impl::BVH<double, uint64_t>`, …), so forward declarations of the
  old classes no longer compile.
- Layouts are move-only: copy constructors are deleted. `ReferenceFrom`,
  `ReleaseOwnership` and `DropReference` share buffers explicitly.
- `BVHBase::BVHType` is now `tinybvh::BVHType`, and `LAYOUT_BVH_DOUBLE` is gone
  (`BVH_Double` reports `LAYOUT_BVH`).
- `BuildAVX` and `BuildNEON` are replaced by `BuildSIMD`.
- The `Ray` constructor no longer normalizes its direction ([#296]), so `hit.t` is
  measured in units of the direction's length. Call `Ray::Normalize()` where
  needed, or define `NORMALIZE_RAY_DIR` for the old behaviour.
- Custom-geometry callbacks take an extra `void* userdata` argument, which comes
  from `customUserdata` ([#277]).
- Custom allocators: define `TINYBVH_ALIGNED_ALLOC` and `TINYBVH_ALIGNED_FREE`
  (both or neither) instead of `_ALIGNED_ALLOC` and `_ALIGNED_FREE`.
- `BVH_GPU` data changed: leaves are always compact, and vertices are reordered
  per leaf (`orderedVerts`), removing the triangle index indirection.
- `RayEx::mask` is 32 bits; `BLASInstanceEx::transform` is a `bvhdblmat4`.

### Added
- `BVH4_Double`: double-precision `BVH4_CPU` with AVX2 and NEON kernels. By Wenzel
  Jakob ([#293]).
- `BVH_Double` gains everything `BVH` has: SBVH, presplitting, full-sweep and
  quick builders, `Optimize`, `Refit`, octant-specialized traversal, `Save` and
  `Load`; primary rays are about 30% faster on Sponza ([#290]). Strided
  double-precision vertex data via `bvhdbl3slice` ([#277]).
- NEON traversal for `BVH4_CPU` ([#287]); on an Apple M4 Max (one core) it traces
  13.0 / 48.8 / 4.1 MRays/s primary / shadow / diffuse, against 5.5 / 20.2 / 1.8
  for the binary BVH. Scalar reference traversal for every wide layout, used where
  no SIMD kernel exists.
- Bundle traversal (WiVeC, AVX2): `IntersectBundle` and `IsOccludedBundle` on
  `BVH` and `BVH4_CPU` trace `TINYBVH_BUNDLE_RAYS` rays per call (default 64).
  Based on work by Markus Gnauck (nodebleed).
- `BVH_GPU::BuildAABB`, used by the new GPU hair example.
- `tinybvh_set_fatal_error_handler`. Fatal errors go through
  `tinybvh_fatal_error`; a handler may throw, and if it returns, the process
  exits.
- `tinybvh_shutdown_builtin_pool()` releases the built-in thread pool at a
  controlled point.
- `TINYBVH_STACK_SIZE` (default 128) and `BVH::CollectNodes`.
- A BVH can be built from a static constructor (the SSE constants are no longer
  static members).
- A translation unit may include `tiny_bvh.h` more than once; the implementation
  is emitted once.

### Changed
- `BVH4_CPU` SSE traversal uses the "branchy" strategy of the NEON kernel
  ([#288]). On a Ryzen 9 7950X: 7.6 → 11.5 MRays/s primary, 8.8 → 19.8 shadow, 3.9
  → 4.5 diffuse. `BVH4_CPU` is now the fastest CPU layout.
- `BVH8_CPU` sorts children by their true entry distance, adds a fast path for two
  hit children, and prefetches (`BVH8_USE_PREFETCHING`) ([#294]). On a Ryzen 9
  7950X: 11.3 → 12.9 MRays/s primary, 5.3 → 6.4 diffuse.
- A TLAS transforms the ray in place when it enters a `BVH4_CPU` or `BVH8_CPU`
  BLAS ([#297]).
- New MBVH collapse, based on Ylitie et al. (2017).
- CWBVH traversal and conversion optimizations; faster presplitting through a fast
  cube root approximation, by BoyBaykiller ([#279]); `[[likely]]` and
  `[[unlikely]]` hints in traversal.

### Fixed
- Small triangles were missed because the triangle tests used an absolute
  determinant epsilon; the tests are also NaN-safe now ([#283]).
- The SIMD triangle tests use an exact division instead of an approximate
  reciprocal ([#285]).
- Axis-aligned rays: reciprocal directions are clamped to a power of two ([#286]),
  and the safe reciprocal clamps to `BVH_FAR`, so slab tests no longer produce
  NaNs or miss hits on box faces. Double-precision rays compute their reciprocals
  the same way as single-precision ones ([#277]).
- The slab tests enforce a specific FMA contraction; with `-ffp-contract=fast`,
  rays could slip through nodes with a collapsed axis (seen with AppleClang on
  arm64) ([#277]).
- TLAS traversal forwards the full ray state (instance index, mask, hit record) to
  the BLAS, also for occlusion rays and double precision ([#277]).
- SBVH rejected spatial splits in large nodes because of an integer overflow
  ([#290]).
- Empty child slots in `BVH4_CPU` and `BVH8_CPU` nodes get empty bounds instead of
  a tiny box near 1e30 ([#274]).
- The NEON SIMD builder is fixed, and is now actually used by `Build` on ARM.
- `Optimize` selected subtrees for reinsertion with a wrongly clamped metric.
- `NO_INDEXED_GEOMETRY` had no effect in 1.8.0.
- Strict-aliasing violations removed throughout.
- Aligned allocation rewritten: `_aligned_malloc` on Windows (MSVC, MinGW,
  clang-cl), `posix_memalign` everywhere else. This fixes Emscripten ([#247]),
  older glibc and older macOS.

### Removed
- `BVH_SoA` (deprecated since 1.8.0) and the 256-ray packet traversal
  (`Intersect256Rays`, `Intersect256RaysSSE`).
- The `tiny_bvh_fenster.cpp` example.

### Tools, examples and repository
- `tests/` folder: SIMD-against-scalar reference test, axis-aligned ray test, TLAS
  state test and double-precision test.
- New examples: `tiny_bvh_instancing.cpp` (formerly `tiny_bvh_anim.cpp`) and its
  double-precision variant, `tiny_bvh_raybundle.cpp`, `tiny_bvh_gpu_hair.cpp`.
- DirectX 12 (`external/dx12rt`) and Vulkan (`external/vulkanrt`, also on Linux)
  benchmarks that compare TinyBVH GPU traversal with hardware ray tracing.
- TinyOCL 0.3.1 to 0.3.4: stability, device query and teardown fixes, leak fixes,
  interop detection.

---

## 1.8.0 — 2026-07-08

_1.8.0 in a nutshell: A build settings struct, Bring Your Own Thread Pool, a 
benchmark against Embree and Madmann91's BVH library, and a reorganized repository. 
Includes development versions 1.7.2 and 1.7.3._

### API
- Build options live in `BVHBase::settings` (`BVHBuildSettings`):
  `usePresplitting`, `presplitFactor`, `presplitPostPass`, `useSpatialSplits`,
  `useFullSweep`, `postOptimize`, `optimizeIterations` and `useSIMDifavailable`.
  `Build` picks the builder from these; `BuildHQ` and `BuildAVX` remain available.
- `BVH_SoA` is deprecated and only compiled with `USE_DEPRECATED_LAYOUT`.
- `MT_USE_JOBSYSTEM` and `MT_NO_JOBSYSTEM` are gone: threading goes through the
  new `BVHContext` hooks, with `MT_SPAWN_DEPTH` limiting task depth.

### Added
- **Bring Your Own Thread Pool**: `BVHContext::spawn`, `barrier` and
  `parallel_for` hooks, used by all builders. The built-in pool is part of
  `tiny_bvh.h` and can be left out with `TINYBVH_NO_BUILTIN_POOL`. By Wenzel Jakob
  ([#264]).
- Indexed geometry for `BVH_Double`. By Wenzel Jakob ([#263]).
- Presplitting for the full-sweep builder, and an optional pass that un-splits
  primitives in leaves after a presplit build (`presplitPostPass`).
- `NO_VOXEL_SUPPORT`; with `NO_THREADED_BUILDS`, TinyBVH now compiles as C++14.

### Changed
- `BuildAVX`: threaded fragment preparation, plus further speedups.
- Full-sweep builder: the radix sort runs in parallel over x, y and z.
- Threaded `EPOCost` (plus a faster SSE path) and threaded SAH calculation on x64.
- `BVH_GPU` node order is optimized for cache coherence during traversal.

### Fixed
- `BVH_Double::IsOccluded` called the intersection callback instead of the
  occlusion callback for custom geometry ([#263]).
- Undefined references with `CWBVH_COMPRESSED_TRIS` ([#265]) and
  `BVH4_GPU_COMPRESSED_TRIS` ([#266]). By Cody Bennett.
- Presplitting edge case close to the parent's extent. By BoyBaykiller ([#259]).
- Fatal-error messages are formatted with `snprintf`.

### Tools, examples and repository
- Repository reorganized into `examples/`, `kernels/` and `benchmark/`.
- `tiny_bvh_benchmark.cpp` times and validates all builders and traversal kernels,
  compares them with Embree and Madmann91's BVH library, and includes GPU
  measurements and CSV export.
- `tiny_bvh_viewer.cpp`: a bare-bones CPU scene viewer.

---

## 1.7.1 — 2026-06-09

_Triangle pre-splitting and faster SBVH clipping._

### Added
- Triangle pre-splitting before the build (`usePresplitting`, `presplitFactor`),
  after Section 5 of Karras & Aila (2013) and BoyBaykiller's implementation.

### Changed
- SBVH fragment clipping and splitting rewritten after Madmann91's approach, with
  an SSE path.
- TinyOCL 0.3.0: targets OpenCL 3.1.

### Fixed
- Multiple-definition link errors when `tiny_bvh.h` is included in more than one
  source file (the double-precision helpers are now `inline`).

---

## 1.7.0 — 2026-06-06

_A faster threaded `BuildAVX`, `constexpr` vector types, and cleaner MSVC builds._

### Added
- `constexpr` constructors for the vector types ([#253]).
- Aligned allocation can be overridden by defining `_ALIGNED_ALLOC` and
  `_ALIGNED_FREE`, and Android below API 28 uses `memalign`. By Mauro Deryckere
  ([#252]); merged in 1.6.8 but enabled in this release. (Renamed in 1.9.0.)
- Threaded builds can use `std::thread` instead of the job system
  (`MT_NO_JOBSYSTEM`). (Replaced in 1.8.0.)

### Changed
- `BuildAVX`: optimized threading and AVX code.
- NaN checks use a bit-level test instead of `std::isnan`, so they survive
  `-ffast-math`; the safe reciprocal is more robust.

### Fixed
- MSVC: `_CRT_SECURE_NO_WARNINGS` is no longer required, and fatal-error message
  boxes no longer assume a multi-byte character set.
- Reference binned builder: no division by zero for zero-area nodes or flat axes.

---

## 1.6.8 — 2026-06-01

_Collects the work on `dev` from September 2025 to June 2026: job-system threading
for the builders, `BuildAABB`, move constructors, and a batch of community fixes.
`BuildAABB` ([#244]) and the `WATERTIGHT_TRITEST` fix ([#246]) reached `main`
early, on 2026-02-13, still labeled 1.6.7._

### API
- The implementation needs C++17 again (1.6.7 built as C++14), for the new job
  system. (From 1.8.0, `NO_THREADED_BUILDS` allows C++14.)

### Added
- `BVH::BuildAABB( aabbs, count )` builds a BVH over axis-aligned boxes, two
  `bvhvec4` per box. By thatalloguy ([#244]).
- Move constructors for all layouts, so they can be stored in containers. By David
  Peicho ([#235]).

### Changed
- Threaded builds (`Build`, `BuildHQ`, `BuildAVX`) run on a built-in job system,
  condensed from Wicked Engine's, instead of spawning a `std::thread` per subtree;
  parallel binning in `BuildAVX`, previously disabled, is enabled.
- Full-sweep builder: better multithreading ([#236]) and 19% less memory ([#243]).
  By BoyBaykiller.

### Fixed
- `IntersectSphere` looped forever when it reached a leaf. By David Peicho
  ([#234]).
- `MBVH::Refit` did not update interior node bounds. By David Peicho ([#242]).
- `WATERTIGHT_TRITEST` did not compile in `TriOccludes`. By Marco Miralles
  ([#246]).
- `BVH8_CWBVH::ConvertFrom` counted internal children twice.

### Tools, examples and repository
- OpenCL headers and ICD loader refreshed; Embree (used for comparisons) updated
  to 4.4.1; `tiny_ocl.h` reports build errors even when the error log is empty.

---

## 1.6.7 — 2025-09-05

_A threaded full-sweep builder, a fix for `BuildAVX`, and C++14 instead of C++17.
This was the release on `main` until June 2026._

### Changed
- The library builds as C++14: it no longer uses C++17 inline variables.
- The full-sweep SAH builder is multi-threaded.

### Fixed
- Bad `BuildAVX` builds: threaded builds take nodes from one atomic counter
  instead of reserved per-subtree ranges, so the node list has no gaps, and
  `usedNodes` is exact (it reported the whole allocation before).

### Tools, examples and repository
- The Fenster-based examples no longer need C++17.

[#234]: https://github.com/jbikker/tinybvh/pull/234
[#235]: https://github.com/jbikker/tinybvh/pull/235
[#236]: https://github.com/jbikker/tinybvh/pull/236
[#242]: https://github.com/jbikker/tinybvh/pull/242
[#243]: https://github.com/jbikker/tinybvh/pull/243
[#244]: https://github.com/jbikker/tinybvh/pull/244
[#246]: https://github.com/jbikker/tinybvh/pull/246
[#247]: https://github.com/jbikker/tinybvh/issues/247
[#252]: https://github.com/jbikker/tinybvh/pull/252
[#253]: https://github.com/jbikker/tinybvh/issues/253
[#259]: https://github.com/jbikker/tinybvh/pull/259
[#263]: https://github.com/jbikker/tinybvh/pull/263
[#264]: https://github.com/jbikker/tinybvh/pull/264
[#265]: https://github.com/jbikker/tinybvh/pull/265
[#266]: https://github.com/jbikker/tinybvh/pull/266
[#274]: https://github.com/jbikker/tinybvh/issues/274
[#277]: https://github.com/jbikker/tinybvh/pull/277
[#279]: https://github.com/jbikker/tinybvh/pull/279
[#283]: https://github.com/jbikker/tinybvh/pull/283
[#285]: https://github.com/jbikker/tinybvh/pull/285
[#286]: https://github.com/jbikker/tinybvh/pull/286
[#287]: https://github.com/jbikker/tinybvh/pull/287
[#288]: https://github.com/jbikker/tinybvh/pull/288
[#289]: https://github.com/jbikker/tinybvh/issues/289
[#290]: https://github.com/jbikker/tinybvh/pull/290
[#293]: https://github.com/jbikker/tinybvh/pull/293
[#294]: https://github.com/jbikker/tinybvh/pull/294
[#296]: https://github.com/jbikker/tinybvh/pull/296
[#297]: https://github.com/jbikker/tinybvh/pull/297
[#300]: https://github.com/jbikker/tinybvh/issues/300
[#302]: https://github.com/jbikker/tinybvh/pull/302
[#305]: https://github.com/jbikker/tinybvh/pull/305
[#306]: https://github.com/jbikker/tinybvh/issues/306
[#307]: https://github.com/jbikker/tinybvh/issues/307
[#308]: https://github.com/jbikker/tinybvh/pull/308
[#311]: https://github.com/jbikker/tinybvh/pull/311
[#313]: https://github.com/jbikker/tinybvh/pull/313
[#314]: https://github.com/jbikker/tinybvh/pull/314
[#315]: https://github.com/jbikker/tinybvh/pull/315
