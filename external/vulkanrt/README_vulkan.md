# Vulkan port of the tinybvh GPU ray tracing benchmark

Same experiment as `program.cpp`: a fixed ray set is traced `NUM_DISPATCHES` times
per frame by one backend, the timestamped interval becomes MRays/s, and the
backends are cycled one per frame so they see the same clocks.

## Files

| file | was |
| --- | --- |
| `program_vk.cpp` | `program.cpp` |
| `shaders/common.glsl` | (new) shared bindings + `safe_rcp` |
| `shaders/trace.rgen`, `trace.rmiss`, `trace.rchit` | `shader.hlsl` |
| `shaders/tiny.comp` | `tiny.hlsl` |
| `shaders/tiny4.comp` | `tiny4.hlsl` |
| `shaders/tiny8.comp` | `tiny8.hlsl` |
| `shaders/tinyrq.comp` | `tinyrq.hlsl` |
| `compile_shaders.bat`, `build.bat` | (new) |

Drop `program_vk.cpp` where `program.cpp` lives, so `../../tiny_bvh.h`,
`../../testdata/cryteksponza.bin` and `raysets/view3rays.bin` still resolve.
Put `shaders/` next to the executable (or one or two levels up — the loader
searches `shaders/`, `../shaders/`, `../../shaders/`).

## Build

Needs the LunarG Vulkan SDK. From a Developer Command Prompt:

```
compile_shaders.bat
build.bat
```

`build.bat` calls `compile_shaders.bat` for you. Shaders are SPIR-V loaded at
runtime rather than baked into `.fxh` headers, so a shader edit only needs
`compile_shaders.bat`, no relink.

## Backends

| backend | mechanism | requirement |
| --- | --- | --- |
| `HWRT` | `VK_KHR_ray_tracing_pipeline` + SBT | hard |
| `BVH_GPU` | `tiny.comp` | — |
| `BVH4_GPU` | `tiny4.comp` | — |
| `CWBVH` | `tiny8.comp` | — |
| `RayQuery` | `tinyrq.comp`, `VK_KHR_ray_query` | optional |

`VK_KHR_acceleration_structure` + `VK_KHR_ray_tracing_pipeline` are required, as
raytracing tier 1.0 was in the D3D12 version. `VK_KHR_ray_query` stands in for
tier 1.1: without it the program cycles one backend fewer instead of failing.
The `ENABLE_*` defines at the top control *printing* only — every backend still
runs, so the per-slot timing stays aligned.

## What changed, and why

- **Descriptors.** D3D12 root descriptors have no Vulkan equivalent, so all nine
  resources live in one descriptor set written once at init. The binding table is
  documented in `shaders/common.glsl`. Nothing is rebound per backend, which
  makes the Vulkan per-frame path slightly cheaper outside the timed region.
- **Presentation.** `CopyResource` became `vkCmdBlitImage`, which also copes with
  a BGRA swapchain and a client area that is not exactly 1024x1024. The render
  target stays a fixed 1024x1024 storage image, so a resize only rebuilds the
  swapchain.
- **Semaphore wait stage.** The acquire semaphore is waited at `TRANSFER`, not
  `TOP_OF_PIPE`, so traversal starts before the swapchain image is ready and the
  timestamped interval never absorbs a presentation stall.
- **Timing.** `vkCmdWriteTimestamp` at `BOTTOM_OF_PIPE`, read back with
  `vkGetQueryPoolResults` after the frame fence — no readback buffer needed.
  `timestampPeriod` is nanoseconds per tick; results are masked to
  `timestampValidBits`.
- **Compaction.** `VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR` plus
  `vkCmdCopyAccelerationStructureKHR` in `COMPACT` mode, same two-step as
  `EmitRaytracingAccelerationStructurePostbuildInfo` /
  `CopyRaytracingAccelerationStructure`.
- **Dispatch overlap.** Consecutive dispatches are left unordered, as in the
  D3D12 version, so the two ports measure the same thing. Flip
  `BARRIER_BETWEEN_DISPATCHES` to serialize them.
- **Pre-splitting** has been dropped, as requested. `dxrVerts`/`dxrTriCount` are
  gone; the BLAS is built from `verts` directly.
- **Ray set fallback.** If `raysets/view3rays.bin` is missing the program prints a
  warning and generates a synthetic pinhole camera ray set rather than crashing
  in `fread`. Those numbers are not comparable to a recorded run.
- **`CWBVH_COMPRESSED_TRIS`** is now a hard `#error` rather than a silent
  mismatch: `tiny8.comp` hard-codes a 4x`vec4` triangle stride (as `tiny8.hlsl`
  did), so if the define ever gets switched off in `tiny_bvh.h` the shader would
  read past the end of the triangle buffer.

## Shader translation notes

Mechanical, and each traversal kernel keeps its original structure line for line:

| HLSL | GLSL |
| --- | --- |
| `asuint` / `asfloat` | `floatBitsToUint` / `uintBitsToFloat` |
| `firstbithigh` / `countbits` | `findMSB` / `bitCount` (cast: they return `int`) |
| `select(c, a, b)` | `mix(b, a, c)` with a `bvec` selector |
| `groupshared` | `shared` |
| `SV_DispatchThreadID` / `SV_GroupIndex` | `gl_GlobalInvocationID` / `gl_LocalInvocationIndex` |
| `uav.GetDimensions(w,h)` | `imageSize(uav)` |
| `RWTexture2D<float4>` | `layout(rgba8) uniform image2D` |
| `StructuredBuffer<T>` | `readonly buffer { T x[]; }`, `std430` |
| `RayQuery<>::TraceRayInline` | `rayQueryInitializeEXT` + `rayQueryProceedEXT` loop |

`std430` reproduces the D3D12 `StructuredBuffer` strides exactly (`RayData` 32 B,
`BVHNode` 64 B, `vec4` 16 B) — verified against the disassembled SPIR-V, so the
tinybvh buffers can be uploaded byte-for-byte as before.

Two small behavioural notes carried over from the HLSL as-is: the `HWRT` path
uses `TMin 0.001 / TMax 1000` while the compute paths use `0 / 1e30`, and the
ray query path uses `TMin 0`. Worth aligning if you want the backends to be
bit-identical rather than just comparable.

If you would rather not maintain a GLSL fork, `dxc -spirv -fvk-use-dx-layout`
can compile the original HLSL to SPIR-V; you would still need to replace the
`register(tN, spaceM)` bindings with `[[vk::binding(n)]]` to match the single
descriptor set above.
