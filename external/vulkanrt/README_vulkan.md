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
| `compile_shaders.bat`, `build.bat` | (new) Windows command-line build |
| `program_vk.sln`, `.vcxproj`, `.vcxproj.filters`, `.vcxproj.user` | (new) VS2022 project |
| `Makefile`, `compile_shaders.sh` | (new) Linux build |

Drop the whole folder where `program.cpp` lives, so `../../tiny_bvh.h`,
`../../testdata/cryteksponza.bin` and `raysets/view3rays.bin` still resolve.
`shaders/` stays a subfolder of the project directory; the loader searches
`shaders/`, `../shaders/`, `../../shaders/`.

## Build

Needs the LunarG Vulkan SDK, whose installer sets `VULKAN_SDK`.

**Visual Studio 2022:** open `program_vk.sln`, pick `Release|x64`, build, run.
x64 only, `v143`, C++20, `/arch:AVX2`. The shaders are `CustomBuild` items that
compile in place to `shaders/*.spv` and rebuild when you edit them (or when you
edit `common.glsl`, via `AdditionalInputs`), so shader errors land in the Error
List with file and line. `Clean` deletes the `.spv` files too.

Two project details that matter rather than being boilerplate:

- The working directory must stay `$(ProjectDir)`, which is both the default and
  what `.vcxproj.user` sets explicitly. All three runtime paths depend on it.
  Running `bin\Release\program_vk.exe` from Explorer will not find the scene;
  launch from the IDE, or `cd` to the project folder first.
- A `CheckVulkanSDK` target fails the build with a readable message if
  `VULKAN_SDK` is unset or `glslangValidator.exe` is missing, instead of letting
  it surface as a confusing include error. Visual Studio only sees environment
  variables that existed when it started, so restart the IDE if you installed
  the SDK while it was open.

Debug builds are fine but slow to start: `BuildHQ` plus `Optimize` on Sponza is
minutes of unoptimized tinybvh. Benchmark in Release.

**Windows command line:** from a Developer Command Prompt:

```
build.bat
```

which calls `compile_shaders.bat` first. Shaders are SPIR-V loaded at runtime
rather than baked into `.fxh` headers, so a shader-only edit needs just
`compile_shaders.bat` — no relink.

**Linux:**

```
make          # shaders + binary
make run      # ... and run it from this directory, which the paths need
```

Dependencies, beyond a driver with `VK_KHR_ray_tracing_pipeline`:

```
sudo apt install libvulkan-dev libxcb1-dev glslang-tools     # Debian/Ubuntu
sudo dnf install vulkan-loader-devel libxcb-devel glslang    # Fedora
sudo pacman -S vulkan-headers libxcb glslang                 # Arch
```

The LunarG SDK works instead of the distro packages. `-mavx2 -mfma` is applied
only on x86_64, so the same Makefile builds on aarch64 and lets tinybvh take its
NEON path. Editing `shaders/common.glsl` rebuilds all seven shaders;
`compile_shaders.sh` does the shaders alone.

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

## Platforms

One source file, two windowing backends behind `#if defined(_WIN32)`:

|  | Windows | Linux |
| --- | --- | --- |
| surface | `VK_KHR_win32_surface` | `VK_KHR_xcb_surface` |
| window | Win32 + `WndProc` | XCB |
| link | `vulkan-1.lib`, `user32` | `-lvulkan -lxcb -pthread` |

XCB rather than Xlib on purpose: `X.h` turns ordinary words into macros
(`None`, `Below`, `Complex`, `Success`), and `tiny_bvh_base.h` already contains
one of them, so including `Xlib.h` ahead of tinybvh would need a pile of
`#undef`s. Everything XCB exposes is `xcb_`-prefixed. Wayland-only sessions go
through XWayland, which is normally present.

The event handlers now only set `windowClosed` / `windowResized` and the shared
main loop acts on them. That is not just for portability: it takes `Resize()`
out of the Win32 `WndProc`, where it previously ran re-entrantly from inside
`DispatchMessage`, so the swapchain is now only ever rebuilt between frames.

Everything Windows-specific is reached through six one-line shims
(`AlignedAlloc64`, `AlignedFree64`, `SleepMs`, `GetCwd`, `ShowFatal`,
`QuitProcess`) plus the three window functions, so the ~950 lines of Vulkan in
between are genuinely shared rather than forked.

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
- **Ray set is the viewpoint.** There is no camera in either version: the whole
  viewpoint is whatever `raysets/view3rays.bin` recorded, read verbatim. A
  missing file is therefore fatal, with a message naming the working directory,
  since that is nearly always the cause. `ALLOW_SYNTHETIC_RAYS` turns on an
  invented pinhole camera instead, but its viewpoint is unrelated to the
  recorded one, so the image will not match the D3D12 output and the timings are
  not comparable. Leave it off unless you just want the program to start.
  The loader now also rejects a truncated or wrong-resolution ray set, which
  would otherwise leave zeroed rays whose infinite reciprocals traverse as
  garbage.
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
