# Dev
This is the *development branch* for TinyBVH. New features are tested here first. Please direct your PRs to this branch.

# TinyBVH
Single-header BVH construction and traversal library written as "Sane C++" (or "C with classes"). Some C++17 is used, e.g. for threading. The library has no dependencies. 
TinyBVH is _fast_. Here is, in a nutshell, how it compares to [Intel's Embree](https://www.embree.org) and [Madmann91's BVH library](https://github.com/madmann91/bvh).
![Performance](images/perfgraphs.png)
TinyBVH currently builds a SAH BVH faster than Embree and Maddmann91. It traces individual primary rays faster than both alternatives. Embree outperforms TinyBVH for _any hit_ rays. Note: These results are based on the new ````tiny_bvh_benchmark.cpp```` application and are cautiously presented as 'preliminary'. If you suspect an inperfection in the experiment setup, please [let me know](mailto:bikker.j@protonmail.com).
![Performance](images/cpu_vs_gpu.png)
When tracing rays on the GPU, (multicore) CPU performance is dwarfed. TinyBVH traces up to 4 billion rays per second in Crytek's Sponza scene using the straight-forward binary BVH format, regardless of graphics API and without using specialized ray tracing hardware. That is 4 rays per pixel at 4k@120Hz, enough for most purposes.
# TinyOCL
TinyBVH GPU ray traversal examples are available for OpenCL and OpenGL with compute shaders. For OpenCL support, you can use TinyOCL: a single-header OpenCL library, which helps you select and initialize a device. It also loads, compiles and runs kernels, with several convenient features:
* Include-file expansion for AMD devices
* Multi-argument passing
* Host/device buffer management
* Vendor and architecture detection and propagation to #defines in OpenCL code
* ..And many other things.

![Bistro](images/combined.jpg)

To use tinyocl, just include ````tiny_ocl.h````; this will automatically cause linking with ````OpenCL.lib```` in the 'external' folder, which in turn passes on work to vendor-specific driver code. But all that is not your problem!

Note that the ````tiny_bvh.h```` library will work without ````tiny_ocl.h```` and remains dependency-free. The new ````tiny_ocl.h```` is only needed in projects that wish to trace rays _on the GPU_ using BVHs created by ````tiny_bvh.h````.
  
# BVH?
A Bounding Volume Hierarchy is a data structure used to quickly find intersections in a virtual scene; most commonly between a ray and a group of triangles. You can read more about this in a series of articles on the subject: https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics .

To build a BVH using TinyBVH, simply call ````BVH::Build```` on an instantiated ````BVH````. See [````tiny_bvh_minimal.cpp````](https://github.com/jbikker/tinybvh/blob/dev/tiny_bvh_minimal.cpp) for a (really) small example.
Internally, the call will get forwarded to a specialized builder. There are fast builders, builders for 'high quality' BVHs and experimental builders for research and development. 

A selection:
* ````BVH::Build```` : Efficient plain-C/C+ binned SAH BVH builder which should run on any platform.
* ````BVH::BuildAVX```` : A highly optimized version of BVH::Build for Intel and AMD CPUs. 
* ````BVH::BuildHQ```` : A 'spatial splits' BVH builder, for optimal BVH quality.

A constructed BVH can be used to intersect a ray with the geometry, using ````BVH::Intersect```` or ````BVH::IsOccluded````, for shadow (aka 'any hit') rays.

Apart from the default BVH layout (simply named ````BVH````), several other layouts are available, which all serve one or more specific purposes. You can create a BVH in the desired layout by instantiating the appropriate class. The available layouts are:
* ````BVH```` : A compact format that stores the AABB for a node, along with child pointers and leaf information in a cross-platform-friendly way. The 32-byte size allows for cache-line alignment.
* ````BVH_Double```` : Double-precision version of ````BVH````.
* ````MBVH<M>```` : In this (templated) format, each node stores M child pointers, reducing the depth of the tree. This improves performance for divergent rays. Based on the [2008 paper](https://graphics.stanford.edu/~boulos/papers/multi_rt08.pdf) by Ingo Wald et al.
* ````BVH4_CPU```` : SSE-optimzied wide BVH traversal (["WiVe"](https://web.cs.ucdavis.edu/~hamann/FuetterlingLojewskiPfreundtHamannEbertHPG2017PaperFinal06222017.pdf)). The fastest option for CPUs that do not support AVX.
* ````BVH8_CPU```` : AVX2-optimized wide BVH traversal (["WiVe"](https://web.cs.ucdavis.edu/~hamann/FuetterlingLojewskiPfreundtHamannEbertHPG2017PaperFinal06222017.pdf)). This is the fastest option on CPU.
* ````BVH_GPU```` : This format uses 64 bytes per node and stores the AABBs of the two child nodes. This is the format presented in the [2009 Aila & Laine paper](https://research.nvidia.com/sites/default/files/pubs/2009-08_Understanding-the-Efficiency/aila2009hpg_paper.pdf). It can be traversed with a simple GPU kernel.
* ````BVH4_GPU```` : A compact version of the ````BVH4```` format, which may be faster for GPU ray tracing.
* ````BVH8_CWBVH```` : An advanced 80-byte representation of the 8-wide BVH, for state-of-the-art GPU rendering, based on the [2017 paper](https://research.nvidia.com/publication/2017-07_efficient-incoherent-ray-traversal-gpus-through-compressed-wide-bvhs) by Ylitie et al. and [code by AlanWBFT](https://github.com/AlanIWBFT/CWBVH).

A BVH in any format can be _rebuilt_ at any time by calling the ````Build```` method on the changed triangle data. A BVH may also be _refitted_, in case the triangles moved, using ````BVH::Refit````. Refitting is substantially faster than rebuilding and works well if the animation is subtle. Refitting does not work if polygon counts change.

Most layouts may be serialized and de-serialized via ````::Save```` and ````::Load````.

A more complete overview of TinyBVH functionality can be found in the [Basic Use Manual](https://jacco.ompf2.com/2025/01/24/tinybvh-manual-basic-use) and the [Advanced Topics Manual](https://jacco.ompf2.com/2025/01/25/tinybvh-manual-advanced).

# How To Use
The library ````tiny_bvh.h```` is designed to be easy to use. Please have a look at [````tiny_bvh_minimal.cpp````](https://github.com/jbikker/tinybvh/blob/dev/tiny_bvh_minimal.cpp) for an example. A Visual Studio 'solution' (.sln/.vcxproj) is included, as well as a CMake file. That being said: Most examples consists of only a single source file, which can be compiled with clang or g++, e.g.:

````g++ tiny_bvh_minimal.cpp````

The cross-platform fenster-based single-source **bitmap renderer** can be compiled with

````g++ -mwindows -O3 tiny_bvh_fenster.cpp -o tiny_bvh_fenster```` (on Linux and Windows)

````c++ --std=c++17 -framework Cocoa -O3 tiny_bvh_fenster.cpp -o tiny_bvh_fenster```` (on macOS)
09:05 07/07/2026
The multi-threaded **path tracing** demo can be compiled with

````g++ -mwindows -O3 tiny_bvh_pt.cpp -o tiny_bvh_pt```` (on Linux and Windows)

````c++ --std=c++17 -framework Cocoa -O3 tiny_bvh_pt.cpp -o tiny_bvh_pt```` (on macOS)

The **performance measurement tool** can be compiled with:

````g++ -mavx2 -mfma -Ofast tiny_bvh_speedtest.cpp -o tiny_bvh_speedtest```` (on Linux and Windows)

````c++ --std=c++17 -framework OpenCL -Ofast tiny_bvh_speedtest.cpp -o tiny_bvh_speedtest```` (on macOS)

Many additional demos are provided, demonstrating features of the library in small source files.

# Version 1.7.3

Basic use:

````
BVH bvh;
bvh.Build( (bvhvec4*)myTriData, triangleCount ); // or: BuildHQ( .. )
bvh.Intersect( ray );
````

To build a BVH for indexed vertices, use the new indexed interface:

````
BVH bvh;
bvh.Build( (bvhvec4*)vertices, (uint32_t*)indices, triangleCount );
````

If you wish to use a specific builder (such as the spatial splits builder) or if you need to do custom operations on the BVH, such as post-build optimizing, you can still do the conversions manually. Example:

````
BVH bvh;
bvh.Build( verts, indices, triCount );
BVH_Verbose tmp;
tmp.ConvertFrom( bvh );
tmp.Optimize( 100 );
bvh.ConvertFrom( tmp );
printf( "Optimized BVH SAH cost: %f\n", bvh.SAHCost() );
````

Note that in this case, data ownership and lifetime must be managed carefully. Specifically, layouts converted from other layouts use data from the original, so both must be kept alive.

**New in 1.7.3:** BVH build settings are now in ````BVHBase::settings````:
````
struct BVHBuildSettings
{
    bool usePresplitting = false;	// pre-split triangles before building the BVH.
    bool useSpatialSplits = false;	// consider spatial splits during construction (SBVH).
    bool presplitPostPass = true;	// attempt to un-split primitives in leafs after a presplit build.
    float presplitFactor = 0.3f;	// presplit budget relative to input data size.
    bool useFullSweep = false;		// for experiments only; full-sweep SAH builder.
    bool postOptimize = false;		// optimize generated BVH using tree rotations.
    int optimizeIterations = 25;	// default optimization iterations.
    bool useSIMDifavailable = true;	// set to false to use the scalar reference builder.
};
````   
This supersedes the calls to ````::BuildHQ```` and ````::BuildAVX````, which are now invoked from ````::Build```` based on the hints in ````BVHBuildSettings````. The old interface also remains available for now.

This version of the library includes the following functionality:
* Reference 'full-sweep' SAH BVH builder
* Fast cross-platform binned SAH BVH builder
* Fast binned SAH BVH builder using AVX intrinsics
* Fast binned SAH BVH builder using NEON intrinsices, by [wuyakuma](https://github.com/wuyakuma)
* Spatial Splits ([SBVH](https://www.nvidia.in/docs/IO/77714/sbvh.pdf), Stich et al., 2009) builder, including "unsplitting"
* Wide BVHs (any width) using collapsing
* TLAS builder with instancing and fast TLAS/BLAS traversal, even for 'mixed trees'
* TLAS masking (similar to [OptiX](https://raytracing-docs.nvidia.com/optix7/guide/optix_guide.230712.A4.pdf)), by [Romain Augier](https://github.com/romainaugier).
* Support for custom geometry and mixed scenes
* Double-precision binned SAH BVH builder
* Customizable SAH parameters for all builders
* "End-Point Overlap" BVH cost metric (["On Quality Metrics of Bounding Volume Hierarchies"](https://users.aalto.fi/~ailat1/publications/aila2013hpg_paper.pdf), Aila et al., 2013)
* GPU-friendly layouts, including 'Compressed Wide BVH' (CWBVH) for state-of-the-art GPU performance
* Support for custom primitives via callbacks, also in double-precision BVHs
* BVH (de)serialization for most layouts

Besides basic examples demonstrating usage of the library, these more advanced examples are provided:
* Example OpenCL code for GPU TLAS/BLAS traversal (dragon invasion demo, tiny_bvh_gpu2.cpp)
* Example OpenGL / compute shader code for GPU BLAS traversal (tiny_bvh_gl_compute)
* Example TLAS/BLAS application using OpenGL interop (windows only)

Advanced / exotic features of the library include:
* BVH optimizer: reduces SAH cost and improves ray tracing performance ([Bittner et al., 2013](https://dspace.cvut.cz/bitstream/handle/10467/15603/2013-Fast-Insertion-Based-Optimization-of-Bounding-Volume-Hierarchies.pdf))
* BVH pre-splitting (implementing ideas from [a paper](https://research.nvidia.com/sites/default/files/pubs/2013-07_Fast-Parallel-Construction/karras2013hpg_paper.pdf) by Karras and Aila and [explanation](https://github.com/BoyBaykiller/IDKEngine) by BoybayKiller)
* Full-Sweep SAH BVH (with support from [BoyBaykiller](https://github.com/BoyBaykiller))
* Opacity Micro Map support (as proposed [by Gruen et al.](https://dl.acm.org/doi/10.1145/3406180) in 2020)
* Sphere/BVH collision detection via BVH::IntersectSphere(..)
* Fast AVX2 ray tracing: Implements the 2017 paper by [Fuetterling et al.](https://web.cs.ucdavis.edu/~hamann/FuetterlingLojewskiPfreundtHamannEbertHPG2017PaperFinal06222017.pdf)
* Fast SSE4.2 ray tracing: A modified version of the AVX2 implementation using just SSE4.2 achieves 80% of AVX2 performance.
* Fast triangle intersection: Implements the 2016 paper by [Baldwin & Weber](https://jcgt.org/published/0005/03/03/paper.pdf)
* 'Watertight' ray/triangle intersection, based on the [paper](https://jcgt.org/published/0002/01/05/paper.pdf) by Woop et al.
* OpenCL traversal example code: Aila & Laine, 4-way quantized, CWBVH
* OpenCL support for MacOS, by [wuyakuma](https://github.com/wuyakuma)
* Support for WASM / EMSCRIPTEN, g++, clang, Visual Studio
* Optional user-defined memory allocation, by [Thierry Cantenot](https://github.com/tcantenot)
* Vertex array with a custom stride, by [David Peicho](https://github.com/DavidPeicho)
* Vertex array with indexing
* 'Bring Your Own Vector Types' (BYOVT), thanks [Tijmen Verhoef](https://github.com/nemjit001)
* 'Bring Your Own Thread Pool', thanks [Wenzel Jakob](https://github.com/wjakob)
* 'Benchmark' tool that times and validates all builders and traversal kernels
* A [manual](https://jacco.ompf2.com/2025/01/24/tinybvh-manual-basic-use) is now available.

The current version of the library is stable. Changes may happen but should be limited.

# Platforms
TinyBVH is a cross-platfrom library and should build on any platform that supports C++20 (the '20' bit is for threading). That being said, several platforms are specifically supported:
* ````x86/x64 Windows/Linux````: These platforms benefit from highly optimized SSE/AVX/AVX2 traversal kernels.
* ````OPENCL````: Although TinyBVH does not directly use OpenCL, GPU-specific BVHs can be constructed and example code for BLAS/TLAS traversal in OpenCL is provided.
* ````OpenGL/compute````: Example compute shader code for BLAS traversal is also provided.
* ````ARM_NEON````: Efficient BVH construction code is provided for ARM NEON. The ````BVH_SoA```` layout uses NEON intrinsics for fast traversal.
* ````ANDROID````: This platform benefits from ARM NEON support as well as aligned memory allocation.
* ````APPLE````: This platform benefits from ARM NEON support, dedicated paths in GPU example code and specialized support in TinyBVH itself.
  
# TinyBVH in the Wild
A list of projects using TinyBVH:
* Remedy's [Northlight](https://www.remedygames.com/northlight) engine uses TinyBVH in their editor and for baking per-micro-vertex displacements.
* [EA SEED's Gigi](https://github.com/electronicarts/gigi/releases/tag/v1.0.0): Uses TinyBVH for WebGPU ray tracing, "..as fast as when using the DXR api in DX12". Try the [demo](https://electronicarts.github.io/gigi/Demos/tinybvh/index.html).
* SideFX [Houdini](https://www.sidefx.com/docs/houdini/licenses) uses TinyBVH.
* [wave_tracer](https://github.com/ssteinberg/wave_tracer) by Shlomi Steinberg uses TinyBVH to trace path segments.
* [TrenchBroomBFG](https://github.com/RobertBeckebans/TrenchBroomBFG), by Robert Beckebans. "TinyBVH allows to load bigger glTF 2 maps almost instantly instead of minutes".
* Jon Baker's [Icarus](https://jbaker.graphics/writings/icarusPLY.html) Point Cloud visualizer uses TinyBVH to represent geometry.
* The [Wonderland Engine](https://wonderlandengine.com/) for web-based 3D uses TinyBVH for WebGL ray tracing.
* [unity-tinybvh](https://github.com/andr3wmac/unity-tinybvh): An example implementation for TinyBVH in Unity and a foundation for building compute based raytracing solutions, by Andrew MacIntyre.

# TinyBVH Rust bindings
The TinyBVH library can now also be used from Rust, with the [Rust bindings](https://docs.rs/tinybvh-rs/latest/tinybvh_rs) provided by David Peicho.

# TinyBVH Python bindings
Florent Le Moël created Python bindings for TinyBVH. You can find details in the [pytinybvh repository](https://github.com/FlorentLM/pytinybvh) on GitHub.

# TinyBVH .NET wrapper
Anders Forsgren provides [TinyBVHNet](https://github.com/andersforsgren/TinyBVHNet), a .NET wrapper for TinyBVH, targetting ```net48``` and ```net10.0```. Get it via [NuGet](https://www.nuget.org/packages/TinyBVHNet).

Created or know about other projects? [Let me know](mailto:bikker.j@protonmail.com)!

# Contribute
TinyBVH received features and bug fixes from [32 contributors](https://github.com/jbikker/tinybvh/graphs/contributors?all=1) so far. Thanks! Feel free to submit PRs; **please do so in the [development branch](https://github.com/jbikker/tinybvh/tree/dev)** for a good workflow.

# Contact
Questions, remarks? Contact me at bikker.j@protonmail.com or BlueSky: @jbikker.bsky.social .

# License
This library is made available under the MIT license, which starts as follows: "Permission is hereby granted, free of charge, .. , to deal in the Software **without restriction**". Enjoy. If you are using this work in your research, please cite TinyBVH: Details are available in BibTeX and APA format, see the 'About' section for this repo on Github.

# Acknowledgement
The development of this library is supported by an AMD hardware grant.


<br><br>
  
![Student work: Tamara Heeffer](images/tamara.jpg)
<p align="center"><i>Image credit: Tamara Heeffer, IGAD / Breda University</i> </p>

<br><br>
![Student work: Hesam Ghadimi](images/hesam.jpg)
<p align="center"><i>Image credit: Hesam Ghadimi, IGAD / Breda University</i> </p>
