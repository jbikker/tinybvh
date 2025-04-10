# dev
This is the **development branch** for TinyBVH. Changes are tested here first.

# tinybvh
Single-header BVH construction and traversal library written as "Sane C++" (or "C with classes"). Some C++11 is used, e.g. for threading. The library has no dependencies. 

# tinyocl
Single-header OpenCL library, which helps you select and initialize a device. It also loads, compiles and runs kernels, with several convenient features:
* Include-file expansion for AMD devices
* Multi-argument passing
* Host/device buffer management
* Vendor and architecture detection and propagation to #defines in OpenCL code
* ..And many other things.

![Bistro](images/invasion.png)

To use tinyocl, just include ````tiny_ocl.h````; this will automatically cause linking with ````OpenCL.lib```` in the 'external' folder, which in turn passes on work to vendor-specific driver code. But all that is not your problem!

Note that the ````tiny_bvh.h```` library will work without ````tiny_ocl.h```` and remains dependency-free. The new ````tiny_ocl.h```` is only needed in projects that wish to trace rays _on the GPU_ using BVHs created by ````tiny_bvh.h````. 
  
# BVH?
A Bounding Volume Hierarchy is a data structure used to quickly find intersections in a virtual scene; most commonly between a ray and a group of triangles. You can read more about this in a series of articles on the subject: https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics .

Right now tinybvh comes with the following builders:
* ````BVH::Build```` : Efficient plain-C/C+ binned SAH BVH builder which should run on any platform.
* ````BVH::BuildAVX```` : A highly optimized version of BVH::Build for Intel CPUs.
* ````BVH::BuildHQ```` : A 'spatial splits' BVH builder, for highest BVH quality.

A constructed BVH can be used to quickly intersect a ray with the geometry, using ````BVH::Intersect```` or ````BVH::IsOccluded````, for shadow rays. The double-precision BVH is traversed using ````BVH::IntersectEx````.

Apart from the default BVH layout (simply named ````BVH````), several other layouts are available, which all serve one or more specific purposes. You can create a BVH in the desired layout by instantiating the appropriate class, or by converting from ````BVH```` using the ````::ConvertFrom```` methods. The available layouts are:
* ````BVH```` : A compact format that stores the AABB for a node, along with child pointers and leaf information in a cross-platform-friendly way. The 32-byte size allows for cache-line alignment.
* ````BVH_SoA```` : This format stores bounding box information in a SIMD-friendly format, making the BVH faster to traverse.
* ````BVH_Double```` : Double-precision version of ````BVH````.
* ````BVH_GPU```` : This format uses 64 bytes per node and stores the AABBs of the two child nodes. This is the format presented in the [2009 Aila & Laine paper](https://research.nvidia.com/sites/default/files/pubs/2009-08_Understanding-the-Efficiency/aila2009hpg_paper.pdf). It can be traversed with a simple GPU kernel.
* ````MBVH<M>```` : In this (templated) format, each node stores M child pointers, reducing the depth of the tree. This improves performance for divergent rays. Based on the [2008 paper](https://graphics.stanford.edu/~boulos/papers/multi_rt08.pdf) by Ingo Wald et al.
* ````BVH4_GPU```` : A more compact version of the ````BVH4```` format, which will be faster for GPU ray tracing.
* ````BVH4_CPU```` : A SIMD-friendly version of the ````BVH4```` format.
* ````BVH8_CPU```` : AVX2-optimized wide BVH traversal. Currently (by far) the fastest option on CPU.
* ````BVH8_CWBVH```` : An advanced 80-byte representation of the 8-wide BVH, for state-of-the-art GPU rendering, based on the [2017 paper](https://research.nvidia.com/publication/2017-07_efficient-incoherent-ray-traversal-gpus-through-compressed-wide-bvhs) by Ylitie et al. and [code by AlanWBFT](https://github.com/AlanIWBFT/CWBVH).

A BVH in the ````BVH```` format may be _refitted_, in case the triangles moved, using ````BVH::Refit````. Refitting is substantially faster than rebuilding and works well if the animation is subtle. Refitting does not work if polygon counts change.

New in version 1.1.3: Most layouts may be serialized and de-serialized via ````::Save```` and ````::Load````.

A more complete overview of tinybvh functionality can be found in the [Basic Use Manual](https://jacco.ompf2.com/2025/01/24/tinybvh-manual-basic-use) and the [Advanced Topics Manual](https://jacco.ompf2.com/2025/01/25/tinybvh-manual-advanced).

# How To Use
The library ````tiny_bvh.h```` is designed to be easy to use. Please have a look at tiny_bvh_minimal.cpp for an example. A Visual Studio 'solution' (.sln/.vcxproj) is included, as well as a CMake file. That being said: The examples consists of only a single source file, which can be compiled with clang or g++, e.g.:

````g++ tiny_bvh_minimal.cpp````

The single-source sample **ASCII test renderer** can be compiled with

````g++ tiny_bvh_renderer.cpp -o tiny_bvh_renderer````

The cross-platform fenster-based single-source **bitmap renderer** can be compiled with

````g++ -mwindows -O3 tiny_bvh_fenster.cpp -o tiny_bvh_fenster```` (on windows)

````g++ -framework -O3 Cocoa tiny_bvh_fenster.cpp -o tiny_bvh_fenster```` (on macOS)

The multi-threaded **path tracing** demo can be compiled with

````g++ -mwindows -O3 tiny_bvh_pt.cpp -o tiny_bvh_pt```` (on windows)

The **performance measurement tool** can be compiled with:

````g++ -mavx2 -mfma -Ofast tiny_bvh_speedtest.cpp -o tiny_bvh_speedtest````

# Version 1.5.1

Version 1.5.0 introduced a new fast layout for x86/x64 systems that do not (or cannot be presumed to) support AVX2. For those, please use BVH4_CPU form optimal performance (about 80% of the fastest AVX2 code).

Version 1.4.0 introduced a new BVH layout for fast single-ray traversal on CPU: BVH8_CPU. This supersedes the previous fastest scheme, BVH4_CPU. 

Version 1.1.0 introduced a <ins>change to the API</ins>. The single BVH class with multiple layouts has been replaced with a BVH class per layout. You can simply instantiate the desired layout; conversion (and data ownership) is then handled properly by the library. Examples:

````
BVH bvh;
bvh.Build( (bvhvec4*)myTriData, triangleCount ); // or: BuildHQ( .. )
bvh.Intersect( ray );
````

````
BVH4_CPU bvh;
bvh.Build( (bvhvec4*)myTriData, triangleCount );
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
bvh.BuildHQ( verts, indices, triCount );
BVH_Verbose tmp;
tmp.ConvertFrom( bvh );
tmp.Optimize( 100 );
bvh.ConvertFrom( tmp );
printf( "Optimized BVH SAH cost: %f\n", bvh.SAHCost() );
````

Note that in this case, data ownership and lifetime must be managed carefully. Specifically, layouts converted from other layouts use data from the original, so both must be kept alive.

This version of the library includes the following functionality:
* Reference binned SAH BVH builder
* Fast binned SAH BVH builder using AVX intrinsics
* Fast binned SAH BVH builder using NEON intrinsices, by [wuyakuma](https://github.com/wuyakuma)
* Customizable SAH parameters
* TLAS builder with instancing and fast TLAS/BLAS traversal, even for 'mixed trees'
* TLAS masking (similar to [OptiX](https://raytracing-docs.nvidia.com/optix7/guide/optix_guide.230712.A4.pdf)), by [Romain Augier](https://github.com/romainaugier).
* Double-precision binned SAH BVH builder
* Support for custom geometry and mixed scenes
* Example code for GPU TLAS/BLAS traversal (dragon invasion demo, tiny_bvh_gpu2.cpp)
* Example TLAS/BLAS application using OpenGL interop (windows only)
* Spatial Splits ([SBVH](https://www.nvidia.in/docs/IO/77714/sbvh.pdf), Stich et al., 2009) builder, including "unsplitting"
* BVH optimizer: reduces SAH cost and improves ray tracing performance ([Bittner et al., 2013](https://dspace.cvut.cz/bitstream/handle/10467/15603/2013-Fast-Insertion-Based-Optimization-of-Bounding-Volume-Hierarchies.pdf))
* Collapse to N-wide MBVH using templated code
* Conversion of 4-wide BVH to GPU-friendly 64-byte quantized format
* 'Compressed Wide BVH' (CWBVH) data structure
* Single-ray and packet traversal
* Sphere/BVH collision detection via BVH::IntersectSphere(..)
* BVH (de)serialization for most layouts
* Fast AVX2 ray tracing: Implements the 2017 paper by [Fuetterling et al.](https://web.cs.ucdavis.edu/~hamann/FuetterlingLojewskiPfreundtHamannEbertHPG2017PaperFinal06222017.pdf)
* Fast SSE4.2 ray tracing: A modified version of the AVX2 implementation using just SSE4.2 achieves 80% of AVX2 performance.
* Fast triangle intersection: Implements the 2016 paper by [Baldwin & Weber](https://jcgt.org/published/0005/03/03/paper.pdf)
* 'Watertight' ray/triangle intersection, based on the [paper](https://jcgt.org/published/0002/01/05/paper.pdf) by Woop et al.
* OpenCL traversal example code: Aila & Laine, 4-way quantized, CWBVH
* OpenCL support for MacOS, by [wuyakuma](https://github.com/wuyakuma)
* Support for WASM / EMSCRIPTEN, g++, clang, Visual Studio
* Optional user-defined memory allocation, by [Thierry Cantenot](https://github.com/tcantenot)
* Vertex array can now have a custom stride, by [David Peicho](https://github.com/DavidPeicho)
* Vertex array can now be indexed
* Custom primitives can be intersected via callbacks, also in double-precision BVHs
* Clear data ownership and intuitive management via the new and simplified API, with lots of help from David Peicho
* You can now also BYOVT ('bring your own vector types'), thanks [Tijmen Verhoef](https://github.com/nemjit001)
* 'SpeedTest' tool that times and validates all (well, most) traversal kernels
* A [manual](https://jacco.ompf2.com/2025/01/24/tinybvh-manual-basic-use) is now available.

The current version of the library is rapidly gaining functionality. Please expect changes to the interface.

Plans, ordered by priority:

* Speed improvements:
  * Faster optimizer for AVX-capable CPUs
  * Improve speed of SBVH builder
* Features & outstanding issues:
  * 'Watertight' triangle intersection option
  * Use PARANOID flag to check NaNs and more
* Demo of tinybvh on GPU using other apis:
  * Ray tracing in pure OpenGL
  * Ray tracing in pure DirectX
  * SDL3 sample application
* Bridge to rt hw / layouts:
  * Produce a BVH for Intel rt hw (mind the quads)
  * Produce a BVH for AMD rt hw
  * Use inline asm on AMD for aabb/tri intersect
* Comparisons / experiments:
  * Memory use analysis in speedtest
  * DXR renderer to compare against hw rt
* CPU single-ray performance
  * Experiment with 1, 4 and 8 tris in BVH8_CPU
  * Reverse-engineer Embree & PhysX
  * Combination of TLAS and packet traversal
* Ease-of-use
  * Robust default origin offset
  * Engine layer
  
# tinybvh in the Wild
A list of projects using tinybvh:
* [unity-tinybvh](https://github.com/andr3wmac/unity-tinybvh): An example implementation for tinybvh in Unity and a foundation for building compute based raytracing solutions, by Andrew MacIntyre.
* [TrenchBroomBFG](https://github.com/RobertBeckebans/TrenchBroomBFG), by Robert Beckebans. "TinyBVH allows to load bigger glTF 2 maps almost instantly instead of minutes". 

# tinybvh Rust bindings
The tinybvh library can now also be used from Rust, with the [Rust bindings](https://docs.rs/tinybvh-rs/latest/tinybvh_rs) provided by David Peicho.

Created or know about other projects? [Let me know](mailto:bikker.j@gmail.com)!

# Contact
Questions, remarks? Contact me at bikker.j@gmail.com ~~or on twitter: @j_bikker,~~ or BlueSky: @jbikker.bsky.social .

# License
This library is made available under the MIT license, which starts as follows: "Permission is hereby granted, free of charge, .. , to deal in the Software **without restriction**". Enjoy. If you are using this work in your research, please cite TinyBVH: Details are available in BibTeX and APA format, see the 'About' section for this repo on Github.


![Output of the sample wavefront path tracer](images/wavefront.png)

![Rendered with tinybvh](images/test.png)
