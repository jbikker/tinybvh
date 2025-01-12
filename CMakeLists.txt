cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(tiny_bvh LANGUAGES CXX)

if (APPLE)
	find_library(COCOA_LIBRARY Cocoa)
elseif (UNIX AND NOT EMSCRIPTEN)
	find_package(X11)
elseif (EMSCRIPTEN)
	# To auto generate test pages
	set(CMAKE_EXECUTABLE_SUFFIX ".html")

	# Enable clang's Address Sanitizer for WASM
	option(enable_asan "Enable ASAN on the build (it'll slow down the app)" OFF)

	# Enable wasm SIMD instructions
	set(emcc_simd "fixed" CACHE STRING "SIMD type to use in the build (none, fixed, relaxed)")
	set_property(CACHE emcc_simd PROPERTY STRINGS none fixed relaxed)

	# Enable thread support for WASM (note that you need special http headers now https://emscripten.org/docs/porting/pthreads.html )
	option(emcc_threads "Enable Threads in WASM (note that means you need to change your http headers)" OFF)

	# Enable C/C++ main thread as a proxy thread to avoid blocking DOM/UI thread https://emscripten.org/docs/porting/pthreads.html
	option(emcc_proxy_main "Enable C/C++ main thread as a proxy thread to avoid blocking DOM/UI thread" ${emcc_threads})
endif()

enable_testing()

add_executable(tiny_bvh_minimal tiny_bvh_minimal.cpp)
add_executable(tiny_bvh_renderer tiny_bvh_renderer.cpp)
add_executable(tiny_bvh_speedtest tiny_bvh_speedtest.cpp)
if (NOT EMSCRIPTEN) # EMSCRIPTEN doesn't render anything by default (you would need WebGL/WebGPU)
	add_executable(tiny_bvh_fenster tiny_bvh_fenster.cpp)
endif()

target_include_directories(tiny_bvh_speedtest PRIVATE ${CMAKE_CURRENT_LIST_DIR}/external/OpenCL/inc)

if (NOT MSVC)
	# Produce debug symbols and set optimization level
	set(common_cxx_flags
		$<$<OR:$<CONFIG:RelWithDebInfo>,$<CONFIG:Debug>>:-g> $<$<OR:$<CONFIG:RelWithDebInfo>,$<CONFIG:Release>>:-O3>
	)

	set(common_link_flags
		$<$<OR:$<CONFIG:RelWithDebInfo>,$<CONFIG:Debug>>:-g>
	)

	if (NOT EMSCRIPTEN)
		# EMSCRIPTEN doesn't know about archs
		set(common_cxx_flags
			${common_cxx_flags} -march=native
		)
	else()
		# EMSCRIPTEN flags:
		# -sALLOW_MEMORY_GROWTH (allow runtime allocations) (for some reason cmake passes this as a linker flag when in theory it's a compiler one)
		set(common_link_flags ${common_link_flags} -sALLOW_MEMORY_GROWTH=1)

		# -fsanitize=address (enable ASAN)
		# More at https://emscripten.org/docs/debugging/Sanitizers.html#address-sanitizer 
		if (enable_asan) 
			set(common_cxx_flags ${common_cxx_flags} -fsanitize=address)
			set(common_link_flags ${common_link_flags} -fsanitize=address)
		endif()

		# Enable SIMD instructions and AVX intrinsics
		string(TOLOWER "${emcc_simd}" emcc_simd_lower)
		if (emcc_simd_lower STREQUAL fixed)
			# Supported by all browsers https://webassembly.org/features/
			set(common_cxx_flags ${common_cxx_flags} -msimd128 -mavx)
		elseif (emcc_simd_lower STREQUAL relaxed)
			# Supported some browsers https://webassembly.org/features/
			set(common_cxx_flags ${common_cxx_flags} -mrelaxed-simd -mavx)
		endif()

	endif()

	target_compile_options(tiny_bvh_minimal PRIVATE ${common_cxx_flags})
	target_link_options(tiny_bvh_minimal PRIVATE ${common_link_flags})

	target_compile_options(tiny_bvh_renderer PRIVATE ${common_cxx_flags})
	target_link_options(tiny_bvh_renderer PRIVATE ${common_link_flags})

	# No openmp support in default compiler
	set(tiny_bvh_speedtest_cxx_flags ${common_cxx_flags})
	set(tiny_bvh_speedtest_link_flags ${common_link_flags})

	if (NOT APPLE AND NOT EMSCRIPTEN)
		set(tiny_bvh_speedtest_cxx_flags ${tiny_bvh_speedtest_cxx_flags})
		set(tiny_bvh_speedtest_link_flags ${tiny_bvh_speedtest_link_flags})
		target_link_libraries(tiny_bvh_speedtest ${CMAKE_CURRENT_LIST_DIR}/external/OpenCL/lib/OpenCL.lib)
	elseif (EMSCRIPTEN)
		# Embed Sponza into the wasm to load it (in theory there are other methods, but this is the simplest)
		# See https://emscripten.org/docs/porting/files/packaging_files.html#modifying-file-locations-in-the-virtual-file-system
		set(tiny_bvh_speedtest_link_flags ${tiny_bvh_speedtest_link_flags} --preload-file "${CMAKE_CURRENT_LIST_DIR}/testdata@/testdata")

		if (emcc_threads)
			set(tiny_bvh_speedtest_cxx_flags ${tiny_bvh_speedtest_cxx_flags} -pthread)
			set(tiny_bvh_speedtest_link_flags ${tiny_bvh_speedtest_link_flags} -pthread -sUSE_PTHREADS=1 -sPTHREAD_POOL_SIZE=navigator.hardwareConcurrency)

			if (FALSE)
				# From:
				#	- https://github.com/llvm/llvm-project/pull/95169 
				#	- https://github.com/llvm/llvm-project/pull/71297 
				#	- https://reviews.llvm.org/D142593?id=492403
				# llvm's and clang's OpenMP has been ported to WASM around August 2024
				# Sadly emcc's clang doesn't include a copy of the lib
				# So you would need to do compile it yourself https://github.com/abrown/wasm-openmp-examples/blob/main/emscripten.sh
				# Then link it here
				set(tiny_bvh_speedtest_cxx_flags ${tiny_bvh_speedtest_cxx_flags} -fopenmp)
				set(tiny_bvh_speedtest_link_flags ${tiny_bvh_speedtest_link_flags} -fopenmp)
			endif()

			if (emcc_proxy_main)
				set(tiny_bvh_speedtest_link_flags ${tiny_bvh_speedtest_link_flags} -sPROXY_TO_PTHREAD=1)
			endif()
		endif()
	endif()
	target_compile_options(tiny_bvh_speedtest PRIVATE ${tiny_bvh_speedtest_cxx_flags})
	target_link_options(tiny_bvh_speedtest PRIVATE ${tiny_bvh_speedtest_link_flags})
	
	if (TARGET tiny_bvh_fenster)
		# Not all platforms support all targets. E. g.:
		#	- EMSCRIPTEN doesn't render anything by default (you would need WebGL/WebGPU)
		target_compile_options(tiny_bvh_fenster PRIVATE ${common_cxx_flags})
		target_link_options(tiny_bvh_fenster PRIVATE ${common_link_flags})
		if (WIN32)
			target_link_libraries(tiny_bvh_fenster -mwindows)
		elseif (APPLE)
			target_link_libraries(tiny_bvh_fenster ${COCOA_LIBRARY})
		elseif (X11_FOUND)
			target_link_libraries(tiny_bvh_fenster ${X11_LIBRARIES})
		endif()
	endif()
else()
	if (WIN32)
		set_target_properties(tiny_bvh_fenster PROPERTIES WIN32_EXECUTABLE TRUE)
	endif()
endif()

add_test(NAME "tiny_bvh_minimal" COMMAND tiny_bvh_minimal)
