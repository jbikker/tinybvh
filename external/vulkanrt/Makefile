# Linux build for the Vulkan tinybvh ray tracing benchmark.
#
#   make            build the shaders and the binary
#   make run        ... and run it from this directory
#   make clean
#
# Needs: Vulkan headers + loader, libxcb, and glslangValidator.
#   Debian/Ubuntu  sudo apt install libvulkan-dev libxcb1-dev glslang-tools
#   Fedora         sudo dnf install vulkan-loader-devel libxcb-devel glslang
#   Arch           sudo pacman -S vulkan-headers libxcb glslang
# The LunarG SDK works too; it puts glslangValidator on PATH. Whichever route,
# a driver with VK_KHR_ray_tracing_pipeline is required at run time
# (NVIDIA proprietary, or Mesa RADV / ANV recent enough).

CXX      ?= g++
GLSLANG  ?= glslangValidator
TARGET   := program_vk

# tinybvh's x86 kernels want AVX2/FMA; skip those on other architectures so the
# same Makefile works on aarch64, where tinybvh uses its NEON path instead.
ARCH := $(shell uname -m)
ifneq (,$(filter $(ARCH),x86_64 amd64))
  ARCHFLAGS := -mavx2 -mfma
endif

CXXFLAGS ?= -std=c++20 -O2 -Wall -pthread $(ARCHFLAGS)
LDLIBS   := -lvulkan -lxcb

SHADERS := trace.rgen trace.rmiss trace.rchit tiny.comp tiny4.comp tiny8.comp tinyrq.comp
SPV     := $(addprefix shaders/,$(addsuffix .spv,$(SHADERS)))

all: $(SPV) $(TARGET)

# wildcard, not a literal path: if this folder is not two levels below the
# tinybvh root the dependency quietly drops out and you get a readable #include
# error, rather than make refusing with "No rule to make target".
TINYBVH := $(wildcard ../../tiny_bvh*.h)

$(TARGET): program_vk.cpp $(TINYBVH)
	$(CXX) $(CXXFLAGS) -o $@ program_vk.cpp $(LDLIBS)

# vulkan1.2 is required rather than cosmetic: ray tracing and ray query need
# SPIR-V 1.4, which the vulkan1.0/1.1 targets cannot emit.
shaders/%.spv: shaders/% shaders/common.glsl
	$(GLSLANG) --target-env vulkan1.2 -Ishaders -o $@ $<

# The scene, the ray set and the shaders are all found relative to the working
# directory, so run from here rather than from an install prefix.
run: all
	./$(TARGET)

clean:
	rm -f $(TARGET) $(SPV)

.PHONY: all run clean
