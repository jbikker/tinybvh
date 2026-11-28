#!/bin/sh
# Compiles traverse_vk.comp into the SPIR-V that benchmark/vulkan_rt.cpp embeds (traverse_vk_*.inc).
# Only needed after changing the shader; needs glslc (Vulkan SDK). vulkan1.2: ray queries need SPIR-V 1.4.
set -e
cd "$(dirname "$0")"
GLSLC=${VULKAN_SDK:+$VULKAN_SDK/bin/}glslc
$GLSLC --target-env=vulkan1.2 -O -mfmt=c traverse_vk.comp -o traverse_vk_nearest.inc
$GLSLC --target-env=vulkan1.2 -O -mfmt=c -DANY_HIT traverse_vk.comp -o traverse_vk_any.inc
echo compiled traverse_vk_nearest.inc and traverse_vk_any.inc
