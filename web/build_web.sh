#!/bin/sh
# Builds the page's generator, tinybvh_custom.js, from tinybvh_custom.cpp with Emscripten, and
# copies the six TinyBVH headers it works on into headers/. Run it from this folder, with em++ on
# the path (emsdk). The generator and the headers must come from the same release of TinyBVH: the
# generator's rules match the headers' exact text.
#
#   sh build_web.sh [path to tinybvh_custom.cpp] [folder with the six headers]
#
# The defaults suit a staging folder in the repository root, such as the workflow's _site.
set -e
SRC=${1:-../tools/tinybvh_custom.cpp}
HEADERS=${2:-..}

# -fwasm-exceptions: the generator reports malformed headers with an exception; WebAssembly's own
# exception handling keeps that working and is about 2.5 times faster than Emscripten's emulation.
# SINGLE_FILE embeds the WebAssembly in the .js file, so the site has one file less to serve.
em++ -std=c++17 -O3 -fwasm-exceptions "$SRC" -o tinybvh_custom.js --no-entry \
	-sSINGLE_FILE=1 -sMODULARIZE=1 -sEXPORT_NAME=createTinyBVHCustom -sFORCE_FILESYSTEM=1 \
	-sEXPORTED_FUNCTIONS=_tinybvh_custom_run -sEXPORTED_RUNTIME_METHODS=ccall,FS -sALLOW_MEMORY_GROWTH=1

mkdir -p headers
for h in tiny_bvh.h tiny_bvh_base.h tiny_bvh_x86_float.h tiny_bvh_x86_double.h tiny_bvh_arm_float.h tiny_bvh_arm_double.h; do
	cp "$HEADERS/$h" headers/
done
echo "built tinybvh_custom.js; copied the six headers into headers/"
