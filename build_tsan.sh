#!/bin/bash
# Build with ThreadSanitizer

mkdir -p build_tsan
cd build_tsan

cmake .. \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-fsanitize=thread -g -O1" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=thread"

make -j4 VoxelTests
