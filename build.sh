#!/usr/bin/env bash

cmake -B ./build -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON
cmake --build ./build --config Release
cd build
ctest -C Release --output-on-failure -E "etl_unit_tests"
