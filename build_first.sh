#!/bin/bash
cd build
rm CMakeCache.txt
cmake .. -DPROJ=hello_world
make clean
make -j32