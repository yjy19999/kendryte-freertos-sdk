#!/bin/bash
cd build
cmake .. -DPROJ=hello_world
make clean
make -j32