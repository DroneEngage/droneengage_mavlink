#!/bin/bash
rm -rf ./build
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON  ../
make 

