#!/bin/bash

# Usage:
#   ./build.sh [RELEASE] [DEFINE1] [DEFINE2] ... [CMAKE_VAR=VALUE] ...
# Notes:
#   - RELEASE sets CMAKE_BUILD_TYPE=RELEASE (default is DEBUG)
#   - Any bare token like DEBUGXX/DDEBUG is passed to the compiler as -D<token>
#   - Any argument containing '=' is passed to CMake as -D<arg> (e.g. FOO=ON)

# Default build type is DEBUG
BUILD_TYPE="DEBUG"
ADDITIONAL_DEFINES=""
 EXTRA_CFLAGS=""

# Check for RELEASE parameter
if [ "$1" = "RELEASE" ]; then
    BUILD_TYPE="RELEASE"
    shift  # Remove RELEASE from arguments
fi

# Add any additional defines from remaining parameters
for arg in "$@"; do
    case "$arg" in
        *=*) ADDITIONAL_DEFINES="$ADDITIONAL_DEFINES -D$arg" ;;
        *)   EXTRA_CFLAGS="$EXTRA_CFLAGS -D$arg" ;;
    esac
done

rm -rf ./logs
rm -rf ./build
mkdir build
cd build
export CFLAGS="$CFLAGS $EXTRA_CFLAGS"
export CXXFLAGS="$CXXFLAGS $EXTRA_CFLAGS"
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON $ADDITIONAL_DEFINES ../
make

