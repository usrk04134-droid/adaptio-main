#!/bin/bash

# Script to run BigSnake::Parse() unit tests
# This script builds and runs the unit tests for BigSnake image processing

set -e

WORKSPACE_ROOT="/workspace"
BUILD_DIR="$WORKSPACE_ROOT/build"

echo "================================"
echo "BigSnake::Parse() Unit Tests"
echo "================================"

# Check if we're using Nix
if command -v nix &> /dev/null; then
    echo "Building with Nix..."
    cd "$WORKSPACE_ROOT"
    nix build .#adaptio-unit-tests
    
    if [ -f "./result/bin/adaptio-unit-tests" ]; then
        echo "Running BigSnake tests..."
        ./result/bin/adaptio-unit-tests --test-case="*BigSnake*"
    else
        echo "Error: Unit test executable not found after Nix build"
        exit 1
    fi
else
    echo "Nix not found, attempting CMake build..."
    
    # Create build directory if it doesn't exist
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    
    # Configure with CMake
    echo "Configuring project..."
    cmake .. -DCMAKE_BUILD_TYPE=Debug
    
    # Build unit tests
    echo "Building unit tests..."
    make adaptio-unit-tests -j$(nproc)
    
    # Run BigSnake tests
    if [ -f "./src/adaptio-unit-tests" ]; then
        echo "Running BigSnake tests..."
        ./src/adaptio-unit-tests --test-case="*BigSnake*"
    else
        echo "Error: Unit test executable not found"
        exit 1
    fi
fi

echo "================================"
echo "Test execution completed"
echo "================================"