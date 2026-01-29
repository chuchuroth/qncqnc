#!/bin/bash
#
# Build script for ModbusRTU Protocol Component
#
# Usage:
#   ./scripts/build.sh              # Build standalone (no DDS)
#   ./scripts/build.sh --dds        # Build with DDS support
#   ./scripts/build.sh --docker     # Build Docker image
#   ./scripts/build.sh --clean      # Clean build directory
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"

# Default options
BUILD_TYPE="Release"
BUILD_DDS="OFF"
BUILD_DOCKER=""
CLEAN_BUILD=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --dds)
            BUILD_DDS="ON"
            shift
            ;;
        --docker)
            BUILD_DOCKER="yes"
            shift
            ;;
        --clean)
            CLEAN_BUILD="yes"
            shift
            ;;
        --debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

echo "==========================================="
echo "ModbusRTU Protocol Build"
echo "==========================================="
echo "Project dir: $PROJECT_DIR"
echo "Build type:  $BUILD_TYPE"
echo "DDS support: $BUILD_DDS"
echo ""

# Clean if requested
if [[ -n "$CLEAN_BUILD" ]]; then
    echo "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
fi

# Docker build
if [[ -n "$BUILD_DOCKER" ]]; then
    echo "Building Docker image..."
    cd "$PROJECT_DIR"

    if [[ "$BUILD_DDS" == "ON" ]]; then
        docker build \
            --build-arg BUILD_WITH_DDS=ON \
            -t modbus-rtu-protocol:dds \
            -t modbus-rtu-protocol:latest \
            .
    else
        docker build \
            -t modbus-rtu-protocol:latest \
            .
    fi

    echo ""
    echo "Docker image built successfully!"
    echo "Run with: docker run --rm -it --device=/dev/ttyUSB0 modbus-rtu-protocol /dev/ttyUSB0"
    exit 0
fi

# Native build
echo "Creating build directory..."
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

echo "Configuring with CMake..."
cmake "$PROJECT_DIR" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_WITH_DDS="$BUILD_DDS" \
    -DENABLE_LOGGING=ON

echo ""
echo "Building..."
make -j$(nproc)

echo ""
echo "==========================================="
echo "Build complete!"
echo "==========================================="
echo ""
echo "Executable: $BUILD_DIR/modbus_rtu_example"
echo ""
echo "Usage:"
echo "  $BUILD_DIR/modbus_rtu_example /dev/ttyUSB0 [slave_id] [baudrate]"
echo ""
