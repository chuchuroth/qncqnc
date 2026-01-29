# ModbusRTU Protocol Docker Image
#
# Multi-stage build for ModbusRTU protocol component
# Supports both standalone (header-only) and DDS-enabled builds
#
# Build:
#   docker build -t modbus-rtu-protocol .
#   docker build --build-arg BUILD_WITH_DDS=ON -t modbus-rtu-protocol:dds .
#
# Run:
#   docker run --rm -it --device=/dev/ttyUSB0 modbus-rtu-protocol /dev/ttyUSB0 1 115200

ARG BASE_IMAGE=ubuntu:22.04

#############################################
# Stage 1: Build dependencies
#############################################
FROM ${BASE_IMAGE} AS builder

ARG DEBIAN_FRONTEND=noninteractive
ARG BUILD_WITH_DDS=OFF

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    git \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Optional: Install FastDDS for DDS support
RUN if [ "$BUILD_WITH_DDS" = "ON" ]; then \
    apt-get update && apt-get install -y --no-install-recommends \
        libfastcdr-dev \
        libfastdds-dev \
        fastdds-tools \
    && rm -rf /var/lib/apt/lists/*; \
    fi

# Create working directory
WORKDIR /build

# Copy source files
COPY include/ /build/include/
COPY idl/ /build/idl/
COPY schemas/ /build/schemas/
COPY descriptors/ /build/descriptors/
COPY examples/ /build/examples/
COPY CMakeLists.txt /build/

# Build
RUN mkdir -p /build/build && cd /build/build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=ON \
        -DBUILD_WITH_DDS=${BUILD_WITH_DDS} \
        -DENABLE_LOGGING=ON \
    && make -j$(nproc)

#############################################
# Stage 2: Runtime image
#############################################
FROM ${BASE_IMAGE} AS runtime

ARG DEBIAN_FRONTEND=noninteractive

# Install minimal runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user
RUN useradd -m -s /bin/bash modbus

# Create directories
RUN mkdir -p /opt/modbus_rtu/bin \
             /opt/modbus_rtu/share/descriptors \
             /opt/modbus_rtu/share/schemas \
             /opt/modbus_rtu/share/idl

# Copy built binaries and resources
COPY --from=builder /build/build/modbus_rtu_example /opt/modbus_rtu/bin/
COPY --from=builder /build/descriptors/ /opt/modbus_rtu/share/descriptors/
COPY --from=builder /build/schemas/ /opt/modbus_rtu/share/schemas/
COPY --from=builder /build/idl/ /opt/modbus_rtu/share/idl/
COPY --from=builder /build/include/ /opt/modbus_rtu/include/

# Set permissions
RUN chown -R modbus:modbus /opt/modbus_rtu

# Add to PATH
ENV PATH="/opt/modbus_rtu/bin:${PATH}"

# Working directory
WORKDIR /opt/modbus_rtu

# Switch to non-root user
USER modbus

# Default entrypoint
ENTRYPOINT ["modbus_rtu_example"]
CMD ["--help"]

# Labels
LABEL maintainer="NeuraSync <info@neurasync.io>"
LABEL description="ModbusRTU Protocol Component - Device-agnostic Modbus RTU communication"
LABEL version="1.0.0"

#############################################
# Stage 3: Development image (optional)
#############################################
FROM builder AS development

ARG DEBIAN_FRONTEND=noninteractive

# Install additional development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    gdb \
    valgrind \
    vim \
    less \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python tools for testing
RUN pip3 install --no-cache-dir pymodbus pyserial

WORKDIR /build

# Keep container running for development
CMD ["/bin/bash"]
