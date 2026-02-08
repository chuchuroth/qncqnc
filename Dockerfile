# ModbusRTU Protocol Docker Image with NeuraSync Support
#
# Build:
#   docker build -t modbus-rtu-protocol:neurasync .
#
# Run gripper control (publisher):
#   docker run --rm -it --device=/dev/ttyUSB0 modbus-rtu-protocol:neurasync gripper_control /dev/ttyUSB0 1 115200
#
# Run dashboard (subscriber):
#   docker run --rm -it --network=host modbus-rtu-protocol:neurasync gripper_dashboard

FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    git \
    ca-certificates \
    libyaml-cpp-dev \
    libboost-filesystem-dev \
    libasio-dev \
    libtinyxml2-dev \
    libssl-dev \
    default-jdk \
    && rm -rf /var/lib/apt/lists/*

# Create working directory
WORKDIR /build

# Clone and build foonathan_memory_vendor
RUN git clone --depth 1 --branch v1.3.1 https://github.com/eProsima/foonathan_memory_vendor.git && \
    cd foonathan_memory_vendor && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# Clone and build Fast-CDR 2.x
RUN git clone --depth 1 --branch v2.2.6 https://github.com/eProsima/Fast-CDR.git && \
    cd Fast-CDR && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# Clone and build Fast-DDS 3.x
RUN git clone --depth 1 --branch v3.1.0 https://github.com/eProsima/Fast-DDS.git && \
    cd Fast-DDS && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
        -DCOMPILE_EXAMPLES=OFF \
        -DBUILD_TESTING=OFF \
        -DTHIRDPARTY_Asio=FORCE && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# Clone and build Fast-DDS-Gen for IDL generation
RUN git clone --depth 1 --branch v4.0.3 --recursive https://github.com/eProsima/Fast-DDS-Gen.git && \
    cd Fast-DDS-Gen && \
    ./gradlew assemble && \
    mkdir -p /usr/local/share/fastddsgen && \
    cp share/fastddsgen/java/fastddsgen.jar /usr/local/share/fastddsgen/ && \
    echo '#!/bin/bash' > /usr/local/bin/fastddsgen && \
    echo 'java -jar /usr/local/share/fastddsgen/fastddsgen.jar "$@"' >> /usr/local/bin/fastddsgen && \
    chmod +x /usr/local/bin/fastddsgen

# Copy and install neura_sync_idl_tools
COPY neura_sync_idl_tools/ /build/neura_sync_idl_tools/
RUN cd /build/neura_sync_idl_tools && \
    mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make install

# Copy neura-sync source
COPY neura-sync/ /build/neura-sync/

# Build and install neura-sync
RUN cd /build/neura-sync && \
    mkdir -p build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# Copy modbus_rtu_protocol source files
COPY include/ /build/include/
COPY idl/ /build/idl/
COPY schemas/ /build/schemas/
COPY descriptors/ /build/descriptors/
COPY examples/ /build/examples/
COPY CMakeLists.txt /build/

# Build modbus_rtu_protocol with NeuraSync support
RUN mkdir -p /build/build && cd /build/build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=ON \
        -DBUILD_WITH_NEURASYNC=ON \
        -DENABLE_LOGGING=ON \
    && make -j$(nproc)

# Install binaries
RUN mkdir -p /opt/modbus_rtu/bin /opt/modbus_rtu/share && \
    cp /build/build/modbus_rtu_example /opt/modbus_rtu/bin/ && \
    cp /build/build/gripper_control /opt/modbus_rtu/bin/ && \
    cp /build/build/gripper_dashboard /opt/modbus_rtu/bin/ && \
    cp -r /build/descriptors /opt/modbus_rtu/share/ && \
    cp -r /build/schemas /opt/modbus_rtu/share/ && \
    cp -r /build/idl /opt/modbus_rtu/share/

# Cleanup build artifacts to reduce image size
RUN rm -rf /build

# Add to PATH
ENV PATH="/opt/modbus_rtu/bin:${PATH}"

# Working directory
WORKDIR /opt/modbus_rtu

# Default command - shows available binaries
CMD ["sh", "-c", "echo 'Available commands:' && echo '  gripper_control <port> <slave_id> <baudrate>' && echo '  gripper_dashboard' && echo '  modbus_rtu_example' && echo '' && echo 'Example:' && echo '  docker run --device=/dev/ttyUSB0 modbus-rtu-protocol:neurasync gripper_control /dev/ttyUSB0 1 115200'"]

# Labels
LABEL maintainer="NeuraSync <info@neurasync.io>"
LABEL description="ModbusRTU Protocol with NeuraSync - Gripper control with DDS publishing"
LABEL version="1.0.0"
